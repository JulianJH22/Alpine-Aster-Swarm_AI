import numpy as np
import torch
from learning.mappo import MAPPO, encode, gae, ppo_update, PPOConfig

def test_shapes_and_bounds():
    net = MAPPO(hidden=32)
    obs = torch.zeros(6, 14)
    action, logp = net.act(obs)
    assert action.shape == (6, 3)
    assert logp.shape == (6,)
    assert bool((action.abs() <= 1).all())
    state = encode({'arm_1': np.zeros(14, dtype=np.float32)}, ['arm_1'])
    assert state.shape == (90,)
    assert state[14] == 1 and state[29] == 0
    assert net.value(torch.from_numpy(state)).ndim == 0

def test_gae_terminal():
    a, ret = gae(np.array([[1.], [1.]], np.float32), np.zeros((2,1), np.float32),
                 np.zeros((2,1), np.float32), np.array([[0.], [1.]], np.float32), gamma=1, lam=1)
    np.testing.assert_allclose(a[:,0], [2.,1.])
    np.testing.assert_allclose(ret[:,0], [2.,1.])

def test_update_finite():
    net = MAPPO(hidden=32)
    optim = torch.optim.Adam(net.parameters(), lr=1e-3)
    obs = torch.randn(8, 14)
    with torch.no_grad():
        act, logp = net.act(obs)
    batch = {'actor_obs': obs.numpy(), 'actions': act.numpy(), 'old_logp': logp.numpy(),
             'advantages': np.linspace(-1, 1, 8).astype('float32'),
             'global_states': np.random.randn(4, 90).astype('float32'),
             'returns': np.ones(4, dtype='float32')}
    result = ppo_update(net, optim, batch, PPOConfig(epochs=1, minibatch=4))
    assert all(np.isfinite(v) for v in result.values())
