"""MAPPO con actor compartido y critico centralizado (CTDE).

El critico recibe 6 observaciones con ceros para agentes inactivos y una mascara.
La politica es una Normal diagonal con accion limitada mediante tanh; las
log-probabilidades incluyen la correccion del cambio de variable.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
import torch
from torch import nn
from torch.distributions import Normal

MAX_AGENTS = 6
OBS_DIM = 14
ACT_DIM = 3


def mlp(n_in, n_out, hidden=128):
    return nn.Sequential(nn.Linear(n_in, hidden), nn.Tanh(), nn.Linear(hidden, hidden), nn.Tanh(), nn.Linear(hidden, n_out))


class MAPPO(nn.Module):
    def __init__(self, hidden=128):
        super().__init__()
        self.actor = mlp(OBS_DIM, ACT_DIM, hidden)
        self.log_std = nn.Parameter(torch.full((ACT_DIM,), -0.5))
        self.critic = mlp(MAX_AGENTS * (OBS_DIM + 1), 1, hidden)

    def distribution(self, obs):
        mean = self.actor(obs)
        return Normal(mean, self.log_std.clamp(-5, 1).exp().expand_as(mean))

    def act(self, obs, deterministic=False):
        dist = self.distribution(obs)
        raw = dist.mean if deterministic else dist.rsample()
        action = torch.tanh(raw)
        logp = self.log_prob(obs, action)
        return action, logp

    def log_prob(self, obs, action):
        a = action.clamp(-0.999999, 0.999999)
        raw = torch.atanh(a)
        return (self.distribution(obs).log_prob(raw) - torch.log(1 - a.square() + 1e-6)).sum(-1)

    def value(self, global_state):
        return self.critic(global_state).squeeze(-1)


def encode(obs, agent_ids):
    """Estado global de longitud fija, ordenado por indice fisico arm_1..arm_6."""
    state = np.zeros((MAX_AGENTS, OBS_DIM + 1), dtype=np.float32)
    for aid in agent_ids:
        index = int(aid.split('_')[-1]) - 1
        state[index, :OBS_DIM] = obs[aid]
        state[index, OBS_DIM] = 1.0
    return state.reshape(-1)


@dataclass
class PPOConfig:
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_eps: float = 0.2
    entropy_coef: float = 0.001
    value_coef: float = 0.5
    learning_rate: float = 3e-4
    epochs: int = 4
    minibatch: int = 128
    max_grad_norm: float = 0.5


def gae(rewards, values, next_values, dones, gamma=0.99, lam=0.95):
    """GAE temporal [T,N]. dones incluye terminacion o truncamiento.

    next_values se calcula sobre el estado siguiente; las transiciones
    truncadas se pueden bootstrapear si el siguiente estado es observable.
    """
    advantages = np.zeros_like(rewards, dtype=np.float32)
    carry = np.zeros(rewards.shape[1], dtype=np.float32)
    for t in range(len(rewards) - 1, -1, -1):
        nonterminal = 1.0 - dones[t]
        delta = rewards[t] + gamma * next_values[t] * nonterminal - values[t]
        carry = delta + gamma * lam * nonterminal * carry
        advantages[t] = carry
    return advantages, advantages + values


def ppo_update(model, optimizer, batch, cfg: PPOConfig):
    """Actualiza actor con muestras por agente y critico con muestras por paso.

    batch contiene tensores actor_obs, actions, old_logp, advantages,
    global_states, returns, old_values, y actor_step_index.
    """
    device = next(model.parameters()).device
    def t(name):
        return torch.as_tensor(batch[name], dtype=torch.float32, device=device)
    obs, actions, old_logp, advantages = map(t, ('actor_obs', 'actions', 'old_logp', 'advantages'))
    states, returns = map(t, ('global_states', 'returns'))
    adv = (advantages - advantages.mean()) / (advantages.std(unbiased=False) + 1e-8)
    n_actor = len(obs)
    n_critic = len(states)
    if not n_actor or not n_critic:
        raise ValueError('Rollout vacio')
    metrics = []
    for _ in range(cfg.epochs):
        actor_perm = torch.randperm(n_actor, device=device)
        critic_perm = torch.randperm(n_critic, device=device)
        n_chunks = max(1, (n_actor + cfg.minibatch - 1) // cfg.minibatch)
        for k in range(n_chunks):
            ai = actor_perm[k * cfg.minibatch:(k + 1) * cfg.minibatch]
            vi = critic_perm[k * n_critic // n_chunks:(k + 1) * n_critic // n_chunks]
            if len(vi) == 0:
                vi = critic_perm[:1]
            logp = model.log_prob(obs[ai], actions[ai])
            ratio = torch.exp(logp - old_logp[ai])
            surrogate = torch.minimum(ratio * adv[ai], ratio.clamp(1-cfg.clip_eps, 1+cfg.clip_eps) * adv[ai])
            actor_loss = -surrogate.mean()
            value_loss = (model.value(states[vi]) - returns[vi]).square().mean()
            entropy = model.distribution(obs[ai]).entropy().sum(-1).mean()
            loss = actor_loss + cfg.value_coef * value_loss - cfg.entropy_coef * entropy
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), cfg.max_grad_norm)
            optimizer.step()
            metrics.append((float(actor_loss.detach()), float(value_loss.detach())))
    return {'actor_loss': float(np.mean([m[0] for m in metrics])), 'value_loss': float(np.mean([m[1] for m in metrics]))}
