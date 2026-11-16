from control_tareas_54 import execute

class Client:
    def __init__(self):
        self.calls = []
        self.states = {f'arm_{i}': {'id': f'arm_{i}', 'available': True, 'status': 'idle', 'tcp': {'position': [0.2, 0.0, 0.2]}} for i in range(1,7)}
    def call(self, op, **kwargs):
        self.calls.append((op, kwargs))
        if op == 'snapshot': return {'states': list(self.states.values())}
        if op == 'plan_move': return {'feasible_preflight': True}
        if op == 'submit_planned': return {'submitted': True}
        if op == 'status': return {'status': 'completed'}
        if op == 'step': return {'sim_time': 0.15}
        raise AssertionError(op)

def test_test1_two_arms_two_movements():
    c = Client()
    result = execute(c, ['test1'])
    assert result.count('completed') == 4
    assert len([x for x in c.calls if x[0] == 'submit_planned']) == 4

def test_test3_six_arms_two_movements():
    c = Client()
    result = execute(c, ['test3'])
    assert result.count('completed') == 12

def test_macro_stops_on_busy():
    c = Client()
    c.states['arm_1']['available'] = False
    assert 'ocupado' in execute(c, ['test1'])
    assert not any(op == 'submit_planned' for op, _ in c.calls)

def test_macro_stops_on_preflight_rejection():
    c = Client()
    original = c.call
    def call(op, **kwargs):
        if op == 'plan_move': return {'feasible_preflight': False, 'reason': 'out_of_reach'}
        return original(op, **kwargs)
    c.call = call
    assert 'out_of_reach' in execute(c, ['test1'])
