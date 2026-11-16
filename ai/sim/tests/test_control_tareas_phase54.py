from control_tareas_54 import execute


class Client:
    def __init__(self):
        self.calls = []

    def call(self, op, **kwargs):
        self.calls.append((op, kwargs))
        return {'ok': True}


def test_plan_does_not_execute():
    c = Client()
    execute(c, ['plan', 'arm_1', '0.1', '0.2', '0.3'])
    assert c.calls[0][0] == 'plan_move'


def test_move_uses_preflight():
    c = Client()
    execute(c, ['mover', 'arm_2', '0.1', '0.2', '0.3'])
    op, args = c.calls[0]
    assert op == 'submit_planned'
    assert args['command']['robot_id'] == 'arm_2'
    assert args['command']['type'] == 'Move'


def test_recycling_is_logical():
    c = Client()
    execute(c, ['desmontar', 'assembly_board', 'clip_tool'])
    assert c.calls == [('recycling_disassemble', {'component_id': 'assembly_board', 'tool': 'clip_tool'})]


def test_reset_requires_confirmation():
    c = Client()
    assert execute(c, ['reiniciar'], confirm=lambda _: False) == 'Reinicio cancelado'
    assert not c.calls
    execute(c, ['reiniciar'], confirm=lambda _: True)
    assert c.calls[0][0] == 'reset'


def test_invalid_step():
    c = Client()
    try:
        execute(c, ['avanzar', '-1'])
    except ValueError:
        pass
    else:
        assert False
    assert not c.calls
