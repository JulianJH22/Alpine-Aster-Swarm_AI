"""Pruebas de lógica sin dependencia de PyBullet."""
import json
import importlib.util
from pathlib import Path
import pytest

SIM = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('recycling', SIM / 'swarm_sim/recycling.py')
module = importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name] = module
spec.loader.exec_module(module)
RecyclingModel = module.RecyclingModel


@pytest.fixture
def model():
    definitions = json.loads((SIM / 'config/scenario.json').read_text())['recycling_components']
    return RecyclingModel(definitions)


def test_inspection_does_not_leak_truth(model):
    data = model.inspect('obj_3')
    assert 'functional' not in data and 'funciona' not in str(data)
    assert data['material_observed'] == 'electronico'


def test_simple_test_and_classification(model):
    with pytest.raises(ValueError):
        model.classify('obj_1', 'plastico', 'funciona')
    assert model.test_function('obj_1')['test_result'] == 'funciona'
    assert model.classify('obj_1', 'plastico', 'funciona')['correct']
    with pytest.raises(ValueError):
        model.classify('obj_1', 'plastico', 'funciona')


def test_progressive_disassembly(model):
    with pytest.raises(ValueError):
        model.test_function('assembly_board')
    with pytest.raises(ValueError):
        model.disassemble('assembly_board', 'screwdriver')
    assert model.disassemble('assembly_board', 'clip_tool')['detached']
    assert model.test_function('assembly_board')['test_result'] == 'no_funciona'
    assert not model.classify('assembly_board', 'madera', 'no_funciona')['correct']


def test_reset_new_model():
    data = [{'id':'x','material':'madera','functional':'funciona'}]
    m = RecyclingModel(data)
    m.test_function('x')
    assert m.summary()['tested'] == 1
    assert RecyclingModel(data).summary()['tested'] == 0


def test_invalid_assemblies():
    with pytest.raises(ValueError):
        RecyclingModel([{'id':'a','material':'plastico','functional':'funciona','parent':'b'},
                        {'id':'b','material':'madera','functional':'funciona','parent':'a'}])
