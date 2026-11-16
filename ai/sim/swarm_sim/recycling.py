"""FASE 5.2: modelo lógico verificable de residuos; no simula herramientas ni cortes físicos."""
from __future__ import annotations
from dataclasses import dataclass

MATERIALS = frozenset({'plastico', 'madera', 'electronico'})
FUNCTIONS = frozenset({'funciona', 'no_funciona', 'sin_determinar'})


@dataclass
class Component:
    id: str
    material: str
    functional: str
    parent: str | None = None
    joint: str = 'none'
    detached: bool = False
    tested: bool = False
    classified: bool = False


class RecyclingModel:
    """Estado discreto. Ground truth privado; inspección no revela funcionalidad."""

    def __init__(self, definitions: list[dict]):
        self.components: dict[str, Component] = {}
        for raw in definitions:
            c = Component(**raw)
            if c.id in self.components:
                raise ValueError(f'Componente duplicado: {c.id}')
            if c.material not in MATERIALS or c.functional not in FUNCTIONS:
                raise ValueError(f'Etiqueta inválida: {c.id}')
            if c.joint not in ('none', 'clip', 'screw'):
                raise ValueError(f'Unión inválida: {c.id}')
            self.components[c.id] = c
        for c in self.components.values():
            if c.parent and (c.parent not in self.components or c.parent == c.id):
                raise ValueError(f'Padre inválido: {c.id}')
            seen = {c.id}
            parent = c.parent
            while parent:
                if parent in seen:
                    raise ValueError('Ciclo en ensamblaje')
                seen.add(parent)
                parent = self.components[parent].parent
        self.events: list[dict] = []

    def _get(self, component_id: str) -> Component:
        return self.components[component_id]

    def inspect(self, component_id: str) -> dict:
        c = self._get(component_id)
        return {'id': c.id, 'material_observed': c.material, 'parent': c.parent,
                'joint': c.joint, 'detached': c.detached, 'tested': c.tested,
                'classified': c.classified}

    def test_function(self, component_id: str) -> dict:
        c = self._get(component_id)
        if c.parent and not c.detached:
            raise ValueError('Primero hay que separar el componente del conjunto')
        c.tested = True
        # Modelo de sensor ideal y determinista. La fase 6 añadirá ruido y observaciones físicas.
        result = c.functional
        self.events.append({'action': 'test', 'id': c.id, 'result': result})
        return {'id': c.id, 'test_result': result, 'sensor_model': 'ideal_simulated'}

    def disassemble(self, component_id: str, tool: str) -> dict:
        c = self._get(component_id)
        if not c.parent:
            raise ValueError('El componente no tiene una unión desmontable')
        if c.detached:
            raise ValueError('El componente ya está separado')
        if c.joint == 'screw' and tool != 'screwdriver':
            raise ValueError('Se necesita screwdriver')
        if c.joint == 'clip' and tool != 'clip_tool':
            raise ValueError('Se necesita clip_tool')
        if c.joint == 'none':
            raise ValueError('Unión no desmontable')
        if self._get(c.parent).parent and not self._get(c.parent).detached:
            raise ValueError('Hay que desmontar primero el conjunto padre')
        c.detached = True
        self.events.append({'action': 'disassemble', 'id': c.id, 'tool': tool})
        return {'id': c.id, 'detached': True, 'model': 'logical_only'}

    def classify(self, component_id: str, material: str, functional: str) -> dict:
        c = self._get(component_id)
        if c.parent and not c.detached:
            raise ValueError('Componente aún unido')
        if not c.tested:
            raise ValueError('Se requiere prueba funcional previa')
        if c.classified:
            raise ValueError('Componente ya clasificado')
        if material not in MATERIALS or functional not in FUNCTIONS:
            raise ValueError('Clasificación inválida')
        correct = material == c.material and functional == c.functional
        c.classified = True
        self.events.append({'action': 'classify', 'id': c.id, 'correct': correct})
        return {'id': c.id, 'correct': correct, 'material_correct': material == c.material,
                'function_correct': functional == c.functional}

    def summary(self) -> dict:
        return {'total': len(self.components), 'detached': sum(c.detached for c in self.components.values()),
                'tested': sum(c.tested for c in self.components.values()),
                'classified': sum(c.classified for c in self.components.values()),
                'events': len(self.events)}
