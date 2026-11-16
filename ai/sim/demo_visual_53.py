"""Demostracion visual de FASE 5.3: seis brazos, movimientos TCP cortos y supervisados.

No entrena MAPPO y NO es una demostracion de MPC optimo ni de desmontaje fisico.
Ejecutar: python -u demo_visual_53.py --gui
"""
import argparse
import json
import math
from pathlib import Path

from swarm_sim import ManeuverCommand, ManeuverType, Pose, SimWorld
from swarm_sim.gateway import Gateway


def run_demo(world, amplitude=0.02, timeout=8.0):
    gateway = Gateway(world)
    report = []
    # Secuencial por seguridad: evita asumir que el preflight TCP certifica
    # colisiones de eslabones o trayectorias simultaneas.
    for robot_id in sorted(world.arms, key=lambda x: int(x.split('_')[-1])):
        arm = world.arms[robot_id]
        start = tuple(float(x) for x in arm.tcp()[0])
        # Pequeño desplazamiento vertical y regreso; evita cruzar sectores.
        for phase, goal in (("elevar", (start[0], start[1], start[2] + amplitude)),
                            ("regresar", start)):
            plan = gateway.op_plan_move(robot_id, list(goal), max_speed=0.04)
            if not plan['feasible_preflight']:
                report.append({'robot': robot_id, 'phase': phase, 'status': 'rejected',
                               'reason': plan['reason']})
                print(f"{robot_id} {phase}: RECHAZADO ({plan['reason']})", flush=True)
                if phase == 'elevar':
                    break
                continue
            execution_id = f"demo53_{robot_id}_{phase}"
            cmd = ManeuverCommand(task_id=execution_id, execution_id=execution_id,
                                  robot_id=robot_id, type=ManeuverType.MOVE,
                                  target=Pose(tuple(goal)), max_speed=0.04,
                                  tolerance=0.012, timeout_s=timeout)
            result = arm.submit(cmd)
            if result.status.value == 'accepted':
                world.run_until_idle(timeout)
                result = arm.result(execution_id)
            error = result.error_m
            report.append({'robot': robot_id, 'phase': phase,
                           'status': result.status.value, 'detail': result.detail,
                           'error_m': error})
            print(f"{robot_id} {phase}: {result.status.value} "
                  f"error={error if error is not None else 'N/A'} m "
                  f"{result.detail}", flush=True)
            if result.status.value != 'completed':
                break
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true', help='Mostrar ventana 3D de PyBullet')
    parser.add_argument('--config', default=str(Path(__file__).parent / 'config' / 'scenario.json'))
    parser.add_argument('--amplitude', type=float, default=0.02, help='Desplazamiento vertical en metros (0.005-0.03)')
    parser.add_argument('--output', default='out/demo_visual_53.json')
    args = parser.parse_args()
    if not math.isfinite(args.amplitude) or not 0.005 <= args.amplitude <= 0.03:
        parser.error('--amplitude debe estar entre 0.005 y 0.03 m')
    world = SimWorld(args.config, gui=args.gui, realtime=args.gui)
    try:
        print('FASE 5.3: demostracion secuencial de seis brazos. Sin entrenamiento.', flush=True)
        report = run_demo(world, args.amplitude)
        dest = Path(args.output)
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(json.dumps({'sim_time': world.time, 'movements': report}, indent=2), encoding='utf-8')
        print(f'Resultados guardados en {dest}', flush=True)
        if args.gui:
            input('Pulsa Enter para cerrar la ventana 3D...')
    finally:
        world.close()


if __name__ == '__main__':
    main()
