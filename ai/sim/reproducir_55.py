"""Reproduce estados articulares registrados en una ventana PyBullet, sin ejecutar IA."""
import argparse
import sys
import time
from pathlib import Path

from swarm_sim.episode_55 import load_episode
from swarm_sim.world import SimWorld


def replay(data, config, fps=20):
    samples = data['samples']
    if not samples:
        raise ValueError('No hay muestras para reproducir')
    world = SimWorld(config, gui=True, realtime=False)
    try:
        for i, sample in enumerate(samples):
            for robot_id, arm_data in sample.get('arms', {}).items():
                arm = world.arms.get(robot_id)
                if arm is None:
                    continue
                for joint, angle in zip(arm.arm_joints, arm_data.get('q', [])):
                    world.bc.resetJointState(arm.body, joint, float(angle))
            for obj_id, pos in sample.get('objects', {}).items():
                obj = world.objects.get(obj_id)
                if obj is not None:
                    _, orientation = world.bc.getBasePositionAndOrientation(obj.body)
                    world.bc.resetBasePositionAndOrientation(obj.body, pos, orientation)
            if 'platform_angle_rad' in sample:
                world._set_platform_angle(float(sample['platform_angle_rad']))
            if i + 1 < len(samples):
                delay = max(0, min(0.3, (samples[i + 1]['sim_time'] - sample['sim_time'])))
                time.sleep(delay if delay else 1 / fps)
        input('Reproducción terminada. Pulsa Enter para cerrar...')
    finally:
        world.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('episode', nargs='?', default='out/phase55/ultimo/episode.json')
    parser.add_argument('--config', default=str(Path(__file__).parent / 'config/scenario.json'))
    args = parser.parse_args()
    replay(load_episode(args.episode), args.config)


if __name__ == '__main__':
    main()
