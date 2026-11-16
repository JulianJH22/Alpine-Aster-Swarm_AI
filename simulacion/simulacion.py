#!/usr/bin/env python3
"""Célula visual coordinada para desmontaje, clasificación y evacuación."""
import argparse
from pathlib import Path
from enjambre.geometry import validate_essential_geometry


def main():
    parser=argparse.ArgumentParser(description='IA Enjambre: demostración visual de 6 brazos')
    parser.add_argument('--cycles','--ciclos',type=int,default=2,
                        help='Objetos centrales procesados (por defecto: 2)')
    parser.add_argument('--speed','--velocidad',type=float,default=1.65,
                        help='Multiplicador de reproducción (por defecto: 1.65)')
    parser.add_argument('--fps',type=int,default=30,help='FPS de animación (por defecto: 30)')
    parser.add_argument('--hold','--pausa-final',type=float,default=8.0)
    parser.add_argument('--headless',action='store_true',help='Modo sin ventana para pruebas')
    parser.add_argument('--no-status',action='store_true',help='Ocultar textos de las fases')
    parser.add_argument('--no-screenshot',action='store_true',help='No generar captura PNG')
    parser.add_argument('--capture-moments',action='store_true',help='Dos capturas adicionales de clasificación y giro')
    parser.add_argument('--record',action='store_true',help='Intentar grabación MP4 nativa de PyBullet (requiere GUI compatible)')
    parser.add_argument('--check',action='store_true',help='Validar geometría sin PyBullet')
    parser.add_argument('--out',default='salida/demo',help='Carpeta de resultados')
    a=parser.parse_args()
    if a.cycles<1 or a.speed<=0 or a.fps<10 or a.fps>120 or a.hold<0:
        parser.error('Usa cycles>=1, speed>0, 10<=fps<=120, hold>=0')
    check=validate_essential_geometry()
    if a.check:
        print('Geometría horizontal idealizada correcta:',check)
        print('Aviso: no equivale a validación IK 3D ni a seguridad física')
        return
    from enjambre.simulator import Simulation
    sim=Simulation(headless=a.headless,speed=a.speed,
                   fps=a.fps,show_status=not a.no_status)
    video_log=None
    try:
        sim.capture_dir=Path(a.out)
        sim.capture_moments=a.capture_moments and not a.no_screenshot
        if a.record and not a.headless:
            try:
                import pybullet as p
                Path(a.out).mkdir(parents=True,exist_ok=True)
                video_log=p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,
                                              str(Path(a.out)/'demo.mp4'))
                print('Grabación de vídeo solicitada: demo.mp4')
            except Exception as error:
                print('Grabación MP4 no disponible en esta instalación:',error)
        sim.run(a.cycles,a.hold)
        info=sim.save(a.out)
        if not a.no_screenshot:
            try:
                sim.screenshot(Path(a.out)/'captura.png')
                print('Captura guardada:',Path(a.out)/'captura.png')
            except Exception as e:
                print('No se pudo crear la captura (la demo sí ha terminado):',e)
        print('RESUMEN:',info)
        print('Resultados:',Path(a.out).resolve())
    except Exception:
        sim.save(a.out)
        raise
    finally:
        if video_log is not None and video_log>=0:
            try:
                import pybullet as p
                p.stopStateLogging(video_log)
            except Exception:pass
        sim.close()

if __name__=='__main__':main()
