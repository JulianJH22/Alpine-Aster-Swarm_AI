"""Coordinación visual: seis robots / un objeto / dos materiales / dos cintas.

No entrenamos ni cargamos IA aquí. La coreografía pasa por Controller/RobotCommand,
por lo que el siguiente paso puede sustituir ScriptedController por una política.
"""
from __future__ import annotations
import json
import math
import time
from pathlib import Path
import pybullet as p
import pybullet_data
from .geometry import polar, ease, validate_essential_geometry
from .control import RobotCommand, CellObservation, ScriptedController
from .robots import Robot
from .scene import Scene


def command(number, pos=None, grip=None, seconds=1.0):
    return RobotCommand(f'brazo_{number}',pos,grip,seconds)


class Simulation:
    def __init__(self, headless=False, speed=1.5, fps=30, show_status=True):
        self.headless=headless
        self.speed=speed
        self.fps=fps
        self.frame_counter=0
        self.paused=False
        self.capture_dir=None
        self.capture_moments=False
        self.events=[]
        self.cycles_done=0
        self.connection=p.connect(p.DIRECT if headless else p.GUI)
        if self.connection<0:raise RuntimeError('No se ha podido conectar con PyBullet')
        try:
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,-9.81)
            p.setTimeStep(1.0/240.0)
            p.setRealTimeSimulation(0)
            if not headless:
                try:
                    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
                    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,1)
                    p.resetDebugVisualizerCamera(2.48,43,-58,(0,0,.16))
                except Exception:
                    pass
            self.scene=Scene(show_status=show_status)
            self.robots={f'brazo_{i}':Robot(i) for i in range(1,7)}
            self.controller=ScriptedController()
            self.record('inicio',geometry=validate_essential_geometry())
        except Exception:
            p.disconnect()
            raise

    def record(self,event,**kwargs):
        v={'t':round(self.scene.time,2),'evento':event,**kwargs}
        self.events.append(v)
        print(f'[{v["t"]:6.1f}s] {event}',flush=True)

    def observation(self,phase):
        return CellObservation(
            sim_time_s=self.scene.time,phase=phase,
            arms_tcp={name:robot.tcp() for name,robot in self.robots.items()},
            pieces={k:tuple(p.getBasePositionAndOrientation(obj.body)[0])
                    for k,obj in self.scene.pieces.items()},
            turntable_angle_deg=self.scene.rotation,
            active_cargo=tuple(k for k,v in self.scene.pieces.items() if v.state=='tray'),
            counters=dict(self.scene.counters))

    def _keyboard(self):
        if self.headless:return
        try:
            events=p.getKeyboardEvents()
        except Exception:
            return
        for key,value in events.items():
            if not (value & p.KEY_WAS_TRIGGERED):continue
            if key in (ord('p'),ord('P')):
                self.paused=not self.paused
                print('PAUSA' if self.paused else 'REANUDADO',flush=True)
            elif key in (ord('+'),ord('=')):
                self.speed=min(8.,self.speed*1.25)
                print(f'Velocidad x{self.speed:.2f}',flush=True)
            elif key in (ord('-'),ord('_')):
                self.speed=max(.25,self.speed/1.25)
                print(f'Velocidad x{self.speed:.2f}',flush=True)
            elif key in (ord('0'),):
                p.resetDebugVisualizerCamera(2.48,43,-58,(0,0,.16))

    def frames(self,seconds,callback=None):
        count=max(1,math.ceil(seconds*self.fps))
        dt=seconds/count
        deadline=time.perf_counter()
        for i in range(count):
            self._keyboard()
            while self.paused and not self.headless:
                time.sleep(.04)
                self._keyboard()
                deadline=time.perf_counter()
            if callback:
                callback((i+1)/count)
            # Física avanzando por debajo; todos los actores son cinemáticos.
            p.stepSimulation()
            for robot in self.robots.values():
                robot.update(dt)
            self.scene.update(dt,self.robots)
            self.frame_counter+=1
            if not self.headless:
                deadline+=dt/self.speed
                remaining=deadline-time.perf_counter()
                if remaining>0: time.sleep(remaining)
                elif remaining < -0.2: deadline=time.perf_counter()

    def phase(self,name,seconds,commands=(),after=None,progress=None):
        self.scene.status(name)
        self.record('fase',nombre=name)
        self.controller.set_commands(commands)
        for item in self.controller.decide(self.observation(name)):
            if item.robot_id not in self.robots:
                raise ValueError(f'Comando para robot desconocido: {item.robot_id}')
            self.robots[item.robot_id].aim(item.target_xyz,item.duration_s,
                                           grip=item.gripper)
        self.frames(seconds,progress)
        if after:after()

    def rotate_to(self,angle,seconds=3.6):
        old=self.scene.rotation
        self.scene.status('05  /  PLATAFORMA GIRATORIA')
        self.record('rotacion_inicio',desde_deg=old,hasta_deg=angle)
        self.frames(seconds,lambda x:setattr(self.scene,'rotation',
                                             old+(angle-old)*ease(x)))
        self.scene.rotation=angle
        self.record('rotacion_fin',angulo_deg=self.scene.rotation)

    def _reset_arms(self,secs=2.0):
        self.phase('REPOSICIONAMIENTO',secs,[command(i,'home',0,secs)
                                              for i in range(1,7)])

    def cycle(self,index):
        self.record('nuevo_objeto',numero=index+1)
        self.scene.create_workpiece(index)
        self.phase('01  /  SUJECION DEL OBJETO',2.8,[
            command(1,polar(.157,0,.284),1,2.8),
            command(2,polar(.157,180,.284),1,2.8),
            command(3,polar(.145,60,.384),0,2.8),
            command(4,polar(.145,240,.384),0,2.8),
            command(5,polar(.478,90,.315),0,2.8),
            command(6,polar(.478,270,.315),0,2.8)])
        self.record('pieza_sujetada_por_dos_brazos',modo='cinematico')
        self.phase('SUJECION  /  ELEVAR OBJETO CENTRAL',1.35,[
            command(1,polar(.157,0,.329),1,1.35),
            command(2,polar(.157,180,.329),1,1.35)],
            progress=lambda t:self.scene.set_assembly_lift(.045*ease(t)))
        self.record('objeto_central_elevado',altura_m=.045)
        self.phase('02  /  DESMONTAJE DE DOS COMPONENTES',2.1,[
            command(3,polar(.145,60,.374),0,2.1),
            command(4,polar(.145,240,.374),0,2.1)])
        self.phase('DESMONTAJE  /  LIBERAR FIJACIONES',1.1,[
            command(3,polar(.145,60,.354),1,1.1),
            command(4,polar(.145,240,.354),1,1.1)])
        for kind,arm in (('metal','brazo_3'),('plastico','brazo_4')):
            self.scene.set_piece(kind,'attached',arm)
            self.record('componente_extraido',material=kind,robot=arm)
        self.phase('EXTRACCION  /  RETIRAR MODULOS',1.9,[
            command(3,polar(.200,60,.398),1,1.9),
            command(4,polar(.200,240,.398),1,1.9)])
        self.phase('TRANSFERENCIA  /  ZONAS DE ENTREGA',2.6,[
            command(3,polar(.478,90,.231),1,2.6),
            command(4,polar(.478,270,.231),1,2.6),
            command(5,polar(.478,90,.315),0,2.6),
            command(6,polar(.478,270,.315),0,2.6)])
        self.scene.place_handoff('metal');self.scene.place_handoff('plastico')
        self.record('componentes_en_puntos_de_transferencia')
        self.phase('03  /  RECEPCION POR BRAZOS CLASIFICADORES',1.45,[
            command(3,polar(.39,60,.340),0,1.45),
            command(4,polar(.39,240,.340),0,1.45),
            command(5,polar(.478,90,.230),1,1.45),
            command(6,polar(.478,270,.230),1,1.45)])
        self.scene.set_piece('metal','attached','brazo_5')
        self.scene.set_piece('plastico','attached','brazo_6')
        self.record('componentes_recogidos_por_clasificadores')
        self.phase('CLASIFICACION  /  ELEVACION',1.3,[
            command(5,polar(.435,100,.312),1,1.3),
            command(6,polar(.435,280,.312),1,1.3)])
        self.phase('04  /  DEPOSITO EN SECTORES',2.2,[
            command(5,polar(.327,120,.185),1,2.2),
            command(6,polar(.327,300,.185),1,2.2)])
        self.scene.load_tray('metal');self.scene.load_tray('plastico')
        self.record('componentes_clasificados_en_plataforma',
                    metales='sector_120',plasticos='sector_300')
        if self.capture_moments and index==0 and self.capture_dir:
            self._capture_moment('01_clasificacion.png')
        self.phase('SUJECION  /  DEVOLVER CARCASA',1.2,[
            command(1,polar(.157,0,.284),1,1.2),
            command(2,polar(.157,180,.284),1,1.2)],
            progress=lambda t:self.scene.set_assembly_lift(.045*(1-ease(t))))
        self.record('carcasa_devuelta_al_soporte')
        self.phase('BRAZOS FUERA DE LA ZONA DE GIRO',2.15,[
            command(i,'home',0,2.15) for i in range(1,7)])
        # Dos bandejas cargadas se alinean simultáneamente con sus respectivas cintas.
        start=360.0*index
        self.rotate_to(start+150,seconds=4.0)
        if self.capture_moments and index==0 and self.capture_dir:
            self._capture_moment('02_giro_y_salida.png')
        self.scene.start_discharge('metal')
        self.scene.start_discharge('plastico')
        self.record('descarga_a_cintas',metales='cinta_negativa_y',
                    plasticos='cinta_positiva_y')
        self.scene.status('06  /  EVACUACION EN CINTAS')
        self.frames(4.25)
        self.record('salida_completada',**dict(self.scene.counters))
        self.rotate_to(start+360,seconds=3.1)
        self.cycles_done+=1

    def run(self,cycles=2,hold=6.0):
        try:
            self.scene.status('CELULA DE DESMONTAJE  /  6 BRAZOS')
            self.frames(1.0)
            for index in range(cycles):self.cycle(index)
            self.scene.status('PROCESO COMPLETADO')
            self.record('demostracion_completada',ciclos=cycles)
            if hold>0:self.frames(hold)
        finally:
            pass

    def save(self,out):
        dest=Path(out);dest.mkdir(parents=True,exist_ok=True)
        (dest/'eventos.json').write_text(json.dumps(self.events,ensure_ascii=False,
                                      indent=2),encoding='utf-8')
        summary={
            'escena':'IA_Enjambre_Simulacion',
            'modo':'demostracion_cinematica_guiada',
            'ciclos_completos':self.cycles_done,
            'piezas_recogidas_por_cintas':dict(self.scene.counters),
            'frames':self.frame_counter,
            'tiempo_simulado_s':round(self.scene.time,2),
            'ia_entrenada_en_uso':False,
            'agarre_fisico_real':False,
            'colisiones_fisicas_validadas':False,
            'geometria':'escena compacta de presentacion; verificar volumen barrido e IK',
        }
        (dest/'resumen.json').write_text(json.dumps(summary,ensure_ascii=False,
                                   indent=2),encoding='utf-8')
        return summary

    def _capture_moment(self,filename):
        try:
            self.screenshot(Path(self.capture_dir)/filename)
        except Exception as error:
            print('Captura de momento omitida:',error,flush=True)

    def screenshot(self,path):
        import struct,zlib
        import numpy as np
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        w,h=1280,720
        view=p.computeViewMatrixFromYawPitchRoll((0,0,.18),2.40,43,-59,0,2)
        proj=p.computeProjectionMatrixFOV(55,w/h,.02,15.)
        im=p.getCameraImage(w,h,viewMatrix=view,projectionMatrix=proj,
                            renderer=p.ER_TINY_RENDERER)
        rgba=np.asarray(im[2],dtype=np.uint8).reshape(h,w,4)
        rgb=rgba[:,:,:3]
        payload=b''.join(b'\x00'+rgb[i].tobytes() for i in range(h))
        def chunk(t,data):
            return (struct.pack('!I',len(data))+t+data+
                    struct.pack('!I',zlib.crc32(t+data)&0xffffffff))
        path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',w,h,8,2,0,0,0))+
                         chunk(b'IDAT',zlib.compress(payload,6))+chunk(b'IEND',b''))

    def close(self):
        if p.isConnected(self.connection):p.disconnect(self.connection)
