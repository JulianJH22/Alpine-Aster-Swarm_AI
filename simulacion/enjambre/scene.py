"""Escenografía industrial y transporte animado de materiales.

Las piezas son sólidos visuales en PyBullet; para esta demo el agarre, la
transferencia y las cintas están prescritos cinemáticamente. No hay contactos
físicos realistas entre pieza/pinza, ni clasificación por visión.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
import pybullet as p
from .geometry import DEG, polar, mix, TURNTABLE_TRAY_RADIUS, HANDOFF_RADIUS

SILVER=(.59,.72,.82,1)
COPPER=(1.,.58,.22,1)
GREEN=(.23,.87,.55,1)
CYAN=(.13,.76,.95,1)
PLASTIC=(.91,.32,.62,1)
DARK=(.13,.19,.26,1)
BLACK=(.062,.09,.135,1)
YELLOW=(1.,.76,.25,1)


def box(position, dims, rgba, orientation=None):
    v=p.createVisualShape(p.GEOM_BOX,halfExtents=tuple(x/2 for x in dims),rgbaColor=rgba)
    return p.createMultiBody(0,-1,v,basePosition=position,
        baseOrientation=orientation or (0,0,0,1))


def cylinder(position,radius,height,rgba,orn=None):
    v=p.createVisualShape(p.GEOM_CYLINDER,radius=radius,length=height,rgbaColor=rgba)
    return p.createMultiBody(0,-1,v,basePosition=position,
        baseOrientation=orn or (0,0,0,1))


def reset(body, pos, orn=(0,0,0,1)):
    p.resetBasePositionAndOrientation(body,tuple(pos),orn)


@dataclass
class Piece:
    kind: str
    body: int
    home: tuple
    carrier: str | None=None
    state: str='mounted'
    tray_local_angle: float=0.0
    progress: float=0.0


class Scene:
    def __init__(self, show_status=True):
        self.show_status=show_status
        self.debug_label=None
        self.rotation=0.0
        self.tray_radials=[i*60 for i in range(6)]
        self.trays=[]
        self.rotor=None
        self.slat_bodies={}
        self.pieces={}
        self.housing=[]
        self.housing_baselines={}
        self.assembly_lift=0.0
        self.time=0.0
        self.running_belts=True
        self.counters={'metales':0,'plasticos':0}
        self._build()

    def _build(self):
        # Entorno neutro y contraste pensado para capturas y presentaciones.
        box((0,0,-.045),(3.55,3.55,.075),BLACK)
        box((0,0,-.006),(1.34,1.34,.018),(.21,.26,.32,1))
        box((0,0,.028),(1.12,1.12,.014),(.08,.12,.17,1))
        # Mesa central fija + anillo rotatorio a un nivel ligeramente inferior.
        cylinder((0,0,.055),.205,.105,(.27,.33,.40,1))
        cylinder((0,0,.114),.195,.015,(.48,.58,.66,1))
        cylinder((0,0,.080),.444,.040,(.18,.23,.31,1))
        cylinder((0,0,.104),.431,.016,(.085,.12,.19,1))
        self.rotor=cylinder((0,0,.116),.425,.015,(.29,.35,.44,1))
        # Plataforma con seis bandejas de sectores contrastados.
        colors=[(.12,.39,.49,1),(.22,.29,.49,1),(.17,.49,.37,1),
                (.42,.32,.20,1),(.29,.32,.51,1),(.44,.26,.35,1)]
        for i,a in enumerate(self.tray_radials):
            orn=p.getQuaternionFromEuler((0,0,a*DEG))
            shell=box(polar(TURNTABLE_TRAY_RADIUS,a,.133),(.171,.285,.018),
                      (.08,.12,.19,1),orn)
            pad=box(polar(TURNTABLE_TRAY_RADIUS,a,.145),(.159,.271,.009),
                    colors[i],orn)
            self.trays.append((shell,pad))
        # Centro: el soporte es estático incluso durante la rotación.
        cylinder((0,0,.158),.133,.07,(.12,.16,.22,1))
        cylinder((0,0,.195),.125,.012,(.46,.53,.61,1))
        # Iluminación visual: cuatro detalles metálicos del bastidor.
        for a in (45,135,225,315):
            pos=polar(.155,a,.19)
            cylinder(pos,.012,.012,YELLOW)
        # Bancadas de transferencia entre desmontadores y clasificadores.
        for a in (90,270):
            box(polar(HANDOFF_RADIUS,a,.11),(.133,.116,.08),(.16,.24,.29,1))
            box(polar(HANDOFF_RADIUS,a,.16),(.117,.103,.016),(.50,.61,.68,1))
            for delta in (-.042,.042):
                # Bandas de aviso de la bandeja de transferencia.
                v=polar(HANDOFF_RADIUS+delta,a,.173)
                box(v,(.008,.084,.004),YELLOW)
        self._build_conveyor(+1, PLASTIC)
        self._build_conveyor(-1, COPPER)
        # Banda perimetral de seguridad, sin encapsular brazos con paredes.
        for xx in (-.94,.94):
            box((xx,0,.015),(.012,1.83,.02),(.27,.37,.48,1))
        for yy in (-1.49,1.49):
            box((0,yy,.015),(1.9,.012,.02),(.27,.37,.48,1))

    def _build_conveyor(self,sign,color):
        y=sign*.988
        box((0,y,.073),(.26,.82,.115),(.15,.20,.27,1))
        box((0,y,.137),(.235,.83,.017),(.05,.09,.13,1))
        for x in (-.131,.131):
            box((x,y,.163),(.019,.855,.056),(.40,.46,.55,1))
        # Rodillos de entrada/salida.
        axis=p.getQuaternionFromEuler((0,math.pi/2,0))
        for yy in (sign*.585,sign*1.386):
            cylinder((0,yy,.155),.028,.245,(.58,.67,.75,1),axis)
        # Identificación visual por color; sin nombres sobre la escena.
        box((0,sign*1.389,.182),(.22,.036,.012),color)
        for i in range(13):
            self.slat_bodies.setdefault(sign,[]).append(
                box((0,y,.148),(.186,.012,.006),(.34,.43,.55,1)))
        # Contenedor al final de la cinta.
        end=sign*1.49
        box((0,end,.054),(.30,.165,.09),(.20,.25,.32,1))
        for xx in (-.146,.146):
            box((xx,end,.115),(.015,.165,.11),color)

    def create_workpiece(self,batch):
        # Carcasa central y dos módulos desmontables con codificación cromática.
        if not self.housing:
            self.housing.append(box((0,0,.230),(.244,.244,.071),(.28,.36,.45,1)))
            self.housing.append(box((0,0,.272),(.223,.224,.015),(.76,.82,.86,1)))
            self.housing.append(box((0,0,.285),(.105,.106,.010),(.17,.24,.32,1)))
            for x in (-.087,.087):
                for y in (-.087,.087):
                    self.housing.append(cylinder((x,y,.285),.010,.009,(.10,.15,.22,1)))
            for a in (60,240):
                for n in (-1,1):
                    screw=polar(.145+n*.016,a,.314)
                    self.housing.append(cylinder(screw,.005,.006,(.1,.12,.15,1)))
            self.housing_baselines={body:tuple(p.getBasePositionAndOrientation(body)[0])
                                    for body in self.housing}
        self.pieces.clear()
        for kind,a,col,dims in (('metal',60,COPPER,(.065,.049,.040)),
                                 ('plastico',240,PLASTIC,(.066,.050,.040))):
            pos=polar(.145,a,.289)
            orn=p.getQuaternionFromEuler((0,0,a*DEG))
            b=box(pos,dims,col,orn)
            self.pieces[kind]=Piece(kind,b,pos)
        self.set_assembly_lift(0.0)

    def set_assembly_lift(self,height):
        self.assembly_lift=float(height)
        for body,pos in self.housing_baselines.items():
            reset(body,(pos[0],pos[1],pos[2]+self.assembly_lift))
        for part in self.pieces.values():
            if part.state=='mounted':
                home=part.home
                reset(part.body,(home[0],home[1],home[2]+self.assembly_lift))

    def set_piece(self,kind,state,carrier=None):
        piece=self.pieces[kind]
        piece.state=state
        piece.carrier=carrier
        piece.progress=0.0

    def place_handoff(self,kind):
        part=self.pieces[kind]
        self.set_piece(kind,'handoff')
        a=90 if kind=='metal' else 270
        reset(part.body,polar(HANDOFF_RADIUS,a,.193))

    def load_tray(self,kind):
        part=self.pieces[kind]
        self.set_piece(kind,'tray')
        part.tray_local_angle=120 if kind=='metal' else 300
        self._update_tray_piece(part)

    def _update_tray_piece(self,part):
        reset(part.body,polar(TURNTABLE_TRAY_RADIUS,
                              part.tray_local_angle+self.rotation,.165))

    def start_discharge(self,kind):
        self.set_piece(kind,'discharge')

    def update(self,dt,robots):
        self.time+=dt
        self._update_platform()
        for piece in self.pieces.values():
            if piece.state=='attached':
                if piece.carrier in robots:
                    tip=robots[piece.carrier].tcp()
                    reset(piece.body,(tip[0],tip[1],tip[2]-.020))
            elif piece.state=='tray':
                self._update_tray_piece(piece)
            elif piece.state=='discharge':
                piece.progress+=dt
                sign=-1 if piece.kind=='metal' else 1
                # Deslizamiento controlado hacia la cinta (no fricción simulada).
                if piece.progress<1.15:
                    t=min(1,piece.progress/1.15)
                    rr=.327+(.635-.327)*(t*t*(3-2*t))
                    z=.166+(.019)*(t*t*(3-2*t))
                else:
                    t=min(1,(piece.progress-1.15)/2.55)
                    rr=.635+.79*t
                    z=.185
                reset(piece.body,(0,sign*rr,z))
                if piece.progress>=3.7:
                    self.set_piece(piece.kind,'collected')
                    reset(piece.body,(0,0,-2))
                    key='metales' if piece.kind=='metal' else 'plasticos'
                    self.counters[key]+=1
        # Correas en movimiento. La animación es continua e independiente de los ciclos.
        for sign,slats in self.slat_bodies.items():
            for i,body in enumerate(slats):
                t=((i/len(slats)+self.time*.20)%1.)
                rr=.60+t*.785
                reset(body,(0,sign*rr,.149))

    def _update_platform(self):
        ang=self.rotation*DEG
        reset(self.rotor,(0,0,.116),p.getQuaternionFromEuler((0,0,ang)))
        for i,(shell,pad) in enumerate(self.trays):
            a=self.tray_radials[i]+self.rotation
            orn=p.getQuaternionFromEuler((0,0,a*DEG))
            reset(shell,polar(TURNTABLE_TRAY_RADIUS,a,.133),orn)
            reset(pad,polar(TURNTABLE_TRAY_RADIUS,a,.145),orn)

    def status(self,text):
        if self.debug_label is not None:
            try:p.removeUserDebugItem(self.debug_label)
            except Exception:pass
            self.debug_label=None
        if self.show_status:
            self.debug_label=p.addUserDebugText(text,(-.82,-.79,.065),
                   textColorRGB=(.96,.98,1),textSize=1.25,lifeTime=0)
