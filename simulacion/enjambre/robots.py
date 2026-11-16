"""Visualización de robots Panda (mismo URDF/escalado de Fase 6).

En esta demostración las articulaciones se interpolan cinemáticamente y se
actualizan con resetJointState; NO se simula agarre por contacto ni par motor.
"""
from __future__ import annotations
import math
import pybullet as p
from .geometry import polar, ARM_BASE_RADIUS, ARM_ANGLES, ARM_ROLES, ease

ARM_COLORS = {
    'sujecion': (.12,.78,1.,1.),
    'desmontaje': (1.,.63,.17,1.),
    'clasificacion': (.28,.93,.54,1.),
}


class Robot:
    def __init__(self, robot_id: int):
        self.id=f'brazo_{robot_id}'
        self.angle=ARM_ANGLES[robot_id-1]
        self.role=ARM_ROLES[robot_id-1]
        self.color=ARM_COLORS[self.role]
        self.base=polar(ARM_BASE_RADIUS,self.angle,.052)
        inward=self.angle*math.pi/180+math.pi
        self.yaw=inward
        self.body=p.loadURDF('franka_panda/panda.urdf',self.base,
            p.getQuaternionFromEuler((0,0,inward)),useFixedBase=True,globalScaling=.55128205)
        self.joints=list(range(7)); self.fingers=(9,10); self.ee=11
        self.ll=[];self.ul=[];self.ranges=[]
        movable=[j for j in range(p.getNumJoints(self.body))
                 if p.getJointInfo(self.body,j)[2]!=p.JOINT_FIXED]
        for j in movable:
            info=p.getJointInfo(self.body,j)
            low,high=info[8],info[9]
            self.ll.append(low);self.ul.append(high);self.ranges.append(high-low)
        self.home_q=[0,-.4,0,-2.2,0,1.8,.785]
        self.q=list(self.home_q);self.q0=list(self.q);self.q1=list(self.q)
        self.grip=0.0;self.g0=0.;self.g1=0.
        self.elapsed=0.;self.duration=1.
        self.tool_orientation=p.getQuaternionFromEuler((math.pi,0,inward))
        # Este modo es un storyboard cinemático: impedir impulsos entre partes decorativas.
        for j in range(-1,p.getNumJoints(self.body)):
            p.setCollisionFilterGroupMask(self.body,j,0,0)
        self._apply()
        # Identificación por COLOR en cada base; sin etiquetas de estación.
        self._visual_ring(self.base, self.color)

    @staticmethod
    def _visual_ring(pos,rgba):
        c=p.createVisualShape(p.GEOM_CYLINDER,radius=.071,length=.018,rgbaColor=rgba)
        p.createMultiBody(baseMass=0,baseCollisionShapeIndex=-1,baseVisualShapeIndex=c,
                          basePosition=(pos[0],pos[1],.026))

    def tcp(self):
        return tuple(p.getLinkState(self.body,self.ee,computeForwardKinematics=True)[4])

    def _apply(self):
        for j,q in zip(self.joints,self.q):p.resetJointState(self.body,j,q)
        opening=.0215*(1-self.grip)+.0015*self.grip
        for j in self.fingers:p.resetJointState(self.body,j,opening)

    def aim(self, target, duration, grip=None):
        self.q0=list(self.q)
        if target is None:
            goal=list(self.q)
        elif target=='home':
            goal=list(self.home_q)
        else:
            try:
                # Misma IK analítica-numérica de PyBullet utilizada en el proyecto.
                sol=p.calculateInverseKinematics(self.body,self.ee,
                    tuple(float(v) for v in target),self.tool_orientation,
                    lowerLimits=self.ll,upperLimits=self.ul,jointRanges=self.ranges,
                    restPoses=list(self.q)+[.021,.021],maxNumIterations=75,
                    residualThreshold=5e-4)
                goal=[]
                for i in range(7):
                    x=float(sol[i]);goal.append(max(self.ll[i],min(self.ul[i],x)))
            except Exception as e:
                raise RuntimeError(f'IK falló para {self.id}: {target}: {e}') from e
        self.q1=goal
        self.g0=self.grip
        self.g1=self.grip if grip is None else max(0.,min(1.,float(grip)))
        self.elapsed=0.0;self.duration=max(.01,float(duration))

    def update(self,dt):
        self.elapsed=min(self.duration,self.elapsed+dt)
        t=ease(self.elapsed/self.duration)
        self.q=[a+(b-a)*t for a,b in zip(self.q0,self.q1)]
        self.grip=self.g0+(self.g1-self.g0)*t
        self._apply()
