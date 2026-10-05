import math
from typing import Optional, Tuple
from .datatypes import CartesianPositions, JointPositions

class ScaraKinematics:
    def __init__(self):
        # Geometria ramion i promienie
        self.r1 = 200.0
        self.r2 = 250.0
        self.l1 = 200.0
        self.l2 = 250.0
        self.r = 100.0
        
        self.c_steps_per_rev = 200.0

    """
    def calculate_arm_position(self, joints: JointPositions):
        Xa = self.l1 * math.cos(Arad) - self.r / 2
        Ya = self.l1 * math.sin(Arad)
        Xb = self.r1 * math.cos(Brad) + self.r / 2
        Yb = self.r1 * math.sin(Brad)
    """
    def forward_kinematics(self, joints: JointPositions) -> CartesianPositions:
        Arad = self._enc_to_rad(joints.A)
        Brad = self._enc_to_rad(joints.B)
        gamRad = self._step_to_rad(joints.gam)

        Xa = self.l1 * math.cos(Arad) - self.r / 2
        Ya = self.l1 * math.sin(Arad)
        Xb = self.r1 * math.cos(Brad) + self.r / 2
        Yb = self.r1 * math.sin(Brad)
        
        psi = math.atan((Yb - Ya) / (Xb - Xa))
        d = math.sqrt((Xb - Xa)**2 + (Yb - Ya)**2)
        delt = math.acos(d / (2 * self.l2))
        fi1 = psi + delt
        
        Xc = Xa + self.l2 * math.cos(fi1)
        Yc = Ya + self.l2 * math.sin(fi1)
        
        fi2 = math.radians(180) - math.atan((Yc - Yb) / (Xb - Xc))
        C = math.degrees(fi2 + gamRad)

        return CartesianPositions(X=Xc, Y=Yc, C=C, Z=joints.Z)

    def inverse_kinematics(self, pose: CartesianPositions) -> Optional[JointPositions]:
        Crad = math.radians(pose.C)
        
        niL = math.atan(pose.Y / (pose.X + (self.r / 2)))
        if (pose.X + self.r / 2) < 0:
            niL = niL + math.radians(180)
            
        niR = math.atan(pose.Y / (pose.X - (self.r / 2)))
        if (pose.X - self.r / 2) < 0:
            niR = niR + math.radians(180)
            
        eL = math.sqrt((pose.X + self.r / 2)**2 + pose.Y**2)
        eR = math.sqrt((pose.X - self.r / 2)**2 + pose.Y**2)
        
        try:
            epsL = math.acos((- (self.l2**2) + (self.l1**2) + (eL**2)) / (2 * self.l1 * eL))
            epsR = math.acos((- (self.r2**2) + (self.r1**2) + (eR**2)) / (2 * self.r1 * eR))
        except ValueError:
            return None

        Arad = niL + epsL
        Brad = niR - epsR 
        
        Xb = self.r1 * math.cos(Brad) + self.r / 2
        Yb = self.r1 * math.sin(Brad)

        fi2 = math.radians(180) - math.atan((pose.Y - Yb) / (Xb - pose.X))
        gamRad = Crad - fi2

        A = self._rad_to_enc(Arad)
        B = self._rad_to_enc(Brad)
        gam = self._rad_to_step(gamRad)
        
        return JointPositions(A=A, B=B, gam=gam, Z=pose.Z)


    #odpowienik check litmits
    def check_limits(self, joints: JointPositions) -> bool:
        A, B, Z = joints.A, joints.B, joints.Z
        if ((A - B) > 0 and (A - B) < 15000 and A > 5553 and 
            B > -1295 and A < 21775 and B < 14927 and Z >= 0 and Z < 1450):
            return True
        return False

    #odpowienik check litmitsC
    def check_limits_c(self, C: float) -> float:
        if C < 0:
            return 0.0
        elif C > 200:
            return 200.0
        return C


    @staticmethod
    def polar_to_xy(r: float, phi_deg: float, offset_x: float, offset_y: float) -> Tuple[float, float]:
        phi_rad = math.radians(phi_deg)
        x = r * math.cos(phi_rad) + offset_x
        y = r * math.sin(phi_rad) + offset_y
        return x, y

    def _rad_to_enc(self, rad: float) -> float:
        return math.degrees(rad) * 5 * 8192 / 360

    def _enc_to_rad(self, enc: float) -> float:
        return math.radians(enc * 360 / 8192 / 5)

    def _rad_to_step(self, rad: float) -> float:
        #return math.degrees(rad) * 16 * 6 * 200 / 360
        return math.degrees(rad) * self.c_steps_per_rev / 360

    def _step_to_rad(self, step: float) -> float:
        return math.radians(step * 360 / self.c_steps_per_rev)
    