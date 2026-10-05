from math import sin, cos
from scara_kinematics.datatypes import CartesianPositions

class AutonomousTrajectoryPlanner:
    def __init__(self):
        self.is_active = False
        # temp params
        self.angle = 0

    def start(self):
        self.is_active = True

    def stop(self):
        self.is_active = False

    def get_position(self, current_pos: CartesianPositions, dt: float) -> CartesianPositions:
        if not self.is_active:
            return current_pos
        
        self.angle += 2 * dt
        new_pos = CartesianPositions(
            X=30 * cos(self.angle),
            Y=30 * sin(self.angle),
            C=current_pos.C,
            Z=current_pos.Z
        )
        return new_pos

        
        