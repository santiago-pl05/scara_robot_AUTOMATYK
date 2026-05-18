from dataclasses import dataclass

@dataclass
class GamepadState:
    # Sterowanie z pada
    delta_x: float
    delta_y: float
    delta_z: float
    delta_c: float
    gripper_toogle: bool
    mode_switch: bool