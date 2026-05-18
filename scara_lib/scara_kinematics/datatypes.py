from dataclasses import dataclass


@dataclass
class CartesianPositions:
    # Wspołrzędne kartezjańskie chwytaka
    X: float
    Y: float
    C: float
    Z: float

@dataclass
class JointPositions:
    # Położenie silników
    A: float
    B: float
    gam: float
    Z: float