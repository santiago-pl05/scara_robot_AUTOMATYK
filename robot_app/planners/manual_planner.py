from scara_kinematics.datatypes import CartesianPositions
from input_device import GamePadInput

class ManualTeleopPlanner:
    def __init__(self, pad: GamePadInput, freq: int = 80):
        self.pad = pad
        self.freq = freq
        
        # Przeniesione z app.py
        self.xySpeedPadMultiplyer = 1.6 * 8 * (50 / self.freq) * 4
        self.xyAccelPad = 0.50 * (50 / self.freq)   # Szybkie przyśpieszanie 
        self.xyDecelPad = 0.05 * (50 / self.freq)   # Wolne hamowanie
        
        self.CSpeedPadMultiplyer = 2 * (150 / self.freq)
        self.ZSpeedPadMultiplyer = 7 * (150 / self.freq)
        
        self.prevCommandSpeedX = 0
        self.prevCommandSpeedY = 0

    def get_position(self, current_pos: CartesianPositions, dt: float) -> CartesianPositions:
        pad_state = self.pad.read_values()
        
        padX = pad_state.delta_x
        padY = pad_state.delta_y
        padC = pad_state.delta_c
        padZ = pad_state.delta_z
        
        # --- sterowanie X ---
        if abs(padX) < abs(self.prevCommandSpeedX) or padX == 0:
            krokX = self.xyDecelPad
        else:
            krokX = self.xyAccelPad

        if ((padX - self.prevCommandSpeedX) >= -krokX and (padX - self.prevCommandSpeedX) <= krokX):
            comandedSpeedX = padX
        elif ((self.prevCommandSpeedX - padX) < -krokX):            
            comandedSpeedX = self.prevCommandSpeedX + krokX
        elif ((self.prevCommandSpeedX - padX) > krokX):            
            comandedSpeedX = self.prevCommandSpeedX - krokX
            
        # --- sterowanie Y ---
        if abs(padY) < abs(self.prevCommandSpeedY) or padY == 0:
            krokY = self.xyDecelPad
        else:
            krokY = self.xyAccelPad

        if ((padY - self.prevCommandSpeedY) >= -krokY and (padY - self.prevCommandSpeedY) <= krokY):
            comandedSpeedY = padY
        elif ((self.prevCommandSpeedY - padY) < -krokY):            
            comandedSpeedY = self.prevCommandSpeedY + krokY
        elif ((self.prevCommandSpeedY - padY) > krokY):            
            comandedSpeedY = self.prevCommandSpeedY - krokY

        # --- Wyliczenie nowej pozycji ---
        # Używamy dt z zewnątrz dla pełnej poprawności czasowej (opcjonalnie, jeśli freq byłoby w pełni zastąpione przez dt)
        X = current_pos.X + comandedSpeedX * self.xySpeedPadMultiplyer
        Y = current_pos.Y + comandedSpeedY * self.xySpeedPadMultiplyer
        
        C = current_pos.C + (padC * self.CSpeedPadMultiplyer)
        Z = current_pos.Z + (padZ * self.ZSpeedPadMultiplyer)
        
        self.prevCommandSpeedX = comandedSpeedX
        self.prevCommandSpeedY = comandedSpeedY
        
        return CartesianPositions(X, Y, C, Z)
