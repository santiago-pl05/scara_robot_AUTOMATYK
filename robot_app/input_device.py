import pygame as pg
from time import time
from gamepad_state import GamepadState
    
class JoystickButton:
    def __init__(self):
        self.last_press_time = 0.0
        
        self.state = False
        self.prevState = False # do funki get_switch_state
        self.raw_prev_state = False # wykrywanie zbocza narastającego
        
    def get_switch_state(self):
        if self.state != self.prevState:
            self.prevState = self.state
            return True
        return False
    
    def get_state(self):
        return self.state
    
    def loop(self, pad_button_state):
        if pad_button_state and not self.raw_prev_state: # tylko zbocze narastające
            if (time() - self.last_press_time > 0.1):  # Debounce
                self.state = not self.state
                self.last_press_time = time()
                
        self.raw_prev_state = pad_button_state
        
class GamePadInput:
    
    def __init__(self):
        pg.display.init()
        pg.joystick.init()
        self.joystick = None
        
        self.last_grip_time = 0.0
        
        self.button_mode = JoystickButton()
        self.button_grip = JoystickButton()
        
        self.state_1 = False
        self.last_debounce_1 = 0.0
        
        if pg.joystick.get_count() > 0:
            
            for i in range(pg.joystick.get_count()):
                self.joystick = pg.joystick.Joystick(i)
                self.joystick.init()
                if self.joystick.get_init():
                    print(f"Podłączono pad {self.joystick.get_name() } na porcie {self.joystick.get_id()}")
                    
        else:
            print("Nie wykryto żadnego joysticka")
        
        
    def read_values(self) -> GamepadState:
        
        if self.joystick == None:
            return self._empty_state()
        
        pg.event.pump()   
        
        pad_x = self.joystick.get_axis(3)
        pad_y = self.joystick.get_axis(4)
        pad_z = self.joystick.get_hat(0)[1]
        pad_C = self.joystick.get_hat(0)[0]
        
        raw_grip = self.joystick.get_button(0)
        raw_mode = self.joystick.get_button(1)
        
        self.button_mode.loop(raw_mode)
        self.button_grip.loop(raw_grip)
        

        return GamepadState(
            delta_x=pad_x,
            delta_y=pad_y,
            delta_z=pad_z,
            delta_c=pad_C,
            gripper_toogle=self.button_grip.get_switch_state(),
            mode_switch=self.button_mode.get_switch_state()
        )
        
    def _empty_state(self) -> GamepadState:
        return GamepadState(
            delta_x=0,
            delta_y=0,
            delta_z=0,
            delta_c=0,
            gripper_toogle=False,
            mode_switch=False
        )