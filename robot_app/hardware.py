import time
import sys
import odrive
from odrive.enums import AxisState

import serial


class OdriveController:
    def __init__(self, is_enable: bool = True):
        self.is_enable = is_enable
        self.odrv = None
        
        self.offset_AL = -6932 #dojechac do zderzenia A z lewą stroną ramy i wpisac wartosc tutaj
        self.offset_BR = 8777
        self.magic_const1 = 21775
        self.magic_const2 = -1295
        
        self.is_idle = False 
        # true: iddle, false: pos_control
    
    def connect_and_calibrate(self):
        if self.is_enable == False:
            print("[ODRIVE] - Odłączony nie kalibruje")
            return
    
        print("[ODRIVE] - Szukam Odrive...")
        self.odrv = odrive.find_any()
        print("[ODRIVE] - Połączono, teraz kalibrajca")

        self.odrv.axis0.requested_state = AxisState.ENCODER_INDEX_SEARCH  #wyszukanie pozycji 0
        self.odrv.axis1.requested_state = AxisState.ENCODER_INDEX_SEARCH #wyszukanie pozycji 0
        while (self.odrv.axis1.current_state != AxisState.IDLE or 
               self.odrv.axis0.current_state != AxisState.IDLE):
            time.sleep(0.1)
            
        print("[ODRIVE] - Znaleziono pozycje początkową")
        time.sleep(3)
            
        self.odrv.axis0.controller.config.control_mode= 3
        self.odrv.axis1.controller.config.control_mode= 3
        
        self.odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        self.odrv.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL
    
        self.odrv.axis0.controller.pos_setpoint = 0
        self.odrv.axis1.controller.pos_setpoint = 0
        
        print("[ODRIVE] - Gotowy!!")
        
    def set_position(self, a: float, b: float): 
        if self.is_enable == False:
            return
        
        axis0_target = -a + self.offset_AL + self.magic_const1	
        axis1_target = -b + self.offset_BR + self.magic_const2
        
        self.odrv.axis0.controller.pos_setpoint = axis0_target
        self.odrv.axis1.controller.pos_setpoint = axis1_target


    def get_position(self) -> tuple[float, float]:
        if self.is_enable == False:
            return (0.0, 0.0)
        
        axis0_raw = self.odrv.axis0.encoder.pos_estimate
        axis1_raw = self.odrv.axis1.encoder.pos_estimate
        
        a = - axis0_raw + self.offset_AL + self.magic_const1
        b = - axis1_raw + self.offset_BR + self.magic_const2

        return a, b
    
    def setMode(self, mode):
        if self.is_enable:
            """ set mode 'idle' or 'pos'"""
            if mode == 'pos':
                print("[ODRIVE] - switched to pos controll")
                aktualna_poz_0 = self.odrv.axis0.encoder.pos_estimate
                aktualna_poz_1 = self.odrv.axis1.encoder.pos_estimate

                self.odrv.axis0.controller.pos_setpoint = aktualna_poz_0
                self.odrv.axis1.controller.pos_setpoint = aktualna_poz_1

                self.odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
                self.odrv.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL
            elif mode == 'idle':
                print("[ODRIVE] - switched to idle")
                self.odrv.axis0.requested_state = AxisState.IDLE
                self.odrv.axis1.requested_state = AxisState.IDLE
            
    def switch_mode(self):
        if self.is_idle:
            self.setMode('pos')
        else:
            self.setMode('idle') 
        self.is_idle = not self.is_idle                                   
    
class ArduinoController:
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200, is_enabled: bool = True):
        self.is_enabled = is_enabled
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        
        self._last_sent_z = 0 
        self._last_sent_c = 0 

    def connect(self):
        if not self.is_enabled:
            print("[Arduino] Odłączone nie łącze")
            return

        print(f"[Arduino] Szukam płytki na porcie {self.port}...")
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate)
            self.serial_conn.reset_output_buffer()
            self.serial_conn.reset_input_buffer()
            print("[Arduino] Połączono.")
        except serial.SerialException as e:
            print(f"BŁĄD: Nie można otworzyć portu {self.port}. {e}")
            self.is_enabled = False
            sys.exit(1)
            
    def send_positions(self, gam_steps: float, z_steps: float):
        if not self.is_enabled:
            return
        
        # Rzutujemy na int, ponieważ Arduino prawdopodobnie oczekuje liczb całkowitych dla kroków
        gam_int = int(gam_steps)
        z_int = int(z_steps)

        
        # W kodzie głównym było Zprev != Z
        if self._last_sent_z != z_int:
            cmd_z = f"z 0 {z_int}\n"
            self.serial_conn.write(cmd_z.encode('utf-8'))
            self._last_sent_z = z_int
            #print(f"cmdZ: {cmd_z}")
            
        # Komenda dla osi C
        if self._last_sent_c != gam_int:
            cmd_c = f"b 0 {gam_int}\n"
            self.serial_conn.write(cmd_c.encode('utf-8'))
            self._last_sent_c = gam_int
            #print(f"cmdC: {cmd_c}")
                
    def set_gripper(self, state: bool):
        if not self.is_enabled:
            return
        
        g_val = 1 if state else 0
        cmd_g = f"g {g_val} 0\n"
        print(f"Gripper set!!!!! {cmd_g}")
        self.serial_conn.write(cmd_g.encode('utf-8'))