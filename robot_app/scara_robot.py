from hardware import ArduinoController, OdriveController

from scara_kinematics import ScaraKinematics, CartesianPositions, JointPositions

class ScaraRobot:
    def __init__(self, use_arduino: bool, use_odrv: bool):
        self.kinematics = ScaraKinematics()
        
        self.arduino = ArduinoController(is_enabled=use_arduino, port='/dev/ttyUSB0', baudrate=9600)
        self.odrive = OdriveController(is_enable=use_odrv)
        
        self.current_pos = CartesianPositions(0, 0, 0, 0)
        self.current_joints = JointPositions(0, 0, 0, 0)
        self.gripper_state = False
    
        self.last_update_time = 0

        self.MAX_SPEED_XY = 300  # mm/s
        self.MAX_SPEED_C = 100   # rad/s
        self.MAX_SPEED_Z = 100   # mm/s
        
    def initHardware(self):
        print("Inicjalizajca hardwaru robota...")
        
        self.odrive.connect_and_calibrate()
        self.arduino.connect()
        
        A, B = self.odrive.get_position()
        
        start_joint = JointPositions(A=A, B=B, gam=0, Z=300) #wartosci początkowe z kodu

        self.current_pos = self.kinematics.forward_kinematics(start_joint)
        
        print("Hardware zaincjalizowany")
        print(f"Pozycja początkowa: X={self.current_pos.X} Y={self.current_pos.Y} C={self.current_pos.C} Z={self.current_pos.Z}")
        
        
    def move_to_pos(self, position: CartesianPositions, dt: float):
        # ograniczenia w prędkości 
        #mozna w przyszłosci tu dodac ograniczenie przyśpieszania i hamowania
        if abs(position.X - self.current_pos.X) > dt * self.MAX_SPEED_XY:
            sign_x = 1 if position.X > self.current_pos.X else -1
            position.X = self.current_pos.X + sign_x * dt * self.MAX_SPEED_XY
        if abs(position.Y - self.current_pos.Y) > dt * self.MAX_SPEED_XY:
            sign_y = 1 if position.Y > self.current_pos.Y else -1
            position.Y = self.current_pos.Y + sign_y * dt * self.MAX_SPEED_XY
        if abs(position.C - self.current_pos.C) > dt * self.MAX_SPEED_C:
            sign_c = 1 if position.C > self.current_pos.C else -1
            position.C = self.current_pos.C + sign_c * dt * self.MAX_SPEED_C
        if abs(position.Z - self.current_pos.Z) > dt * self.MAX_SPEED_Z:
            sign_z = 1 if position.Z > self.current_pos.Z else -1
            position.Z = self.current_pos.Z + sign_z * dt * self.MAX_SPEED_Z
        
        target_joints = self.kinematics.inverse_kinematics(position)
        
        if target_joints is None or not self.kinematics.check_limits(target_joints):
            print("Punkt poza zasięgiem (Out Of Range)!")
            return False
        
        target_joints.gam = self.kinematics.check_limits_c(target_joints.gam)
        
        self.odrive.set_position(target_joints.A, target_joints.B)
        #print(target_joints.gam, target_joints.Z)
        self.arduino.send_positions(target_joints.gam, target_joints.Z)
        
        #self.current_pos = position
        #self.current_joints = target_joints
        
        return True
    
    def set_gripper(self, state: bool):
        self.gripper_state = state
        self.arduino.set_gripper(state)
        
    def change_gripper(self):
        self.gripper_state = not self.gripper_state
        self.arduino.set_gripper(self.gripper_state)
        
    def loop(self, cur_time):
        time_since_elapse = cur_time - self.last_update_time
        
        actual_a, actual_b = self.odrive.get_position()
        
        self.current_joints.A = actual_a
        self.current_joints.B = actual_b
        
        self.current_joints.gam = self.arduino._last_sent_c
        self.current_joints.Z = self.arduino._last_sent_z
        
        self.current_pos = self.kinematics.forward_kinematics(self.current_joints)
        
        if time_since_elapse >= 0.1:
            self.last_update_time = cur_time
            print(f"""Obecna pozcyja X: {self.current_pos.X} 
                  Y: {self.current_pos.Y}
                  C: {self.current_pos.C}
                  Z: {self.current_pos.Z}""")
            