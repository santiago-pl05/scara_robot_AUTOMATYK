from scara_robot import ScaraRobot
from input_device import GamePadInput

from scara_kinematics.datatypes import CartesianPositions, JointPositions 

from time import time, sleep

FREQ = 80 * 2
LOOP_TIME = 1 / FREQ

def main():    
    xySpeedPadMultiplyer = 1.6 * 8 * (50 / FREQ) * 4
    xyAccelPad = 0.50 * (50 / FREQ)   # Szybki przyśpieszanie 
    xyDecelPad = 0.05 * (50 / FREQ)   # Wolne hamowanie
    
    CSpeedPadMultiplyer = 2 * (150 / FREQ)
    ZSpeedPadMultiplyer = 7 * (150 / FREQ)
    
    robot = ScaraRobot(use_arduino=True, use_odrv=False)
    pad = GamePadInput()
    
    robot.initHardware()
    
    prevCommandSpeedX = 0
    prevCommandSpeedY = 0
    
    
    # -- MAIN LOOP --
    while True:
        start_time=time()
        
        pad_state = pad.read_values()
        
        padX = pad_state.delta_x
        padY = pad_state.delta_y
        padC = pad_state.delta_c
        padZ = pad_state.delta_z
        
        pad_gripper = pad_state.gripper_toogle
        pad_mode = pad_state.mode_switch
        
        # print(f"Input pada - X: {padX} Y: {padY} C: {padC} Z:{padZ},") # d
        
        # sterowanie gripperem (deboune przeniesiony do input dev)
        if pad_gripper:
            robot.change_gripper()
            
        # przełączanie trybu 1
        if pad_mode:
            robot.odrive.switch_mode()
                        
        # --- sterowanie X ---
        # Czy to przyśpieszenie czy hamowanie
        if abs(padX) < abs(prevCommandSpeedX) or padX == 0:
            krokX = xyDecelPad
        else:
            krokX = xyAccelPad

        if((padX-prevCommandSpeedX) >= -krokX and (padX-prevCommandSpeedX) <= krokX):
            comandedSpeedX = padX
        elif((prevCommandSpeedX-padX) <- krokX):            
            comandedSpeedX = comandedSpeedX+krokX
        elif((prevCommandSpeedX-padX) > krokX):            
            comandedSpeedX = comandedSpeedX-krokX
            
        # --- sterowanie Y ---
        # Czy to przyśpieszenie czy hamowanie
        if abs(padY) < abs(prevCommandSpeedY) or padY == 0:
            krokY = xyDecelPad
        else:
            krokY = xyAccelPad

        if((padY-prevCommandSpeedY) >= -krokY and (padY-prevCommandSpeedY) <= krokY):
            comandedSpeedY = padY
        elif((prevCommandSpeedY-padY) <- krokY):            
            comandedSpeedY = comandedSpeedY+krokY
        elif((prevCommandSpeedY-padY) > krokY):            
            comandedSpeedY = comandedSpeedY-krokY

        X = robot.current_pos.X + comandedSpeedX * xySpeedPadMultiplyer
        Y = robot.current_pos.Y + comandedSpeedY * xySpeedPadMultiplyer
        
        C = robot.current_pos.C
        Z = robot.current_pos.Z
        
        prevCommandSpeedX=comandedSpeedX
        prevCommandSpeedY=comandedSpeedY

        # Sterowanie C z starego kodu
        if(padC==1):
            C = robot.current_pos.C + CSpeedPadMultiplyer
            #robot.arduino.send_positions(robot.current_pos.C, robot.current_pos.Z)
        elif(padC==-1):
            C = robot.current_pos.C - CSpeedPadMultiplyer             
            #robot.arduino.send_positions(robot.current_pos.C, robot.current_pos.Z)            
        # Sterowanie Z z starego kodu
        if(padZ==1):
            Z = robot.current_pos.Z + ZSpeedPadMultiplyer 
            #robot.arduino.send_positions(robot.current_pos.C, robot.current_pos.Z)            
    
        elif(padZ==-1):
            Z = robot.current_pos.Z - ZSpeedPadMultiplyer 
            #robot.arduino.send_positions(robot.current_pos.C, robot.current_pos.Z)            
            
        target_pos = CartesianPositions(X, Y, C, Z) 
        # print(target_pos)
        robot.move_to_pos(target_pos)
        
        cur_time = time()
        robot.loop(cur_time)
        elapsed_time = cur_time - start_time
        
        if elapsed_time < LOOP_TIME:
            sleep(LOOP_TIME - elapsed_time)
    
if __name__ == '__main__':
    main()