from scara_robot import ScaraRobot
from input_device import GamePadInput
from planners.manual_planner import ManualTeleopPlanner
from planners.autonomous_planner import AutonomousTrajectoryPlanner

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
    
    robot = ScaraRobot(use_arduino=False, use_odrv=True)
    pad = GamePadInput()
    planner = ManualTeleopPlanner(pad, FREQ)
    #planner = AutonomousTrajectoryPlanner()
    
    robot.initHardware()
    
    # -- MAIN LOOP --
    while True:
        start_time=time()
        
        pad_state = pad.read_values()
        
        pad_gripper = pad_state.gripper_toogle
        pad_mode = pad_state.mode_switch
        
        # print(f"Input pada - X: {padX} Y: {padY} C: {padC} Z:{padZ},") # d
        
        # sterowanie gripperem
        if pad_gripper:
            robot.change_gripper()
            
        # przełączanie trybu 1
        if pad_mode:
            robot.odrive.switch_mode()

        target_pos = planner.get_position(robot.current_pos, LOOP_TIME)
        robot.move_to_pos(target_pos, LOOP_TIME)
        
        cur_time = time()
        robot.loop(cur_time)
        elapsed_time = cur_time - start_time
        
        if elapsed_time < LOOP_TIME:
            sleep(LOOP_TIME - elapsed_time)
    
if __name__ == '__main__':
    main()