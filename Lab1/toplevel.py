"""
============== UniBo: AI and Robotics 2024 ==============
Base code: top level execution loop
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
import time
import robot_gwy
import observer
import controller

class TopLevel:
    """
    Top level execution loop
    """

    def __init__(self,
                tcycle = 0.1,       # cycle time, in sec
                maxcycles = 0,      # if >0, exit after these many cycles
                ):
        self.tcycle = tcycle
        self.maxcycles = maxcycles
        params = self.simulator.robot.get_parameters()

    def step(self, debug = 0):
        """
        Execute one step of this instance of the Top Level loop
        debug level can be used to selectively print debug information
        """
        wl, wr = robot_gwy.get_wheel_encoders()     # read sensors (wheel rotations)
        mypose = observer.update_pose(wl, wr)       # estimate state (robot's pose)
        vlin, vrot = controller.run(mypose)         # decide action (robot's vel)
        robot_gwy.send_vel_controls(vlin, vrot)     # send action

    def run(self, debug = 0):
        """
        Run this instance of the Top Level loop
        debug level can be used to selectively print debug information
        """
        nstep = 0
        robot_gwy.init_robot()
        observer.init_pose()
        controller.init_control()
        while True:
            self.step(debug)
            time.sleep(self.tcycle)
            nstep += 1
            if self.maxcycles > 0:
                if nstep > self.maxcycles:
                    break
        robot_gwy.shutdown_robot()


""" Create and run a test instance """
    
controller = TopLevel(cycle = 0.1)
controller.run(debug = 0)
