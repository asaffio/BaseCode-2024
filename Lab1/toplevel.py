"""
============== UniBo: AI and Robotics 2024 ==============
Base code: top level execution loop
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
import time, math
import robot_gwy
import observer
import controller

class TopLevelLoop:
    """
    Top level execution loop
    """

    def __init__(self,
                 goal = (0, 0),         # goal position
                 tcycle = 0.1 ,         # cycle time, in sec
                 debug = 0              # debug level, use to decide to print debug info
                 ):
        self.tcycle = tcycle
        self.goal = goal
        self.debug = debug

    def step(self):
        """
        Execute one step of this instance of the Top Level loop
        debug level can be used to selectively print debug information
        """
        wl, wr = robot_gwy.get_wheel_encoders()      # read sensors (wheel rotations)
        mypose = observer.update_pose(wl, wr)        # estimate state (robot's pose)
        vlin, vrot = controller.compute_ctr(mypose)  # decide action (robot's vel)
        robot_gwy.set_vel_values(vlin, vrot)         # send control action
        if self.debug > 0:
            print('vlin = {:.2f}, vrot = {:.2f})'.format(vlin, math.degrees(vrot))) 
        return True                                  # return False to exit the loop

    def run(self):
        """
        Run this instance of the Top Level loop
        debug level can be used to selectively print debug information
        """
        nstep = 0
        pars = robot_gwy.init_robot()
        observer.init_pose(pars)
        controller.init_controls(self.goal)
        while self.step():
            time.sleep(self.tcycle)
        robot_gwy.shutdown_robot()

