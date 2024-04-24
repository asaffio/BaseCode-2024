"""
============== UniBo: AI and Robotics 2024 ==============
Base code: dummy observer, performs the state estimation parts of the main loop
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
import math

WHEEL_RADIUS = 0.0
WHEEL_AXIS   = 0.0


def init_pose (robot_pars):
    """
    Initialize the observer, using the given robot's parameters
    Return True if the inizialization was successful
    """
    WHEEL_RADIUS = robot_pars['wheel_radius']
    WHEEL_AXIS   = robot_pars['wheel_axis']
    return True


def update_pose (wl, wr):
    """
    Incremental position estimation: update the current pose (x, y, th) of the robot
    taking into account the newly read positions (rotations) of the left and right wheels
    Returns the new robot's pose, as a triple (x, y, th)
    """
    return (0.0, 0.0, 0.0)

