"""
============== UniBo: AI and Robotics 2024 ==============
Base code: dummy controller, performs the action decision parts of the main loop
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""


def init_controls (goal):
    """
    Initialize the controller
    Return True if the inizialization was successful
    """
    return True


def compute_ctr (state, debug):
    """
    Action decision. Compute control values (vlin, vrot) given the current robot's pose
    Returns the control values (vlin, vrot), as a pair
    """
    return (0.0, 0.0)
