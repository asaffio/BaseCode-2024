"""
============== UniBo: AI and Robotics 2024 ==============
Base code: gateway to the robot (or simulator)
This is a dummy version, most values and funtions are place-holders
It needs to be customized by the students for the different labs

(c) 2024 Alessandro Saffiotti
"""

parameters = {                  # these are for the Tiago robot
    'wheel_radius' : 0.0985,
    'wheel_axis' : 0.4044
}


def init_robot():
    """
    This should set up the communication channels with the robot (or simulator)
    and perform any initialization needed
    """
    print("Robot initialized")
    return parameters


def shutdown_robot():
    """
    This should perform any finalization needed on the robot,
    and close the communication channels
    """
    print("Robot shut")
