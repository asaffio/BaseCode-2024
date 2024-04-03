"""
============== UniBo: AI and Robotics 2024 ==============
Base code: gateway to the robot (or simulator)
This is a dummy version, most values and funtions are place-holders
It needs to be customized by the students for the different labs

(c) 2024 Alessandro Saffiotti
"""
import math

def sonar_ring_pose(bearing):
    rho = 0.260
    phi = math.radians(bearing)
    x = rho * math.cos(phi)
    y = rho * math.sin(phi)
    return (x, y, phi)

parameters = {                        # these are for the Tiago robot
    'wheel_radius' : 0.0985,
    'wheel_axis' : 0.4044,
    'sonar_num' : 12,
    'sonar_maxrange' : 3.0,
    'sonar_delta' : math.radians(25.0),
    'sonar_poses' : [sonar_ring_pose(0.0),
                     sonar_ring_pose(30.0),
                     sonar_ring_pose(60.0),
                     sonar_ring_pose(90.0),
                     sonar_ring_pose(120.0),
                     sonar_ring_pose(150.0),
                     sonar_ring_pose(180.0),
                     sonar_ring_pose(210.0),
                     sonar_ring_pose(240.0),
                     sonar_ring_pose(270.0),
                     sonar_ring_pose(300.0),
                     sonar_ring_pose(330.0)]
}


def init_robot ():
    """
    This should set up the communication channels with the robot (or simulator)
    and perform any initialization needed
    """
    print("Robot initialized")
    return parameters


def shutdown_robot ():
    """
    This should perform any finalization needed on the robot,
    and close the communication channels
    """
    print("Robot shut")


def get_wheel_encoders ():
    """
    Get current values of wheel encoders, which indicate the current position
    of each wheel in radiants
    """
    return (0.0, 0.0)


def get_sonar_data ():
    """
    Get current values from the sonar ring
    Returns an array of readings in the form (sonar pose, range)
    Sonar pose is (x, y, th) in robot's base frame
    This dummy version returns range 0.0 for all sonars
    """
    res = []
    for i in range(parameters['sonar_num']):
        res.append((parameters['sonar_poses'][i], 0.0))
    return res


def set_vel_values (vlin, vrot):
    """
    Set new linear and rotational velocities for the robot's base
    vlin is m/sec vrot is rad/sec
    Returns True if successful
    """
    return True
