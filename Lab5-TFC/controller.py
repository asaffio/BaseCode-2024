"""
============== UniBo: AI and Robotics 2024 ==============
Base code: dummy controller, performs the action decision parts of the main loop
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
from fcontrol import Behavior
from fcontrol import ramp_up, ramp_down, triangle, trapezoid
from math import degrees

class GoTo (Behavior):
    """
    Navigate to a target object at (x,y,th,radius), where 'x,y' is the object's center
    in global frame, and 'radius' is its size. Use sonars for local obstacle avoidance.
    """
    def __init__(self, target = (0.0, 0.0, 0.0, 0.0)):
        print("Instantiated GoTo behavior with target =", target)
        super().__init__()
    
    def update_state(self, state):
        pass

    def setup(self):
        self.fgoal = "True"


class Cross (Behavior):
    """
    Cross a door at (x,y,th,radius), where 'x,y' is the door's center in global frame,
    and 'radius' is its size. Use sonars  for local obstacle avoidance.
    """
    def __init__(self, door = (0.0, 0.0, 0.0, 0.0)):
        print("Instantiated Cross behavior with door =", door)
        super().__init__()

    def update_state(self, state):
        pass

    def setup(self):
        self.fgoal = "True"


class Open (Behavior):
    """
    Open a door by a given name, just calling the relevant ROS service
    """
    def __init__(self, door):
        print("Instantiated Close behavior with door =", door)
        self.doorname = door
        super().__init__()
    
    def update_state(self, state):
        """
        from robot_gwy import door_client
        if self.doorname == 'D1':
            door_client.door1_update_pos(True)
        elif self.doorname == 'D2':
            door_client.door2_update_pos(True)
        elif self.doorname == 'D3':
            door_client.door3_update_pos(True)
        elif self.doorname == 'D4':
            door_client.door4_update_pos(True)
        else:
            print("'Open' called with wrong parameter:", self.doorname)
        """
        pass

    def setup(self):
        self.fgoal = "True"


class Close (Behavior):
    """
    Close a door by a given name, just calling the relevant ROS service
    """
    def __init__(self, door):
        self.doorname = door
        print("Instantiated Close behavior with door =", door)
        super().__init__()

    def update_state(self, state):
        pass

    def setup(self):
        self.fgoal = "True"


class Controller ():
    def __init__(self):
        self.behavior = None        # top-level fuzzy behavior run by the controller
        self.achieved = 0.0         # level of achievement of current behavior
        self.vlin = 0.0             # current value for vlin control variable
        self.vrot = 0.0             # current value for vrot control variable

    def set_behavior (self, bname = None, bparam = None):
        """
        Initialize the controller, setting the behavior to be executed
        Return True if the inizialization is successful
        """
        from robot_map import map
        if bname:
            self.behavior = globals()[bname](bparam)
        return True

    def run (self, state, debug):
        """
        Action decision. Compute control values (vlin, vrot) given the current robot's pose
        by running the current behavior; return the level of achievement of that behavior
        """
        self.achieved = self.behavior.run(state, debug)
        self.vlin = self.behavior.get_vlin()
        self.vrot = self.behavior.get_vrot()
        if debug > 1:
            print('Goal achievement: {:.2f}'.format(self.achieved))
        if debug > 1:
            print('(vlin, vrot)) = ({:.2f}, {:.2f})'.format(self.vlin, degrees(self.vrot))) 
        return self.achieved

    def get_vlin (self):
        return self.vlin
    
    def get_vrot (self):
        return self.vrot
