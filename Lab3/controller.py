"""
============== UniBo: AI and Robotics 2024 ==============
Base code: dummy controller, performs the action decision parts of the main loop
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
from fcontrol import Behavior
from fcontrol import ramp_up, ramp_down, triangle
from fcontrol import global_to_local, local_to_global
from math import atan2, degrees, sqrt


class GoToTarget (Behavior):
    """
    Instance of the Behavior class, which defines a fuzzy controller
    to navigate to a given target point target = (x,y)
    The setup function is called once when the behavior is created,
    and it creates all the fuzzy predicates, actions and rules that
    define the behavior's control strategy.
    The update_state function is called at every cycle and it sets
    the internal variables based on the passed robot's state
    The run function is also called at every cycle, and it runs
    the fuzzy controller with the behavior's rules and sets the
    output control variables vlin, vrot: this function is inherited
    from the FControl class (superclass of Behavior)
    The values of vlin,vrot are fetched via the get_vlin and get_vrot
    functions, which are inherited from the Behavior class
    """
    def __init__(self, target = (0.0, 0.0)):
        super().__init__()
        self.target = [target[0], target[1], 0.0]   # target point in global coordinates
        self.tlocal = [0, 0, 0]                     # target point in robot's coordinates
    
    def update_state(self, state):
        """
        Update the relevant local state variables (self.state) at every control cycle
        Uses the 'mypose' estimate, passed as part of the state by the top-level loop
        Set the local state variables 'phi' and 'rho'
        """
        mypose = state['mypose']
        global_to_local(self.target, mypose, self.tlocal)
        xt = self.tlocal[0]
        yt = self.tlocal[1]
        self.state['phi'] = degrees(atan2(yt, xt))
        self.state['rho'] = sqrt(xt * xt + yt * yt)

    def setup(self):
        """
        Definition of the rules of our 'go to target' fuzzy controller
        Fuzzy interpretation is built from the local state variables 'phi' and 'rho'
        """

        self.fpreds = {
            # Definition of the fuzzy predicates, used in the rules' LHS
            'TargetLeft'  : (ramp_up(5.0, 60.0), 'phi'),
            'TargetRight' : (ramp_down(-60.0, -5.0), 'phi'),
            'TargetAhead' : (triangle(-60.0, 0.0, 60.0), 'phi'),
            'TargetHere'  : (ramp_down(0.1, 2.0), 'rho')
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':0.5, 'Slow':0.1, 'None':0, 'Back':-0.1}, 'Vlin'),
            'Turn' : ({'Left':40, 'MLeft':10, 'None':0, 'MRight':-10, 'Right':-40}, 'Vrot')
        }

        self.frules = {
            # Lastly, definition of the actual fuzzy rules
            'ToLeft'  : ("TargetLeft AND NOT(TargetHere)", 'Turn', 'Left'),
            'ToRight' : ("TargetRight AND NOT(TargetHere)", 'Turn', 'Right'),
            'Far'     : ("TargetAhead AND NOT(TargetHere)", 'Move', 'Fast'),
            'Stop'    : ("TargetHere", 'Move', 'None')
        }

        # The degree of achievement is given by the predicate 'TargetHere'
        self.fgoal = "TargetHere"

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()
            


behavior = None     # which behavior we are currently executing


def init_controls (goal):
    """
    Initialize the controller, setting the global goal position
    Return True if the inizialization was successful
    """
    global behavior
    behavior = GoToTarget(goal)
    # behavior = Avoid()
    return True


def compute_ctr (state, debug):
    """
    Action decision. Compute control values (vlin, vrot) given the current robot's pose
    Returns the control values (vlin, vrot), as a pair
    """
    global behavior
    achieved = behavior.run(state, debug)
    vlin = behavior.get_vlin()
    vrot = behavior.get_vrot()
    if debug > 0:
        print('Goal achievement: {:.2f}'.format(achieved))
    if debug > 0:
        print('(vlin, vrot)) = ({:.2f}, {:.2f})'.format(vlin, degrees(vrot))) 
    return (vlin, vrot)
