
GOAL_POSITION = (0, 0)

def init_controls (goal):
    """
    Initialize the controller, setting the global goal position
    Return True if the inizialization was successful
    """
    GOAL_POSITION = goal
    return True


def compute_ctr (mypose):
    """
    Action decision. Compute control values (vlin, vrot) given the
    current robot's pose and the global GOAL POSITION
    Returns the control values (vlin, vrot), as a pair
    """
    return (0.0, 0.0)
