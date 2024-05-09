"""
============== UniBo: AI and Robotics 2024 ==============
Base code: top level execution loop
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
import time, math
import robot_gwy, robot_map, observer, controller, pyhop

class TopLevelLoop:
    """
    Top level global execution loop
    """

    def __init__(self,
                 goal  = [],            # top level task, passed to the HTN planner
                 mypose = None,         # robot's starting pose, if None take it from the map
                 tcycle = 0.1 ,         # cycle time, in sec
                 debug = 0,             # debug level, use to decide what debug info to print
                                        # (0: none, 1: only outer SPA loop, 2: inner control loop, 3: details)
                 ):
        self.goal = goal
        self.tcycle = tcycle
        self.debug = debug
        self.mypose = mypose            # estimated robot's pose
        self.pars  = {}                 # robot's parameters
        self.ctr = None                 # controller instance

    def print_pose(self, mypose):
        print('pose = ({:.2f}, {:.2f}, {:.2f})\n'.format(mypose[0], mypose[1], math.degrees(mypose[2]))) 
        
    def run (self, maxsteps = 0, goal = None):
        """
        Main entry point: run this instance of the Top Level loop
        'maxsteps' can be used as a timeout to abort execution after a certain number of cycles
        """
        if goal:
            self.goal = goal
        if self.mypose == None:
            self.mypose = robot_map.map.startpose
        self.pars = robot_gwy.init_robot()          # get the robot's parameter from the robot gateway
        observer.init_pose(self.pars)               # init the observer
        self.ctr = controller.Controller()          # init the controller   
        self.sense_plan_act(self.goal, maxsteps)    # go into the main SPA loop
        robot_gwy.shutdown_robot()                  # done

    def sense_plan_act(self, goal, maxsteps = 0):
        """
        The outer "sense plan act" loop
        """
        from htn_domain import State

        # the 'S' part, fill the state defined in our domain with current data
        state = State()
        self.get_state(state)
        if self.debug > 0:
            print("Planner called from initial state:")
            pyhop.print_state(state)

        # the 'P' part, generate a plan for the given goal in the current state
        plan = pyhop.pyhop(state, goal, verbose=0)
        if self.debug > 0:
            print("Planner returns plan:", plan)
        if plan:

            # the 'A' part, execute the plan action by action
            result = self.execute_plan(plan, maxsteps)       # A = Act
            if self.debug > 0:
                if result:
                    print("Plan execution completed!")
                else:
                    print("Plan execution failed!")
            return result
        else:
            if self.debug > 0:
                print("No plan found!")
        return None
    
    def get_state (self, state):
        """
        Fill the given state with the current values, taken from the map
        or from the robot's sensors
        """
        # first we set the static part of the state, taken from the map
        # see in the map which objects (doors) connect rooms to one another
        for room1 in robot_map.map.topology:
            for room2 in robot_map.map.topology:
                if room1 == room2:
                    continue
                for object1 in robot_map.map.topology[room1]:
                    for object2 in robot_map.map.topology[room2]:
                        if object1 == object2:
                            state.connects[object1] = (room1, room2)
        # set their status
        for door in state.connects:
            state.door[door] = robot_map.map.properties[door]
        # see what is the room of each object (except the doors above)
        for room, contents in robot_map.map.topology.items():
            for object in contents:
                if object in state.connects:
                    continue
                state.room[object] = room

        # second we set the dynamic part of the state, updated through the robot's (real or virtual) sensors
        state.room['me'] = robot_map.map.find_room(self.mypose)
        state.pos['me']  = robot_map.map.find_location(self.mypose)
        state.pos['box1'] = robot_gwy.get_box_position('box1')
        state.pos['box2'] = robot_gwy.get_box_position('box2')
        state.pos['box3'] = robot_gwy.get_box_position('box3')

    def execute_plan (self, plan, maxsteps):
        """
        The "Act" part of the SPA loop
        Pop each action in the plan in sequence, and execute it
        Return True for successful plan execution, False for failure
        """
        if self.debug > 0:
            print("Executing plan")
        for action in plan:
            result = self.execute_action(action, maxsteps)
            if result == False:
                break
        return result

    def execute_action(self, action, threshold = 0.9, maxsteps = 0):
        """
        The inner action execution loop
        Execute 'action' until its degree of achievement is greter than 'threshold'
        timeout with failure after 'maxsteps' steps (zero = no timeout)
        Return True for successful execution, False for failure
        """
        if self.debug > 0:
            print("Executing action:", action)
        nsteps = 0

        # set the behavior to be run by the controller
        behavior = action[0]
        if behavior == 'Open' or behavior == 'Close':  # because these call a service using the door's name
            param = action[1]
        else:
            param = robot_map.map.locations[action[1]]
        self.ctr.set_behavior(bname = behavior, bparam = param)

        # run the main control pipeline until completion, or failure
        while True:
            # here you should check that ROS is still running
            # if rospy.is_shutdown():                     # ROS was killed
            #   return False
            nsteps += 1
            if (maxsteps > 0 and nsteps < maxsteps):    # timeout
                if self.debug > 0:
                    print("Max number of steps reached: exiting")
                return False
            result, done = self.step()                  # execute control pipeline
            if result == False:                         # behavior failure
                if self.debug > 0:
                    print("Action", action, "failed")
                return False                            # action failed
            if done > threshold:                        # behavior completed
                if self.debug > 0:
                    print("Action", action, "completed")
                return True                             # action completed

    def step (self):
        """
        The basic control pipeline: read sensors, estimate state, decide controls, send controls
        Return True for successful execution, plus the current degree of achievement
        """
        wl, wr = robot_gwy.get_wheel_encoders()             # read proprioceptive sensors (wheel rotations)
        sdata  = robot_gwy.get_sonar_data()                 # read exteroceptive sensors (sonars)
        if self.debug > 1:
            print('sonars =', ' '.join(['{:.2f}'.format(s[1]) for s in sdata]))

        self.mypose = observer.update_pose(wl, wr)          # estimate robot state (pose)
        state = {'mypose' : self.mypose, 'sdata' : sdata}   # state passed to the controller

        achieved = self.ctr.run(state, self.debug)          # compute controls (robot's vels)
        vlin = self.ctr.get_vlin()                          # retrieve computed controls (vlin)
        vrot = self.ctr.get_vrot()                          # retrieve computed controls (vrot)

        robot_gwy.set_vel_values(vlin, vrot)                # send controls
        if self.debug > 1:
            self.print_pose(self.mypose)

        time.sleep(self.tcycle)                             # sync on a constant cycle time
        return True, achieved

