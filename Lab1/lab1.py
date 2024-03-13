#!/usr/bin/env python3
import rospy





if __name__ == '__main__':
    rospy.init_node('tiago_controller_lab1', anonymous=False)
    tiago_localizer = Localizer(WHEEL_RADIUS, WHEEL_SEPARATION, 3, 0, 0)
    tiago_controller = GoToTarget()
    tiago_interface = TiagoSimuInterface()
    tiago_controller.target.x = 5.5
    tiago_controller.target.y = 1
    navigation = navigator.TopLevel(localizer)
    while not rospy.is_shutdown():
        #get joint
        wl, wr = tiago_interface.get_wheel_pos()
        current_pos = tiago_localizer.update(wl, wr)
        print(current_pos)

        #update controller
        tiago_controller.set_pose(current_pos)
        tiago_controller.update_state()
        tiago_controller.run()
        tiago_interface.send_vel(tiago_controller.get_vlin(), tiago_controller.get_vrot())
        
        print(tiago_controller.get_vlin(), tiago_controller.get_vrot())
        print("--------------------")