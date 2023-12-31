import math
import time
from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script uses the perception pipeline to pick up objects and place them in some virtual basket on the left side of the robot
# It also uses the AR tag on the arm to get a better idea of where the arm is relative to the camera (though the URDF is pretty accurate already).
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx200 use_perception:=true'
# Then change to this directory and type 'python pick_place_no_armtag.py'

def main():
    bot = InterbotixLocobotXS("locobot_wx250s", arm_model="mobile_wx250s")
    # move camera such that it's tilting down
    bot.camera.pan_tilt_move(0, 0.75)
    # get the positions of any clusters present w.r.t. the 'locobot/arm_base_link'
    # sort the clusters such that they appear from left-to-right w.r.t. the 'locobot/arm_base_link'
    success, clusters = bot.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)
    print(clusters)
    # move the robot back so it's centered and open the gripper
    bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
    bot.gripper.open()

    # pick up each object from left-to-right and drop it in a virtual basket on the left side of the robot
    for cluster in clusters:
        x, y, z = cluster["position"]
        bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
        bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
        bot.gripper.close()

        bot.arm.set_ee_pose_components(x=0.3, z=0.2)
        bot.camera.pan_tilt_move(0, 0)
        time.sleep(5)

        #bot.base.move_to_pose(-1.5, 0, 3, True)
        time.sleep(40)

        bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.25)
        bot.gripper.open()
        time.sleep(5)

        bot.arm.go_to_sleep_pose()

        #bot.base.move_to_pose(0, 0, 0, True)
        time.sleep(20)

        bot.camera.pan_tilt_move(0, 0.75)
        time.sleep(5)

if __name__=='__main__':
    main()
