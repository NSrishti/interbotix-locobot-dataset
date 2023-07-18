from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script commands the base to move to an arbitrary point on a map
# Note that this script assumes you already have a map built
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx200 use_nav:=true localization:=true'
# Then change to this directory and type 'python move_base.py'

def main():
    locobot = InterbotixLocobotXS(robot_model="locobot_wx250s", arm_model="mobile_wx250s", use_move_base_action=True)
    locobot.camera.pan_tilt_move(0, 0)
    locobot.base.move_to_pose(0, 0, 0, True)
    
    z = 1
    while(z<6):
        locobot.base.move_to_pose(0, 0, z, True)
        z = z+1
        
    locobot.base.move_to_pose(0, 0, 0, True)

if __name__=='__main__':
    main()
