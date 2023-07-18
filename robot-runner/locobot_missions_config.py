from EventManager.Models.RobotRunnerEvents import RobotRunnerEvents
from EventManager.EventSubscriptionController import EventSubscriptionController
from ConfigValidator.Config.Models.RunTableModel import RunTableModel
from ConfigValidator.Config.Models.FactorModel import FactorModel
from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext
from ConfigValidator.Config.Models.OperationType import OperationType

import os
import time
import signal
import subprocess
import shutil
from typing import Dict, List
from pathlib import Path

class RobotRunnerConfig:
    # =================================================USER SPECIFIC NECESSARY CONFIG=================================================
    # Name for this experiment
    name:                       str             = "locobot_test"
    # Required ROS version for this experiment to be ran with 
    # NOTE: (e.g. ROS2 foxy or eloquent)
    # NOTE: version: 2
    # NOTE: distro: "foxy"
    required_ros_version:       int             = 1
    required_ros_distro:        str             = "noetic"
    # Experiment operation types
    operation_type:             OperationType   = OperationType.AUTO
    # Run settings
    time_between_runs_in_ms:    int             = 3600000
    # Path to store results at
    # NOTE: Path does not need to exist, will be appended with 'name' as specified in this config and created on runtime
    results_output_path:        Path            = Path("~/Documents/experiments")
    # =================================================USER SPECIFIC UNNECESSARY CONFIG===============================================

    exp_proc = None

    # Dynamic configurations can be one-time satisfied here before the program takes the config as-is
    # NOTE: Setting some variable based on some criteria
    def __init__(self):
        """Executes immediately after program start, on config load"""

        EventSubscriptionController.subscribe_to_multiple_events([ 
            (RobotRunnerEvents.START_RUN,           self.start_run),
            (RobotRunnerEvents.LAUNCH_MISSION,      self.launch_mission),
            (RobotRunnerEvents.STOP_RUN,            self.stop_run),
        ])

    def create_run_table(self) -> List[Dict]:
        """Create and return the run_table here. A run_table is a List (rows) of tuples (columns), 
        representing each run robot-runner must perform"""
        run_table = RunTableModel(
            factors = [
                FactorModel("missions", ['MI-01-ObjectDetection-KeepNear', 'MI-02-ObjectDetection-KeepFar', 
                'MI-03-ObjectDetection-KeepLeft', 'MI-04-ObjectDetection-KeepRight,
                 'MI-05-ObjectDetection-KeepAbove', 'MI-06-ObjectDetection-KeepBelow', 
                 'MI-07-ObjectDetection-KeepFront','MI-08-ObjectDetection-KeepBehind',
                 'MI-09-KeepNear', 'MI-10-KeepFar', 'MI-11-KeepLeft', 'MI-12-KeepRight,
                 'MI-13-KeepAbove', 'MI-14-KeepBelow', 'MI-15-KeepFront','MI-16-KeepBehind',
                 'MI-17-Perception', 'MI-18-Mapping', 'MI-19-Mapping-ObjectDetection',
                  'MI-20-Navigation', 'MI-21-KeepStill',]),
                FactorModel("nr_of_runs", range(0, 10))
            ],
            
        )
        run_table.create_experiment_run_table()
        return run_table.get_experiment_run_table()

    def before_experiment(self, context: RobotRunnerContext):
        print("\n\n ---> NOTE: This mission will only work if the package:  ros-noetic  interbotix_ws and yolov5 has been installed.")

    def start_run(self, context: RobotRunnerContext) -> None:
        """Perform any activity required for starting the run here. 
        Activities before and after starting the run should also be performed here."""
        
        variation = context.run_variation
        operation = variation['missions']
        
        if operation == 'MI-01-ObjectDetection-KeepNear':
            cmd = "(cd /home/locobot/bash_files/; ./yolov.sh; gnome-terminal -- ./monitor.sh 1; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-02-ObjectDetection-KeepFar':
            cmd = "(cd /home/locobot/bash_files/; ./yolov.sh; gnome-terminal -- ./monitor.sh 2; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-03-ObjectDetection-KeepLeft':
            cmd = "(cd /home/locobot/bash_files/; ./yolov.sh; gnome-terminal -- ./monitor.sh 3; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-04-ObjectDetection-KeepRight':
            cmd = "(cd /home/locobot/bash_files/; ./yolov.sh; gnome-terminal -- ./monitor.sh 4; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-05-ObjectDetection-KeepAbove':
            cmd = "(cd /home/locobot/bash_files/; ./yolov.sh; gnome-terminal -- ./monitor.sh 5; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-06-ObjectDetection-KeepBelow':
            cmd = "(cd /home/locobot/bash_files/; ./yolov.sh; gnome-terminal -- ./monitor.sh 6; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
        
        if operation == 'MI-07-ObjectDetection-KeepFront':
            cmd = "(cd /home/locobot/bash_files/; ./yolov.sh; gnome-terminal -- ./monitor.sh 7; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
        
        if operation == 'MI-08-ObjectDetection-KeepBehind':
            cmd = "(cd /home/locobot/bash_files/; ./yolov.sh; gnome-terminal -- ./monitor.sh 8; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
        
        if operation == 'MI-09-KeepNear':
            cmd = "(cd /home/locobot/bash_files; ./near.sh; gnome-terminal -- ./monitor.sh 9; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-10-KeepFar':
            cmd = "(cd /home/locobot/bash_files/; ./far.sh; gnome-terminal -- ./monitor.sh 10; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-11-KeepLeft':
            cmd = "(cd /home/locobot/bash_files/; ./left.sh; gnome-terminal -- ./monitor.sh 11; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-12-KeepRight':
            cmd = "(cd /home/locobot/bash_files/; ./right.sh; gnome-terminal -- ./monitor.sh 12; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-13-KeepAbove':
            cmd = "(cd /home/locobot/bash_files/; ./above.sh; gnome-terminal -- ./monitor.sh 13; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-14-KeepBelow':
            cmd = "(cd /home/locobot/bash_files/; ./below.sh; gnome-terminal -- ./monitor.sh 14; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-15-KeepFront':
            cmd = "(cd /home/locobot/bash_files/; ./front.sh; gnome-terminal -- ./monitor.sh 15; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-16-KeepBehind':
            cmd = "(cd /home/locobot/bash_files/; ./behind.sh; gnome-terminal -- ./monitor.sh 16; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 

        if operation == 'MI-17-Perception':
            cmd = "(cd /home/locobot/bash_files/; ./perception.sh; gnome-terminal -- ./monitor.sh 17; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-18-Mapping':
            cmd = "(cd /home/locobot/bash_files/; gnome-terminal -- ./monitor.sh 18; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-19-Mapping-ObjectDetection':
            cmd = "(cd /home/locobot/bash_files/; gnome-terminal -- ./monitor.sh 19; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-20-Navigation':
            cmd = "(cd /home/locobot/bash_files/; ./search.sh; gnome-terminal -- ./monitor.sh 20; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
    
        if operation == 'MI-21-KeepStill':
            cmd = "(cd /home/locobot/bash_files/; gnome-terminal -- ./monitor.sh 21; sleep 10; gnome-terminal -- ./rosbag.sh; sleep 10)" 
       
        
        self.exp_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                            shell=True, preexec_fn=os.setsid)

    def launch_mission(self, context: RobotRunnerContext):
        time.sleep(360)

    def stop_run(self, context: RobotRunnerContext) -> None:
        """Perform any activity required for stopping the run here.
        Activities before and after stopping the run should also be performed here."""
        
        os.killpg(os.getpgid(self.exp_proc.pid), signal.SIGINT)
        




    # ===============================================DO NOT ALTER BELOW THIS LINE=================================================
    # NOTE: Do not alter these values
    experiment_path:            Path             = None