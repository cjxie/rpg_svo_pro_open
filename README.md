# rpg_svo_pro

# How to run
1. Source the workspace
   
    source /svo_ws/devel/setup.bash

2. Run the vo 
    1. Open a terminal and start the launch file wit specified argument 'cam_name', which automatically pass the camera calib result and svo parameter file in the same name to the node.(Need to tune the svo parameter for different scenarios)

    roslaunch svo_ros run_from_bag cam_name:=omni_21

    2. Open a new termial and run the bag.

    rosbag play <bag>
    
     