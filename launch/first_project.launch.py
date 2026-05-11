from launch import LaunchDescription # type: ignore
from launch_ros.actions import Node # type: ignore
from launch.actions import ExecuteProcess # type: ignore
import os
#as req. not using absolute path so it works in all computers thanks to below import :
from ament_index_python.packages import get_package_share_directory # type: ignore

def generate_launch_description():

    # Get path to rviz config file
    # PDF: "loading a configuration file which shows the two tfs in top view"
    rviz_config = os.path.join(
        get_package_share_directory('first_project'), #returns the path where first_project package is installed
        'config',
        'first_project.rviz' #and adding config and first_project.rviz to that path thanks to os.path.join
        #as req. "The launch file should also open rviz loading a configuration file"
    )

    return LaunchDescription([

        # Start odometer node
        # PDF: "write a node called odometer"
        Node(
            package='first_project', #from which package its gonna be executed
            executable='odometer', #as created in CMakeLists.txt add_executable(odometer ...)
            name='odometer', #we see odometer when ran ros2 node list
            output='screen', #write to terminal 
            parameters=[{'use_sim_time': True}]
            
        ),

        # Start tf_error node
        # this node is dependent on odometer since  we take diff between base_link from bag and base_link2 from our odometer
        # might get a warning if odometer still didnt publish TF
        Node(
            package='first_project',
            executable='tf_error',
            name='tf_error',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Start RViz with config file
        # ROS2's visualization tool directly not a package
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config], # -d means display config so it opens with config file
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
