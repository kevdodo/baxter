"""Launch the baxter hand ball juggling

   ros2 launch demos baxhand_juggle.launch.py

   This is only intended to demonstrate the example.  Please
   edit/update as appropriate.

   This should start
     1) RVIZ, configured to see the visualization markers
     2) The ball demo

"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                      import LaunchDescription
from launch.actions              import Shutdown
from launch_ros.actions          import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES



    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS
    rvizcfgbax = os.path.join(pkgdir('baxtercode'), 'rviz/viewbaxter.rviz')

    # Configure a node for the point_publisher.
    node_ball = Node(
        name       = 'balldemo',
        package    = 'baxtercode',
        executable = 'baxhand_juggle',
        output     = 'screen',
        on_exit    = Shutdown())


    # Locate the URDF file.
    urdf = os.path.join(pkgdir('baxtercode'), 'urdf/baxter.urdf')

    # Load the robot's URDF file (XML).
    with open(urdf, 'r') as file:
        robot_description = file.read()

    node_robot_state_publisher = Node(
        name       = 'robot_state_publisher', 
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}])

    # Configure a node for RVIZ
    node_rviz_bax = Node(
        name       = 'rviz', 
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfgbax],
        on_exit    = Shutdown())
    
    # Configure a node for the joint trajectory
    node_trajectory_bax = Node(
        name       = 'trajectory', 
        package    = 'baxtercode',
        executable = 'baxhand_juggle',
        output     = 'screen')


    ######################################################################
    # RETURN THE ELEMENTS IN ONE LIST

    return LaunchDescription([

        # Start the demo and RVIZ
        node_robot_state_publisher,
        node_rviz_bax,
        node_trajectory_bax,
        node_ball,
    ])