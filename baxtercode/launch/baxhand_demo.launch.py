# """Launch HW6 P4

#    ros2 launch baxter_description baxhand_demo.launch.py

# This should start
#   1) RVIZ, ready to view the robot
#   2) The robot_state_publisher to broadcast the robot model
#   3) The baxhnad_demo code to move the joints

# """

# import os
# import xacro

# from ament_index_python.packages import get_package_share_directory as pkgdir

# from launch                            import LaunchDescription
# from launch.actions                    import DeclareLaunchArgument
# from launch.actions                    import OpaqueFunction
# from launch.actions                    import Shutdown
# from launch.substitutions              import LaunchConfiguration
# from launch_ros.actions                import Node


# #
# # Generate the Launch Description
# #
# def generate_launch_description():

#     ######################################################################
#     # LOCATE FILES

#     # # Locate the RVIZ configuration file.
#     # rvizcfgbax = os.path.join(pkgdir('baxtercode'), 'rviz/viewurdf.rviz')

#     # # Locate the URDF file.
#     # urdf = os.path.join(pkgdir('baxtercode'), 'urdf/baxter.urdf')

#     # # Load the robot's URDF file (XML).
#     # with open(urdf, 'r') as file:
#     #     robot_description = file.read()


#     # ######################################################################
#     # # PREPARE THE LAUNCH ELEMENTS

#     # # Configure a node for the robot_state_publisher.
#     # node_robot_state_publisher = Node(
#     #     name       = 'robot_state_publisher', 
#     #     package    = 'robot_state_publisher',
#     #     executable = 'robot_state_publisher',
#     #     output     = 'screen',
#     #     parameters = [{'robot_description': robot_description}])

#     # # Configure a node for RVIZ
#     # node_rviz_bax = Node(
#     #     name       = 'rviz', 
#     #     package    = 'rviz2',
#     #     executable = 'rviz2',
#     #     output     = 'screen',
#     #     arguments  = ['-d', rvizcfgbax],
#     #     on_exit    = Shutdown())
    
#     # # Configure a node for the joint trajectory
#     # node_trajectory_bax = Node(
#     #     name       = 'trajectory', 
#     #     package    = 'baxtercode',
#     #     executable = 'baxhand_demo',
#     #     output     = 'screen')
    



#     # Locate the RVIZ configuration file.
#     rvizcfgball = os.path.join(pkgdir('baxtercode'), 'rviz/viewmarkers.rviz')


#     ######################################################################
#     # PREPARE THE LAUNCH ELEMENTS

#     # Configure a node for the point_publisher.
#     node_ball = Node(
#         name       = 'balldemo',
#         package    = 'baxtercode',
#         executable = 'baxhand_demo',
#         output     = 'screen',
#         on_exit    = Shutdown())

#     # Configure a node for RVIZ
#     node_rviz_ball = Node(
#         name       = 'rviz', 
#         package    = 'rviz2',
#         executable = 'rviz2',
#         output     = 'screen',
#         arguments  = ['-d', rvizcfgball],
#         on_exit    = Shutdown())


#     ######################################################################
#     # RETURN THE ELEMENTS IN ONE LIST
#     return LaunchDescription([
#         # # Start the robot_state_publisher, RVIZ, and the trajectory.
#         # node_robot_state_publisher,
#         # node_rviz_bax,
#         # node_trajectory_bax,
#         node_ball,
#         node_rviz_ball,
#     ])


"""Launch the ball demo

   ros2 launch demos balldemo.launch.py

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
        executable = 'baxhand_demo',
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
        executable = 'baxhand_demo',
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
