import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. kinematics.yaml 로드
        DeclareLaunchArgument(
            'kinematics_config',
            default_value='$(find bumblebee_moveit_config)/config/kinematics.yaml',
            description='load kinematics'
        ),
        
        # 2. MoveIt 2 노드 실행 (C++ 노드)
        Node(
            package='bumblebee_moveit_config',
            executable='hello_moveit',
            name='hello_moveit',
            output='screen',
            parameters=[{
                'kinematics_config': '$(arg kinematics_config)'  # kinematics.yaml 경로 설정
            }],
          #  remappings=[]  # 필요한 경우 topic remapping
        ),
        
        # 3. 다른 노드들이 필요한 경우 여기에 추가 가능
        LogInfo(
            msg="MoveIt 2 Node and kinematics.yaml have been loaded."
        )
    ])
