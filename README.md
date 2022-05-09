3rd Week : Gazebo_ROS_Integration : One Joint
=============================================

# Description

가제보와 로스를 연동한 튜토리얼

Link: [가제보 로스 연동 기본 세팅법][link]

[link]:[https://classic.gazebosim.org/tutorials?tut=ros_overview#Upgradingfromsimulator_gazebo(ROSgroovyandearlier)]

# Dependency

       sudo apt-get install ros-melodic-joint-state-controller 

       sudo apt-get install ros-melodic-effort-controllers

       sudo apt-get install ros-melodic-velocity-controllers

       sudo apt-get install ros-melodic-velocity-controllers

# Run

1) Download the "simple_example_description" directory and put it into your Desktop/workspace/src/ directory.

2) Build the project: 
       $ cd Desktop/workspace
       $ catkin_make
       $ source devel/setup.bash

3) Run the project:
       $ roslaunch simple_example_description spawn_robot.launch

4) to give a command to get the joint to move:
       $ rostopic pub /simple_model/base_to_second_joint_position_controller/command std_msgs/Float64 "data: 0.4"
       (속도조절)


# Package Description

## 1) urdf

기존에 만든 로봇의 urdf를 추가한다. 그 후에 뒤쪽 부분에 gazebo 플러그인을 추가한다.
```
 <!--                GAZEBO RELATED PART                             -->

  <!-- ROS Control plugin for Gazebo. This allows Gazebo and Ros to to be connected,
       and for ros to control the model in the gazebo visualization -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
	  <!--this is the namespace in the yaml file-->
      <robotNamespace>/simple_model</robotNamespace>
      <!--The two lines below came from: https://answers.ros.org/question/292444/gazebo_ros_control-plugin-gazeboroscontrolplugin-missing-legacymodens-defaultrobothwsim/-->
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
      <legacyModeNS>true</legacyModeNS>
    </plugin>
  </gazebo>


  <!-- transmission is the ability to define the ros controller used to control the joint. -->
  <transmission name="base_to_second_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="motor1">
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
    <joint name="base_to_second_joint">
      <!--lets use a postion type interface, which is a mechanism for ros_control
          to communicate with hardware. See: https://wiki.ros.org/ros_control-->
      <!--hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface-->
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
  </transmission>
```

첫번째는 가제보 플러그인, 두번째는 조인트를 제어하기 위해서 컨트롤러를 정의한다.


## 2) Config.yaml

```
#simple_model is the name space
simple_model:
    # based on https://wiki.ros.org/ros_control this controller reads the state 
    # of all the joints
    joint_state_controller:
        type: joint_state_controller/JointStateController
        publish_rate: 20

    base_to_second_joint_position_controller:
        #this controller recieves position and via PID it takes care of the effort
        # needed to move the joint. based on https://wiki.ros.org/ros_control
        # type: position_controllers/JointPositionController
        #type: velocity_controllers/JointVelocityController
        type: velocity_controllers/JointVelocityController
        joint: base_to_second_joint
        pid: {p: 1.0, i: 0.0, d: 0.1}
```

'#'는 주석이므로 무시하자. 

publish할 토픽에 관한 parameter들을 정의해주는 파일 같다. 

첫번째는 joint_state_controller의 메세지 타입, publish rate 를 정의했고,

두번째는 base_to_second_joint_position_controller에 대한 parameter들을 정의했다. 

## 3) spawn_robot.launch


```
<?xml version="1.0" encoding="UTF-8"?>
<launch>

    <!-- add the Gazebo world so that we launch it all together-->
    <arg name="model" default="$(find simple_example_description)/urdf/robot.urdf"/>
    <arg name="world" default="empty"/> 
    <arg name="paused" default="false"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>
    <arg name="verbose" value="true"/>

    <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find simple_example_description)/worlds/simple.world"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="headless" value="$(arg headless)"/>
    <arg name="debug" value="$(arg debug)"/>
    <!--by activating "verbose" mode, Gazebo prints more to terminal on launch.-->
    <arg name="verbose" value="$(arg verbose)"/>
    </include>

    <!--load urdf into parameter server-->
    <param name="robot_description" textfile="$(find simple_example_description)/urdf/robot.urdf" />

	
    <node name="urdf_spawner" 
          pkg="gazebo_ros" 
          type="spawn_model" 
          respawn="false" 
          output="screen" 
          args="-urdf -model simple_model -param robot_description"/>

    <!-- load the controllers -->
    <rosparam file="$(find simple_example_description)/config/config.yaml" command="load"/>

    <node name="controller_spawner" 
          pkg ="controller_manager" 
          type="spawner" 
          ns="/simple_model" 
          args="base_to_second_joint_position_controller joint_state_controller"/>
    
	<!-- converts joint states to TF transforms -->
    <node name="robot_state_publisher" 
          pkg="robot_state_publisher" 
          type="robot_state_publisher" 
          respawn="false" 
          output="screen">
        <remap from="joint_states" to="/simple_model/joint_states" />
    </node>
</launch>
```

해당 파일은 launch파일로써 여러 노드들을 동시에 실행함과 동시에 여러 파라미터들을 설정하면서 실행이 가능하다. 

앞쪽의 arg name들은 gazebo를 실행하기 위해 여러 파라미터들을 세팅하는 것이다. 파라미터 이름을 잘 읽어보면 이해 가능.

       <param name="robot_description" textfile="$(find simple_example_description)/urdf/robot.urdf" />\

이것은 가제보에 해당 로봇 urdf를 실행시키는 것이다. 이전 가제보 튜토리얼할때 진행했었음.

그 아래는 노드 3개, urdf_spawner, controller_spawner, robot_state_publisher를 실행한다. 

## 4) hello.cpp


그냥 로스를 돌리기 위한 기본 코드. 

## 5) simple.world

가제보 월드에 대한 정보가 담긴 파일이다. 빈 공간이 담겨있는 파일이고, 자세하게 알고 싶다면 가제보 공홈 튜토리얼 참고. 

# Original
This package is built based on a ros answers question and answer (see: https://answers.ros.org/question/273947/moving-joints-in-gazebo-simple-example/). Since the code in the answer does not build I put together this package to allow others to have the ability build and run the code.

*NOTE* this is an example of a JointPositionController. Fun fact: this controller, simply 'snaps' the joint into that position. It does not 'move' it, so there is no consideration of speed and direction. Read on to use other controllers.

This package works on the following system: Ubuntu 16.04 LTS + Gazebo7 version 7.16 + Ros Kinetic

to run the project follow these steps:
1) Download the "simple_example_description" directory and put it into your Desktop/workspace/src/ directory.
2) Build the project: 
       $ cd Desktop/workspace
       $ catkin_make
       $ source devel/setup.bash
3) Run the project:
       $ roslaunch simple_example_description spawn_robot.launch
4) to give a command to get the joint to move:
       $ rostopic pub /simple_model/base_to_second_joint_position_controller/command std_msgs/Float64 "data: 0.4"

The page https://wiki.ros.org/ros_control lists several controllers. To try different controllers simply replace one line in the urdf file (the line with the <hardwareInterface> tag), and one line in the yaml file (the second line starting with "type:.."). I have tried the 4 controller combinations listed below:

----------------------------------Setup #1--------------------------------------

Urdf file: <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>

yaml file: type: effort_controllers/JointPositionController

Result – ugly back and forth oscillation.

------------------------------------Setup #2-----------------------------------

Urdf file: <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>

yaml file: type: effort_controllers/JointEffortController

Result – spins one way, stops, spins the other way, and then rotates very, very fast.

-------------------------------------Setup #3--------------------------------------

Urdf file: <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>

yaml file: type: position_controllers/JointPositionController

Result – snaps into position

----------------------------------Setup #4----------------------------------------------

Urdf file: <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>

yaml file: type: velocity_controllers/JointVelocityController

Result – controlled rotation at the given velocity.