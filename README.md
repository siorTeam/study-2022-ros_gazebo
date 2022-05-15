2022 ROS_GAZEBO
===============

# Creating a ROS Package 

## 1) Catkin package란?

       catkin package의 조건 : catkin compliant package.xml 파일, CMakeLists.txt를 포함하고 있어야 하며 같은 폴더를 공유하는 복수의 패키지를 혀용하지 않는다.

## 2) Catkin에서 작업공간 만들기

       $ source /opt/ros/groovy/setup.sh

       $ mkdir -p ~/catkin_ws/src
       $ cd ~/catkin_ws/src
       $ catkin_init_workspace

       $ cd ~/catkin_ws/
       $ catkin_make

       $ source devel/setup.bash
       

## 3) Catkin package 작성하기
       $ cd ~/catkin_ws/src

       # catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
       $ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp


## 4.1) Publisher node 작성 (Python)
       #beginner_tutorials package로 이동
       $ roscd beginner_tutorials

       $ mkdir scripts
       $ cd scripts
       
       $ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py
       $ chmod +x talker.py

       # script/talker.py 생성 후 아래 코드 복붙
       1 #!/usr/bin/env python
       2 # license removed for brevity
       3 import rospy
       4 from std_msgs.msg import String
       5 
       6 def talker():
       7     pub = rospy.Publisher('chatter', String, queue_size=10)
       8     rospy.init_node('talker', anonymous=True)
       9     rate = rospy.Rate(10) # 10hz
       10     while not rospy.is_shutdown():
       11         hello_str = "hello world %s" % rospy.get_time()
       12         rospy.loginfo(hello_str)
       13         pub.publish(hello_str)
       14         rate.sleep()
       15 
       16 if __name__ == '__main__':
       17     try:
       18         talker()
       19     except rospy.ROSInterruptException:
       20         pass


## 4.2) Publisher node 작성 (C++)

       #src/talker.cpp파일을 생성한 후 아래 내용을 복붙

       #include "ros/ros.h" /*ROS시스템에서 사용되는 헤더파일*/
#include "std_msgs/String.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker"); /*ROS 초기화, 노드의 이름을 갖게 되고 유일 지정*/

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n; /* 이 노드의 핸들러를 만듬*/

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	/*Topic "chatter"에 대한 std_msgs/String타입의 메세지를 publish할 것을 Master에게 알리게 된다.
	1000은 publish queue의 크기를 지정, publishing이 빠르게 일어나면 이전의 메세지를 버리기 전에
	버퍼 1000을 새롭게 채우는 문제가 발생할 수 있다.*/
	/*chatter_pun라는 이름의 ros::Publisher 객체를 생성하게 되고 advertise()는 해당 토픽으로 
	publish가능한 객체인 ros::Publisher클래스를 반환하며 그 객체의publish()를 이용하여 원하는 메세지를
	발행할 수 있다.*/
  ros::Rate loop_rate(10);
	/*ros::Rate 반복하고자하는 주기를 설정하게 된다.*/
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
	/*ros::ok() -> flase인 경우
		* Ctrl+C 의 입력을 받았을 경우
	  * 동일한 이름의 다른 노드로 인해 충돌이 발생한 경우
	  * 다른 부분에서 ros::shutdown() 이 호출된 경우
	  * 모든 ros::NodeHandles 가 종료된 경우
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
		/*msg파일을 통해 실행된 message-adapted class를 통해 ROS에서 메세지를 broadcasting함.*/
    ROS_INFO("%s", msg.data.c_str()); /*메세지를 출력하는 부분*/

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

## 5.1) Subscriber node 작성 (Python)
       $ roscd beginner_tutorials/scripts/
       $ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py   
       $ chmod +x listener.py

       # script/listener.py 생성 후 아래 코드 복붙
       1 #!/usr/bin/env python
       2 import rospy
       3 from std_msgs.msg import String
       4 
       5 def callback(data):
       6     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
       7     
       8 def listener():
       9 
       10     # In ROS, nodes are uniquely named. If two nodes with the same
       11     # name are launched, the previous one is kicked off. The
       12     # anonymous=True flag means that rospy will choose a unique
       13     # name for our 'listener' node so that multiple listeners can
       14     # run simultaneously.
       15     rospy.init_node('listener', anonymous=True)
       16 
       17     rospy.Subscriber("chatter", String, callback)
       18 
       19     # spin() simply keeps python from exiting until this node is stopped
       20     rospy.spin()
       21 
       22 if __name__ == '__main__':
       23     listener()
## 5.1) Subscriber node 작성 (C++))
       #src/listener.cpp
       #include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
/*chatter Topic에 대해 새로운 메시지를 수신하게 되면 호출되는 콜백 함수*/
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	/*
	chatter 토픽에 대한 메시지를 subscribe한다. ROS는 새로운 메시지가 도착할때마다 
	chatterCallback() 함수를 호출한다. 
	2번째 argument는 queue 크기로 queue에 1000 메시지가 가득 차게 되면 오래된 것부터 제거하게 된다.
	NodeHandle::subscribe() 함수는 ros::Subscriber 객체를 반환하고 
	이는 토픽을 unsubscribe할때까지 유지되게 된다.
	subscribe() 함수는 메시지를 받아 callback함수에 전달하게 되고 
	즉 전달받은 메시지를 통해 callback함수가 실행되어 처리되게 된다.
	*/
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
	/*반복적인 subscribe를 수행하고 callback을 지속적으로 요청한다*/
  return 0;
}

## 6) Node 빌드하기
       $ cd ~/catkin_ws
       $ catkin_make

## 7) Running the Publisher
       $ roscore

       # In your catkin workspace
       $ cd ~/catkin_ws
       $ source ./devel/setup.bash

       $ rosrun beginner_tutorials talker.py



       #결과 

       [INFO] [WallTime: 1314931831.774057] hello world 1314931831.77  
       [INFO] [WallTime: 1314931832.775497] hello world 1314931832.77  
       [INFO] [WallTime: 1314931833.778937] hello world 1314931833.78  
       [INFO] [WallTime: 1314931834.782059] hello world 1314931834.78  
       [INFO] [WallTime: 1314931835.784853] hello world 1314931835.78  
       [INFO] [WallTime: 1314931836.788106] hello world 1314931836.79  

## 8) Running the Subscriber
       $ rosrun beginner_tutorials listener.py

       #결과
       [INFO] [WallTime: 1314931969.258941] /listener_17657_1314931968795I heard hello world 1314931969.26  
       [INFO] [WallTime: 1314931970.262246] /listener_17657_1314931968795I heard hello world 1314931970.26  
       [INFO] [WallTime: 1314931971.266348] /listener_17657_1314931968795I heard hello world 1314931971.26  
       [INFO] [WallTime: 1314931972.270429] /listener_17657_1314931968795I heard hello world 1314931972.27  
       [INFO] [WallTime: 1314931973.274382] /listener_17657_1314931968795I heard hello world 1314931973.27  
       [INFO] [WallTime: 1314931974.277694] /listener_17657_1314931968795I heard hello world 1314931974.28  
       [INFO] [WallTime: 1314931975.283708] /listener_17657_1314931968795I heard hello world 1314931975.28  

