// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
 
// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include <iostream>

// Built-in message type that will be used to publish data
#include "std_msgs/msg/string.hpp"

//dependencies for ROS
#include <serial/serial.h>
#include <signal.h>
#include <string>
#include <sstream>

#define DELTAT(_nowtime,_thentime) ((_thentime>_nowtime)?((0xffffffff-_thentime)+_nowtime):(_nowtime-_thentime))


//
// cmd_vel subscriber
//

// Define following to enable cmdvel debug output
#define _CMDVEL_DEBUG

// Define following to enable motor test mode
//  Runs both motors in forward direction at 10% of configured maximum (rpms in close_loop mode, power in open_loop mode)
//  If configured correctly robot should translate forward in a straight line
//#define _CMDVEL_FORCE_RUN

//#include <geometry_msgs/msg/Twist.hpp> //maybe?
//#include <nav_msgs/msg/Odometry.hpp> //Don't use these in ros2
//
// odom publisher
//

// Define following to enable odom debug output
#define _ODOM_DEBUG

// Define following to publish additional sensor information; comment out to not publish

//#define _ODOM_SENSORS

// Define following to enable service for returning covariance
//#define _ODOM_COVAR_SERVER

#define NORMALIZE(_z) atan2(sin(_z), cos(_z))

//#include <tf2_ros/tf2_ros.h> //check
//#include <geometry_msgs/msg/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
//#include <nav_msgs/msg/Odometry.h>
#ifdef _ODOM_SENSORS
#include <std_msgs/msg/Float32.h>
//#include <roboteq_diff_msgs/Duplex.h> // add custom msgs after
#endif
#ifdef _ODOM_COVAR_SERVER
#include "roboteq_diff_msgs/msg/OdometryCovariances.h"
#include "rogoteq_diff_msgs/msg/RequestOdometryCovariances.h"
#endif

void mySigintHandler(int sig)
{
  RCLCPP_INFO(get_logger(),"Received SIGINT signal, shutting down...");
  //ROS_INFO("Received SIGINT signal, shutting down...");
  rclcpp::shutdown();
}


uint32_t millis()
{
	//ros::WallTime walltime = ros::WallTime::now(); // ROS1
    //rclcpp::TimePoint now = rclcpp::Clock::now(); //original
    rclcpp::Time now = this->get_clock()->now();
//	return (uint32_t)((walltime._sec*1000 + walltime.nsec/1000000.0) + 0.5);
//	return (uint32_t)(walltime.toNSec()/1000000.0+0.5);
	//return (uint32_t)(now.toNSec()/1000000); //ROS1
    return static_cast<uint32_t>(now.time_since_epoch().count()/1000000);
}

class MainNode
{

public:
  MainNode();

public:

  //
  // cmd_vel subscriber
  //
  void cmdvel_callback( const geometry_msgs::Twist::SharedPtr twist_msg);
  void cmdvel_setup();
  void cmdvel_loop();
  void cmdvel_run();


  //
  // odom publisher
  //
  void odom_setup();
  void odom_stream(); 
  void odom_loop();
  //void odom_hs_run();
  void odom_ms_run();
  void odom_ls_run();
  void odom_publish();
  #ifdef _ODOM_COVAR_SERVER
  void odom_covar_callback(const roboteq_diff_msgs::srv::RequestOdometryCovariances::Request::SharedPtr req, roboteq_diff_msgs::srv::RequestOdometryCovariances::Response::SharedPtr res);
  #endif

  int run();

protected:
  rclcpp::Node::SharedPtr nh;

  serial::Serial controller;
  uint32_t starttime;
  uint32_t hstimer;
  uint32_t mstimer;
  uint32_t lstimer;

  //
  // cmd_vel subscriber
  //
  ros::Subscriber cmdvel_sub;

  //
  // odom publisher
  //
  rclcpp::Subscription<geometry_msgs::Twist>::SharedPtr cmdvel_sub;

  //
  // odom publisher
  //
  geometry_msgs::TransformStamped tf_msg;
  tf2_ros::TransformBroadcaster odom_broadcaster;
  nav_msgs::Odometry odom_msg;
  rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr odom_pub;
/* ROS1
#ifdef _ODOM_SENSORS
  std_msgs::Float32 voltage_msg;
  ros::Publisher voltage_pub;
  roboteq_diff_msgs::Duplex current_msg;
  ros::Publisher current_pub;
  std_msgs::Float32 energy_msg;
  ros::Publisher energy_pub;
  std_msgs::Float32 temperature_msg;
  ros::Publisher temperature_pub;
#endif
*/
/*
//ROS2: TODO: Add sensor msgs later
#ifdef _ODOM_SENSORS
//std_msgs::Float32 voltage_msg;
//rclcpp::Publisher<std_msgs::Float32>::SharedPtr voltage_pub;
//roboteq_diff_msgs::Duplex current_msg;
//rclcpp::Publisher<roboteq_diff_msgs::Duplex>::SharedPtr current_pub;
//std_msgs::Float32 energy_msg;
//rclcpp::Publisher<std_msgs::Float32>::SharedPtr energy_pub;
//std_msgs::Float32 temperature_msg;
//rclcpp::Publisher<std_msgs::Float32>::SharedPtr temperature_pub;
#endif
*/
  // buffer for reading encoder counts
  int odom_idx;
  char odom_buf[24];

  // toss out initial encoder readings
  char odom_encoder_toss;

  int32_t odom_encoder_left;
  int32_t odom_encoder_right;

  float odom_x;
  float odom_y;
  float odom_yaw;
  float odom_last_x;
  float odom_last_y;
  float odom_last_yaw;

  uint32_t odom_last_time;

#ifdef _ODOM_SENSORS
  float voltage;
  float current_right;
  float current_left;
  float energy;
  float temperature;
  uint32_t current_last_time;
#endif

  // settings
  bool pub_odom_tf;
  std::string odom_frame;
  std::string base_frame;
  std::string cmdvel_topic;
  std::string odom_topic;
  std::string port;
  int baud;
  bool open_loop;
  double wheel_circumference;
  double track_width;
  int encoder_ppr;
  int encoder_cpr;
  double max_amps;
  int max_rpm;

};

MainNode::MainNode() : 
    //initialization
  starttime(0),
  hstimer(0),
  mstimer(0),
  odom_idx(0),
  odom_encoder_toss(5),
  odom_encoder_left(0),
  odom_encoder_right(0),
  odom_x(0.0),
  odom_y(0.0),
  odom_yaw(0.0),
  odom_last_x(0.0),
  odom_last_y(0.0),
  odom_last_yaw(0.0),
  odom_last_time(0),
#ifdef _ODOM_SENSORS
  voltage(0.0),
  current_right(0.0),
  current_left(0.0),
  energy(0.0),
  temperature(0.0),
  current_last_time(0),
#endif
  pub_odom_tf(true),
  open_loop(false),
  baud(115200),
  wheel_circumference(0),
  track_width(0),
  encoder_ppr(0),
  encoder_cpr(0),
  max_amps(0.0),
  max_rpm(0)
{

  // ROS2:
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  auto nhLocal = std::make_shared<rclcpp::Node>("roboteq_diff_driver", node_options);
  //what is the point of that^? 

  nhLocal->get_parameter("pub_odom_tf", pub_odom_tf);
  RCLCPP_INFO(nhLocal->get_logger(), "pub_odom_tf: %d", pub_odom_tf);


  nhLocal->get_parameter("odom_frame", odom_frame);
  RCLCPP_INFO(nhLocal->get_logger(), "odom_frame: %s", odom_frame.c_str());

  std::string base_frame;
  nhLocal->get_parameter("base_frame", base_frame);
  RCLCPP_INFO(nhLocal->get_logger(), "base_frame: %s", base_frame.c_str());


  nhLocal->get_parameter("cmdvel_topic", cmdvel_topic);
  RCLCPP_INFO(nhLocal->get_logger(), "cmdvel_topic: %s", cmdvel_topic.c_str());


  nhLocal->get_parameter("odom_topic", odom_topic);
  RCLCPP_INFO(nhLocal->get_logger(), "odom_topic: %s", odom_topic.c_str());

  nhLocal->get_parameter("port", port);
  RCLCPP_INFO(nhLocal->get_logger(), "port: %s", port.c_str());


  nhLocal->get_parameter("baud", baud);
  RCLCPP_INFO(nhLocal->get_logger(), "baud: %d", baud);


  nhLocal->get_parameter("open_loop", open_loop);
  RCLCPP_INFO(nhLocal->get_logger(), "open_loop: %d", open_loop);


  nhLocal->get_parameter("wheel_circumference", wheel_circumference);
  RCLCPP_INFO(nhLocal->get_logger(), "wheel_circumference: %f", wheel_circumference);


  nhLocal->get_parameter("track_width", track_width);
  RCLCPP_INFO(nhLocal->get_logger(), "track_width: %f", track_width);

  nhLocal->get_parameter("encoder_ppr", encoder_ppr);
  RCLCPP_INFO(nhLocal->get_logger(), "encoder_ppr: %d", encoder_ppr);


  nhLocal->get_parameter("encoder_cpr", encoder_cpr);
  RCLCPP_INFO(nhLocal->get_logger(), "encoder_cpr: %d", encoder_cpr);

  nhLocal->get_parameter("max_amps", max_amps);
  RCLCPP_INFO(nhLocal->get_logger(), "max_amps: %f", max_amps);


  nhLocal->get_parameter("max_rpm", max_rpm);
  RCLCPP_INFO(nhLocal->get_logger(), "max_rpm: %d", max_rpm);

  /*ROS1:
  // CBA Read local params (from launch file)
  ros::NodeHandle nhLocal("~");
  nhLocal.param("pub_odom_tf", pub_odom_tf, true);
  ROS_INFO_STREAM("pub_odom_tf: " << pub_odom_tf);
  nhLocal.param<std::string>("odom_frame", odom_frame, "odom");
  ROS_INFO_STREAM("odom_frame: " << odom_frame);
  nhLocal.param<std::string>("base_frame", base_frame, "base_link");
  ROS_INFO_STREAM("base_frame: " << base_frame);
  nhLocal.param<std::string>("cmdvel_topic", cmdvel_topic, "cmd_vel");
  ROS_INFO_STREAM("cmdvel_topic: " << cmdvel_topic);
  nhLocal.param<std::string>("odom_topic", odom_topic, "odom");
  ROS_INFO_STREAM("odom_topic: " << odom_topic);
  nhLocal.param<std::string>("port", port, "/dev/ttyACM0");
  ROS_INFO_STREAM("port: " << port);
  nhLocal.param("baud", baud, 115200);
  ROS_INFO_STREAM("baud: " << baud);
  nhLocal.param("open_loop", open_loop, false);
  ROS_INFO_STREAM("open_loop: " << open_loop);
  nhLocal.param("wheel_circumference", wheel_circumference, 0.3192);
  ROS_INFO_STREAM("wheel_circumference: " << wheel_circumference);
  nhLocal.param("track_width", track_width, 0.4318);
  ROS_INFO_STREAM("track_width: " << track_width);
  nhLocal.param("encoder_ppr", encoder_ppr, 900);
  ROS_INFO_STREAM("encoder_ppr: " << encoder_ppr);
  nhLocal.param("encoder_cpr", encoder_cpr, 3600);
  ROS_INFO_STREAM("encoder_cpr: " << encoder_cpr);
  nhLocal.param("max_amps", max_amps, 5.0);
  ROS_INFO_STREAM("max_amps: " << max_amps);
  nhLocal.param("max_rpm", max_rpm, 100);
  ROS_INFO_STREAM("max_rpm: " << max_rpm);
  */

}

//
// cmd_vel subscriber
//

void MainNode::cmdvel_callback( const geometry_msgs::Twist::SharedPtr twist_msg) //const???
{
    // wheel speed (m/s)
  float right_speed = twist_msg.linear.x + track_width * twist_msg.angular.z / 2.0;
  float left_speed = twist_msg.linear.x - track_width * twist_msg.angular.z / 2.0;
#ifdef _CMDVEL_DEBUG
ROS_DEBUG_STREAM("cmdvel speed right: " << right_speed << " left: " << left_speed);
#endif

  std::stringstream right_cmd;
  std::stringstream left_cmd;

  if (open_loop)
  {
    // motor power (scale 0-1000)
    
    int32_t right_power = right_speed / wheel_circumference * 60.0 / max_rpm * 1000.0;
    int32_t left_power = left_speed / wheel_circumference * 60.0 / max_rpm * 1000.0;
    /*
    // set minimum to overcome friction if cmd_vel too low
    if (right_power < 10 && left_power > 0){
      right_power = 10
    }
    if (left_power < 10 && left_power > 0){
      left_power = 10
    }
    */

#ifdef _CMDVEL_DEBUG
RCLCPP_DEBUG_STREAM(get_logger(), "cmdvel speed right: " << right_speed << " left: " << left_speed);
#endif
    right_cmd << "!G 1 " << right_power << "\r";
    left_cmd << "!G 2 " << left_power << "\r";
  }
  else
  {
    // motor speed (rpm)
    int32_t right_rpm = right_speed / wheel_circumference * 60.0;
    int32_t left_rpm = left_speed / wheel_circumference * 60.0;
#ifdef _CMDVEL_DEBUG
RCLCPP_DEBUG_STREAM(get_logger(), "cmdvel speed right: " << right_speed << " left: " << left_speed);
#endif
    right_cmd << "!S 1 " << right_rpm << "\r";
    left_cmd << "!S 2 " << left_rpm << "\r";
  }


#ifndef _CMDVEL_FORCE_RUN
  controller.write(right_cmd.str());
  controller.write(left_cmd.str());
  controller.flush();
#endif
}


void MainNode::cmdvel_setup()
{

  // stop motors
  controller.write("!G 1 0\r");
  controller.write("!G 2 0\r");
  controller.write("!S 1 0\r");
  controller.write("!S 2 0\r");
  controller.flush();

  // disable echo
  controller.write("^ECHOF 1\r");
  controller.flush();

  // enable watchdog timer (1000 ms)
  controller.write("^RWD 1000\r");

  // set motor operating mode (1 for closed-loop speed)
  if (open_loop)
  {
    // open-loop speed mode
    controller.write("^MMOD 1 0\r");
    controller.write("^MMOD 2 0\r");
  }
  else
  {
    // closed-loop speed mode
    controller.write("^MMOD 1 1\r");
    controller.write("^MMOD 2 1\r");
  }

  // set motor amps limit (A * 10)
  std::stringstream right_ampcmd;
  std::stringstream left_ampcmd;
  right_ampcmd << "^ALIM 1 " << (int)(max_amps * 10) << "\r";
  left_ampcmd << "^ALIM 2 " << (int)(max_amps * 10) << "\r";
  controller.write(right_ampcmd.str());
  controller.write(left_ampcmd.str());

  // set max speed (rpm) for relative speed commands
  std::stringstream right_rpmcmd;
  std::stringstream left_rpmcmd;
  right_rpmcmd << "^MXRPM 1 " << max_rpm << "\r";
  left_rpmcmd << "^MXRPM 2 " << max_rpm << "\r";
  controller.write(right_rpmcmd.str());
  controller.write(left_rpmcmd.str());

  // set max acceleration rate (2000 rpm/s * 10)
  controller.write("^MAC 1 20000\r");
  controller.write("^MAC 2 20000\r");

  // set max deceleration rate (2000 rpm/s * 10)
  controller.write("^MDEC 1 20000\r");
  controller.write("^MDEC 2 20000\r");

  // set PID parameters (gain * 10)
  controller.write("^KP 1 10\r");
  controller.write("^KP 2 10\r");
  controller.write("^KI 1 80\r");
  controller.write("^KI 2 80\r");
  controller.write("^KD 1 0\r");
  controller.write("^KD 2 0\r");

  // set encoder mode (18 for feedback on motor1, 34 for feedback on motor2)
  controller.write("^EMOD 1 18\r");
  controller.write("^EMOD 2 34\r");

  // set encoder counts (ppr)
  std::stringstream right_enccmd;
  std::stringstream left_enccmd;
  right_enccmd << "^EPPR 1 " << encoder_ppr << "\r";
  left_enccmd << "^EPPR 2 " << encoder_ppr << "\r";
  controller.write(right_enccmd.str());
  controller.write(left_enccmd.str());

  controller.flush();

  RCLCPP_INFO_STREAM(rclcpp::get_logger("my_package"), "Subscribing to topic " << cmdvel_topic);

  // cmdvel_sub = nh.subscribe(cmdvel_topic, 1000, &MainNode::cmdvel_callback, this); //original
  // ths-> before create ???
  cmdvel_sub = this->create_subscription<geometry_msgs::Twist>(
      cmdvel_topic,  // topic name
      1000,       // QoS history depth
      std::bind(&MainNode::cmdvel_callback, this, std::placeholders::_1)); // check - might just need _1

}

void MainNode::cmdvel_loop()
{
}

void MainNode::cmdvel_run()
{
#ifdef _CMDVEL_FORCE_RUN
  if (open_loop)
  {
    controller.write("!G 1 100\r");
    controller.write("!G 2 100\r");
  }
  else
  {
    std::stringstream right_cmd;
    std::stringstream left_cmd;
    right_cmd << "!S 1 " << (int)(max_rpm * 0.1) << "\r";
    left_cmd << "!S 2 " << (int)(max_rpm * 0.1) << "\r";
    controller.write(right_cmd.str());
    controller.write(left_cmd.str());
  }
  controller.flush();
#endif
}



//
// odom publisher


// we really dont use the covar server
#ifdef _ODOM_COVAR_SERVER
void MainNode::odom_covar_callback(const roboteq_diff_msgs::RequestOdometryCovariancesRequest& req, roboteq_diff_msgs::RequestOdometryCovariancesResponse& res)
{
  res.odometry_covariances.pose.pose.covariance[0] = 0.001;
  res.odometry_covariances.pose.pose.covariance[7] = 0.001;
  res.odometry_covariances.pose.pose.covariance[14] = 1000000;
  res.odometry_covariances.pose.pose.covariance[21] = 1000000;
  res.odometry_covariances.pose.pose.covariance[28] = 1000000;
  res.odometry_covariances.pose.pose.covariance[35] = 1000;

  res.odometry_covariances.twist.twist.covariance[0] = 0.001;
  res.odometry_covariances.twist.twist.covariance[7] = 0.001;
  res.odometry_covariances.twist.twist.covariance[14] = 1000000;
  res.odometry_covariances.twist.twist.covariance[21] = 1000000;
  res.odometry_covariances.twist.twist.covariance[28] = 1000000;
  res.odometry_covariances.twist.twist.covariance[35] = 1000;
}
#endif


void MainNode::odom_setup()
{
  
  if ( pub_odom_tf )
  {
    RCLCPP_INFO(get_logger(), "Broadcasting odom tf"); // might use this-> instead of node
//    odom_broadcaster.init(nh);	// ???
  }

  //ROS_INFO_STREAM("Publishing to topic " << odom_topic);
  //maybe use this-> instead of
  RCLCPP_INFO_STREAM(get_logger(), "Publishing to topic " << odom_topic);

  //odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);
  odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1000)
//#ifdef _ODOM_COVAR_SERVER
//  ROS_INFO("Advertising service on roboteq/odom_covar_srv");
//  odom_covar_server = nh.advertiseService("roboteq/odom_covar_srv", &MainNode::odom_covar_callback, this);
//#endif
/*
#ifdef _ODOM_SENSORS
  ROS_INFO("Publishing to topic roboteq/voltage");
  voltage_pub = nh.advertise<std_msgs::Float32>("roboteq/voltage", 1000);
  ROS_INFO("Publishing to topic roboteq/current");
  current_pub = nh.advertise<roboteq_diff_msgs::Duplex>("roboteq/current", 1000);
  ROS_INFO("Publishing to topic roboteq/energy");
  energy_pub = nh.advertise<std_msgs::Float32>("roboteq/energy", 1000);
  ROS_INFO("Publishing to topic roboteq/temperature");
  temperature_pub = nh.advertise<std_msgs::Float32>("roboteq/temperature", 1000);
#endif
*/


  // Set up the header
  tf_msg.header.stamp = 0;
  tf_msg.header.frame_id = odom_frame;
  tf_msg.child_frame_id = base_frame;

  odom_msg.header.stamp = 0;
  odom_msg.header.frame_id = odom_frame;
  odom_msg.child_frame_id = base_frame;

  // Set up the pose covariance
  for (size_t i = 0; i < 36; i++) {
    odom_msg.pose.covariance[i] = 0;
    odom_msg.twist.covariance[i] = 0;
  }

  odom_msg.pose.covariance[0] = 0.001;
  odom_msg.pose.covariance[7] = 0.001;
  odom_msg.pose.covariance[14] = 1000000;
  odom_msg.pose.covariance[21] = 1000000;
  odom_msg.pose.covariance[28] = 1000000;
  odom_msg.pose.covariance[35] = 1000;

  // Set up the twist covariance
  odom_msg.twist.covariance[0] = 0.001;
  odom_msg.twist.covariance[7] = 0.001;
  odom_msg.twist.covariance[14] = 1000000;
  odom_msg.twist.covariance[21] = 1000000;
  odom_msg.twist.covariance[28] = 1000000;
  odom_msg.twist.covariance[35] = 1000;

  // Set up the transform message: move to odom_publish
  /*
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);

  tf_msg.transform.translation.x = x;
  tf_msg.transform.translation.y = y;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();
  */

#ifdef _ODOM_SENSORS
//  voltage_msg.header.seq = 0;
//  voltage_msg.header.frame_id = 0;
//  current_msg.header.seq = 0;
//  current_msg.header.frame_id = 0;
//  energy_msg.header.seq = 0;
//  energy_msg.header.frame_id = 0;
//  temperature_msg.header.seq = 0;
//  temperature_msg.header.frame_id = 0;
#endif

  // start encoder streaming
  odom_stream();

  odom_last_time = millis();
#ifdef _ODOM_SENSORS
  current_last_time = millis();
#endif
}

// Odom msg streams

void MainNode::odom_stream()
{
  
#ifdef _ODOM_SENSORS
  // start encoder and current output (30 hz)
  // doubling frequency since one value is output at each cycle
//  controller.write("# C_?CR_?BA_# 17\r");
  // start encoder, current and voltage output (30 hz)
  // tripling frequency since one value is output at each cycle
  controller.write("# C_?CR_?BA_?V_# 11\r");
#else
//  // start encoder output (10 hz)
//  controller.write("# C_?CR_# 100\r");
  // start encoder output (30 hz)
  controller.write("# C_?CR_# 33\r");

#endif
  controller.flush();

}


void MainNode::odom_loop()
{

  uint32_t nowtime = millis();

  // if we haven't received encoder counts in some time then restart streaming
  if( DELTAT(nowtime,odom_last_time) >= 1000 )
  {
    odom_stream();
    odom_last_time = nowtime;
  }

  // read sensor data stream from motor controller
  if (controller.available())
  {
    char ch = 0;
    if ( controller.read((uint8_t*)&ch, 1) == 0 )
      return;
    if (ch == '\r')
    {
      odom_buf[odom_idx] = 0;
#ifdef _ODOM_DEBUG
//ROS_DEBUG_STREAM( "line: " << odom_buf );
#endif
      // CR= is encoder counts
      if ( odom_buf[0] == 'C' && odom_buf[1] == 'R' && odom_buf[2] == '=' )
      {
        int delim;
        for ( delim = 3; delim < odom_idx; delim++ )
        {
          if ( odom_encoder_toss > 0 )
          {
            --odom_encoder_toss;
            break;
          }
          if (odom_buf[delim] == ':')
          {
            odom_buf[delim] = 0;
            odom_encoder_right = (int32_t)strtol(odom_buf+3, NULL, 10);
            odom_encoder_left = (int32_t)strtol(odom_buf+delim+1, NULL, 10);
#ifdef _ODOM_DEBUG
//ROS_DEBUG_STREAM("encoder right: " << odom_encoder_right << " left: " << odom_encoder_left);
#endif
            odom_publish();
            break;
          }
        }
      }

      odom_idx = 0;
    }
    else if ( odom_idx < (sizeof(odom_buf)-1) )
    {
      odom_buf[odom_idx++] = ch;
    }
  }
}

void MainNode::odom_publish()
{

  // determine delta time in seconds
  uint32_t nowtime = millis();
  float dt = (float)DELTAT(nowtime,odom_last_time) / 1000.0;
  odom_last_time = nowtime;



  // determine deltas of distance and angle
  float linear = ((float)odom_encoder_right / (float)encoder_cpr * wheel_circumference + (float)odom_encoder_left / (float)encoder_cpr * wheel_circumference) / 2.0;
//  float angular = ((float)odom_encoder_right / (float)encoder_cpr * wheel_circumference - (float)odom_encoder_left / (float)encoder_cpr * wheel_circumference) / track_width * -1.0;
  float angular = ((float)odom_encoder_right / (float)encoder_cpr * wheel_circumference - (float)odom_encoder_left / (float)encoder_cpr * wheel_circumference) / track_width;


  // Update odometry
  odom_x += linear * cos(odom_yaw);        // m
  odom_y += linear * sin(odom_yaw);        // m
  odom_yaw = NORMALIZE(odom_yaw + angular);  // rad


  // Calculate velocities
  float vx = (odom_x - odom_last_x) / dt;
  float vy = (odom_y - odom_last_y) / dt;
  float vyaw = (odom_yaw - odom_last_yaw) / dt;

  odom_last_x = odom_x;
  odom_last_y = odom_y;
  odom_last_yaw = odom_yaw;


  //geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(odom_yaw);
  geometry_msgs::msg::Quaternion quat = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), odom_yaw));
  if ( pub_odom_tf )
  {
    tf_msg.header.seq++;
    tf_msg.header.stamp = rclcpp::Clock::now();
    tf_msg.header.frame_id = odom_frame;
    tf_msg.child_frame_id = base_frame;

    tf_msg.transform.translation.x = odom_x;
    tf_msg.transform.translation.y = odom_y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = quat;
    odom_broadcaster.sendTransform(tf_msg);
  }

  odom_msg.header.seq++;
  odom_msg.header.stamp = rclcpp::Clock::now()
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = quat;
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = vyaw;
  odom_pub.publish(odom_msg);

}


int MainNode::run()
{

	RCLCPP_INFO(rclcpp::get_logger(),"Beginning setup...");


	serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
	controller.setPort(port);
	controller.setBaudrate(baud);
	controller.setTimeout(timeout);

	// TODO: support automatic re-opening of port after disconnection
	while ( rclcpp::ok() )
	{
		RCLCPP_INFO_STREAM(rclcpp::get_logger(),"Opening serial port on " << port << " at " << baud << "..." );
		try
		{
			controller.open();
			if ( controller.isOpen() )
			{
				RCLCPP_INFO(this->get_logger(),"Successfully opened serial port");
				break;
			}
		}
		catch (serial::IOException e)
		{
			RCLCPP_WARN_STREAM(rclcpp::get_logger(),"serial::IOException: " << e.what());
		}
		RCLCPP_WARN(rclcpp::get_logger(),"Failed to open serial port");
		sleep( 5 );
	}

	cmdvel_setup();
	odom_setup();

  starttime = millis();
  hstimer = starttime;
  mstimer = starttime;
  lstimer = starttime;

//  ros::Rate loop_rate(10);

  RCLCPP_INFO(rclcpp::get_logger(),"Beginning looping...");
	
  while (rclcpp::ok())
  {

    cmdvel_loop();
    odom_loop();

    uint32_t nowtime = millis();
//ROS_INFO_STREAM("loop nowtime: " << nowtime << " lstimer: " << lstimer << " delta: " << DELTAT(nowtime,lstimer) << " / " << (nowtime-lstimer));
//uint32_t delta = DELTAT(nowtime,lstimer);
//ROS_INFO_STREAM("loop nowtime: " << nowtime << " lstimer: " << lstimer << " delta: " << delta << " / " << (nowtime-lstimer));

//    // Handle 50 Hz publishing
//    if (DELTAT(nowtime,hstimer) >= 20)
    // Handle 30 Hz publishing
    if (DELTAT(nowtime,hstimer) >= 33)
    {
      hstimer = nowtime;
//      odom_hs_run();
    }

    // Handle 10 Hz publishing
    if (DELTAT(nowtime,mstimer) >= 100)
    {
      mstimer = nowtime;
      cmdvel_run();
      odom_ms_run();
    }

    // Handle 1 Hz publishing
    if (DELTAT(nowtime,lstimer) >= 1000)
    {
      lstimer = nowtime;
      odom_ls_run();
    }

    //ros::spinOnce();
    rclcpp::spin_some(node) // maybe put this for node??

//    loop_rate.sleep();
  }
	
  if ( controller.isOpen() )
    controller.close();

  RCLCPP_INFO(this->get_logger(),"Exiting");
	
  return 0;
}

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("roboteq_diff_driver");

  //MainNode node;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);

  return node.run();
}