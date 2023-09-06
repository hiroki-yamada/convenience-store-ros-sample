#include <stdio.h>
#include <cmath>
#include <string>
#include <list>
#include <map>
#include <signal.h>
#include <termios.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <boost/algorithm/string.hpp>

class SIGVerseCupNoodleController
{
private:
  static const char KEYCODE_1 = 0x31;
  static const char KEYCODE_2 = 0x32;
  static const char KEYCODE_3 = 0x33;
  static const char KEYCODE_G = 0x67;
  static const char KEYCODE_H = 0x68;
  static const char KEYCODE_I = 0x69;
  static const char KEYCODE_J = 0x6a;
  static const char KEYCODE_R = 0x72;
  
public:
  static void rosSigintHandler(int sig);
  static int  canReceive(int fd);
  
  void sendGraspedMessage(const std::string &name);
  void sendReleasedMessage();
  void createTimer(const std::string &name, const float speed);
  void sendPositon(const std::string &name, const float posx, const float posy);
  void sendPositonCallback(const std::string &name, const double start_time, const float speed);

  void showHelp();
  int run(int argc, char **argv);

private:
  ros::NodeHandle node_handle_;

  ros::Publisher  pub_msg_;
  ros::Publisher  pub_transform_;
  
  std::map<std::string, ros::Timer> pub_timer_map_;
};

void SIGVerseCupNoodleController::rosSigintHandler(int sig)
{
  ros::shutdown();
}


int SIGVerseCupNoodleController::canReceive( int fd )
{
  fd_set fdset;
  int ret;
  struct timeval timeout;
  FD_ZERO( &fdset );
  FD_SET( fd , &fdset );

  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  return select( fd+1 , &fdset , NULL , NULL , &timeout );
}

int SIGVerseCupNoodleController::run(int argc, char **argv)
{
  char c;
  int  ret;
  char buf[1024];

  /////////////////////////////////////////////
  // get the console in raw mode
  int kfd = 0;
  struct termios cooked;

  struct termios raw;
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  /////////////////////////////////////////////

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  ros::Rate loop_rate(50);

  pub_msg_       = node_handle_.advertise<std_msgs::String>("/goods/message/from_ros", 10);
  pub_transform_ = node_handle_.advertise<geometry_msgs::TransformStamped>("/goods/transform", 10);
  
  showHelp();
  
  createTimer("cupnoodle",             0.5);
  createTimer("cupnoodle_chilitomato", 0.6);
  createTimer("cupnoodle_curry",       0.7);
  createTimer("cupnoodle_seafood",     0.8);

  while (ros::ok())
  {
    if(canReceive(kfd))
    {
      // get the next event from the keyboard
      if((ret = read(kfd, &buf, sizeof(buf))) < 0)
      {
        perror("read():");
        exit(EXIT_FAILURE);
      }

      c = buf[ret-1];
          
      switch(c)
      {
        case KEYCODE_1:
        {
          sendPositon("cupnoodle_table", 0.0, 0.0);
          break;
        }
        case KEYCODE_2:
        {
          sendPositon("cupnoodle_table", 0.5, 0.0);
          break;
        }
        case KEYCODE_3:
        {
          sendPositon("cupnoodle_table", 0.0, 0.5);
          break;
        }
        case KEYCODE_G:
        {
          sendGraspedMessage("cupnoodle");
          break;
        }
        case KEYCODE_H:
        {
          sendGraspedMessage("cupnoodle_chilitomato");
          break;
        }
        case KEYCODE_I:
        {
          sendGraspedMessage("cupnoodle_curry");
          break;
        }
        case KEYCODE_J:
        {
          sendGraspedMessage("cupnoodle_seafood");
          break;
        }
        case KEYCODE_R:
        {
          sendReleasedMessage();
          break;
        }
      }
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  /////////////////////////////////////////////
  // cooked mode
  tcsetattr(kfd, TCSANOW, &cooked);
  /////////////////////////////////////////////

  return EXIT_SUCCESS;
}

void SIGVerseCupNoodleController::showHelp()
{
  puts("---------------------------");
  puts("-- CupNoodle Controller --");
  puts("---------------------------");
  puts("1 : Move Table to Position1");
  puts("2 : Move Table to Position2");
  puts("3 : Move Table to Position3");
  puts("g : Send Grasped cupnoodle");
  puts("h : Send Grasped cupnoodle_chilitomato");
  puts("i : Send Grasped cupnoodle_curry");
  puts("j : Send Grasped cupnoodle_seafood");
  puts("r : Send Released");
  puts("---------------------------");
}


void SIGVerseCupNoodleController::sendGraspedMessage(const std::string &name)
{
  std_msgs::String msg;
  msg.data = "grasped,"+name;
  ROS_INFO("Sent Message: %s", msg.data.c_str());
  pub_msg_.publish(msg);
}

void SIGVerseCupNoodleController::sendReleasedMessage()
{
  std_msgs::String msg;
  msg.data = "released";
  ROS_INFO("Sent Message: %s", msg.data.c_str());
  pub_msg_.publish(msg);
}

void SIGVerseCupNoodleController::createTimer(const std::string &name, const float speed)
{
  pub_timer_map_[name]=node_handle_.createTimer(ros::Duration(0.05), boost::bind(&SIGVerseCupNoodleController::sendPositonCallback, this, name, ros::Time::now().toSec(), speed));
}


void SIGVerseCupNoodleController::sendPositon(const std::string &name, const float posx, const float posy)
{
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = name;   // Please set the name

  // test position
  geometry_msgs::Vector3 pos;
  pos.x = posx;
  pos.y = posy;
  pos.z = 0.0;
  
  // test rotation
  tf::Quaternion tfqua = tf::createQuaternionFromRPY(0, 0, 0);
  
  geometry_msgs::Quaternion qua;
  quaternionTFToMsg(tfqua, qua);
  
  transformStamped.transform.translation = pos;
//  transformStamped.transform.rotation    = qua; // If not set, it will be calculated automatically by SIGVerse.
  
  pub_transform_.publish(transformStamped);
}


void SIGVerseCupNoodleController::sendPositonCallback(const std::string &name, const double start_time, const float speed)
{
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = name;   // Please set the name

  double pos_val = start_time + speed * ros::Time::now().toSec();
  
  // test position
  geometry_msgs::Vector3 pos;
  pos.x = cos(pos_val);
  pos.y = sin(pos_val);
  pos.z = 0.0;
  
  // test rotation
  tf::Quaternion tfqua = tf::createQuaternionFromRPY(0, 0, pos_val);
  
  geometry_msgs::Quaternion qua;
  quaternionTFToMsg(tfqua, qua);
  
  transformStamped.transform.translation = pos;
//  transformStamped.transform.rotation    = qua; // If not set, it will be calculated automatically by SIGVerse.
  
  pub_transform_.publish(transformStamped);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cupnoodle_controller");
  SIGVerseCupNoodleController cupnoodle_controller;
  return cupnoodle_controller.run(argc, argv);
}
  


