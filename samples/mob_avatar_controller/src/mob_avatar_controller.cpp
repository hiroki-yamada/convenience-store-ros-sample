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

class SIGVerseMobAvatarController
{
private:
  static const char KEYCODE_C = 0x63;
  static const char KEYCODE_D = 0x64;
  static const char KEYCODE_E = 0x65;
  static const char KEYCODE_F = 0x66;

public:
  static void rosSigintHandler(int sig);
  static int  canReceive(int fd);
  
  void createAvatar(const std::string &name);
  void deleteAvatar(const std::string &name);
  void receiveMessageCallback(const std_msgs::String::ConstPtr& message);
  void sendPositonCallback(const std::string &name, const double start_time);

  void showHelp();
  int run(int argc, char **argv);

private:
  ros::NodeHandle node_handle_;

  ros::Subscriber sub_msg_;
  ros::Publisher  pub_msg_;
  ros::Publisher  pub_transform_;
  
  std::map<std::string, ros::Timer> pub_timer_map_;
};

void SIGVerseMobAvatarController::rosSigintHandler(int sig)
{
  ros::shutdown();
}


int SIGVerseMobAvatarController::canReceive( int fd )
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

int SIGVerseMobAvatarController::run(int argc, char **argv)
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

  pub_msg_ = node_handle_.advertise<std_msgs::String>("/mob/message/from_ros", 10);
  sub_msg_ = node_handle_.subscribe<std_msgs::String>("/mob/message/from_sigverse", 100, &SIGVerseMobAvatarController::receiveMessageCallback, this);

  pub_transform_ = node_handle_.advertise<geometry_msgs::TransformStamped>("/mob/avatar_transform", 10);
  
  showHelp();

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
        case KEYCODE_C:
        {
          ROS_DEBUG("Create Avatar01");
          createAvatar("HumanAvatar01"); // example avatar name
          break;
        }
        case KEYCODE_D:
        {
          ROS_DEBUG("Delete Avatar01");
          deleteAvatar("HumanAvatar01"); // example avatar name
          break;
        }
        case KEYCODE_E:
        {
          ROS_DEBUG("Create Avatar02");
          createAvatar("HumanAvatar02"); // example avatar name
          break;
        }
        case KEYCODE_F:
        {
          ROS_DEBUG("Delete Avatar02");
          deleteAvatar("HumanAvatar02"); // example avatar name
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

void SIGVerseMobAvatarController::showHelp()
{
  puts("---------------------------");
  puts("-- Mob Avatar Controller --");
  puts("---------------------------");
  puts("c : Create Avatar01");
  puts("d : Delete Avatar01");
  puts("e : Create Avatar02");
  puts("f : Delete Avatar02");
  puts("---------------------------");
}

void SIGVerseMobAvatarController::createAvatar(const std::string &name)
{
  std_msgs::String msg;
  msg.data = "mob_create,"+name+",Mob_Customer_ManB";
  ROS_INFO("Create Avatar Name=%s", name.c_str());
  ROS_INFO("Sent Message: %s", msg.data.c_str());
  pub_msg_.publish(msg);
}

void SIGVerseMobAvatarController::deleteAvatar(const std::string &name)
{
  std_msgs::String msg;
  msg.data = "mob_delete,"+name;
  ROS_INFO("Delete Avatar Name=%s", name.c_str());
  ROS_INFO("Sent Message: %s", msg.data.c_str());
  pub_msg_.publish(msg);
}

void SIGVerseMobAvatarController::receiveMessageCallback(const std_msgs::String::ConstPtr& message)
{
  ROS_INFO("Subscribe Message: %s", message->data.c_str());
 
  std::vector<std::string> message_array;
 
  boost::split(message_array, message->data, boost::is_any_of(","));
  
  if(message_array[0]=="info")
  {
    if(message_array[1]=="mob_create")
    {
      std::string avatar_name = message_array[2];
      pub_timer_map_[avatar_name] = node_handle_.createTimer(ros::Duration(0.05), boost::bind(&SIGVerseMobAvatarController::sendPositonCallback, this, avatar_name, ros::Time::now().toSec()));
    }
    if(message_array[1]=="mob_delete")
    {
      std::string avatar_name = message_array[2];
      pub_timer_map_[avatar_name].stop();
    }
  }
  else
  {
	ROS_WARN("Warn: %s", message->data.c_str());
  }
}


void SIGVerseMobAvatarController::sendPositonCallback(const std::string &name, const double start_time)
{
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = name;   // Please set the avatar name

  float speed = 0.5;
  
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
  ros::init(argc, argv, "mob_avatar_controller");
  SIGVerseMobAvatarController mob_avatar_controller;
  return mob_avatar_controller.run(argc, argv);
}
  


