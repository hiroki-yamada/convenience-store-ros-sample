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
public:
  static void rosSigintHandler(int sig);
  static int  canReceive(int fd);
  
  void sendPositonCallback(const std::string &name, const double start_time);

  void showHelp();
  int run(int argc, char **argv);

private:
  ros::NodeHandle node_handle_;

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

  pub_transform_ = node_handle_.advertise<geometry_msgs::TransformStamped>("/goods/transform", 10);
  
  showHelp();

  std::string goods_name = "cupnoodle_curry";
  pub_timer_map_[goods_name] = node_handle_.createTimer(ros::Duration(0.05), boost::bind(&SIGVerseCupNoodleController::sendPositonCallback, this, goods_name, ros::Time::now().toSec()));

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
          
//      switch(c)
//      {
//        case KEYCODE_C:
//        {
//          break;
//        }
//      }
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
}

void SIGVerseCupNoodleController::sendPositonCallback(const std::string &name, const double start_time)
{
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = name;   // Please set the name

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
  ros::init(argc, argv, "cupnoodle_controller");
  SIGVerseCupNoodleController cupnoodle_controller;
  return cupnoodle_controller.run(argc, argv);
}
  


