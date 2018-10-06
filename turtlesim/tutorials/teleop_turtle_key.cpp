#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#ifndef _WINDOWS
#include <termios.h>
#else
#include <windows.h>
#endif
#include <stdio.h>

#ifdef _WINDOWS
#define KEYCODE_R VK_RIGHT 
#define KEYCODE_L VK_LEFT
#define KEYCODE_U VK_UP
#define KEYCODE_D VK_DOWN
#define KEYCODE_Q 0x51
#else
#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#endif

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

private:  
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;
  
};

TeleopTurtle::TeleopTurtle():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
}


#ifdef _WINDOWS
HANDLE kStdin = 0;
DWORD saveOldMode;
#else
int kfd = 0;
struct termios cooked, raw;
#endif

void quit(int sig)
{
  (void)sig;
#ifdef _WINDOWS
  SetConsoleMode(kStdin, saveOldMode);
#else
  tcsetattr(kfd, TCSANOW, &cooked);
#endif 
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

#ifndef _WINDOWS
  signal(SIGINT,quit);
#endif

  teleop_turtle.keyLoop();
  
  return(0);
}


void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty = false;
#ifdef _WINDOWS
  INPUT_RECORD inRecord[128]; 
  DWORD numRead = 0;
#endif

#ifdef _WINDOWS
  kStdin = GetStdHandle(STD_INPUT_HANDLE); 
  if (kStdin == INVALID_HANDLE_VALUE) exit(0);
  if (!GetConsoleMode(kStdin, &saveOldMode) ) exit(0);
  DWORD fdwMode = ENABLE_WINDOW_INPUT | ENABLE_PROCESSED_INPUT; 
  if (!SetConsoleMode(kStdin, fdwMode) ) exit(0);
#else
  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
#endif

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");

  for(;;)
  {
    // get the next event from the keyboard
#ifdef _WINDOWS
    if(!ReadConsoleInput(kStdin, inRecord, 128, &numRead))
	{
	  perror("ReadConsoleInput():");
	  exit(-1);
	}
	
	for (int i = 0; i < numRead; i++)
	{
	  if (inRecord[i].EventType == KEY_EVENT && 
	      inRecord[i].Event.KeyEvent.bKeyDown == false)
	  {
		  c = inRecord[i].Event.KeyEvent.wVirtualKeyCode;
		  //c = inRecord[i].Event.KeyEvent.uChar.AsciiChar;
#else
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
#endif
    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -1.0;
        dirty = true;
        break;
    }
   

    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_.publish(twist);    
      dirty=false;
    }
#ifdef _WINDOWS
	  }
	}
#endif
  }


  return;
}



