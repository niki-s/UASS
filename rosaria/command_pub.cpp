/*
	Command Publisher
	Accepts commands from Unity Client through UDP connection
	Computes desired angle to turn and distance to cover
	Uses tf and odometry to control movement
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include<cstdio>
#include<cstdlib>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <iostream>
#include <string.h>

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"


class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  //! We will be listening to TF transforms as well
  tf::TransformListener listener_;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
  }

  //! Drive forward a specified distance based on odometry information
  bool driveForwardOdom(double distance)
  {
    //wait for the listener to get the first message
    listener_.waitForTransform("base_link", "odom", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_link", "odom", 
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at 0.25 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 0.25;
    
    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh_.ok())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_link", "odom", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();
		printf("%f\n",dist_moved);
      if(dist_moved > distance) done = true;
    }
    if (done)
    {
    	base_cmd.linear.x = 0;
    	cmd_vel_pub_.publish(base_cmd);
    	return true;
    }
    return false;
  }
   bool turnOdom(bool clockwise, double radians)
  {
    while(radians < 0) radians += 2*M_PI;
    while(radians > 2*M_PI) radians -= 2*M_PI;

    //wait for the listener to get the first message
    listener_.waitForTransform("base_link", "odom", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_link", "odom", 
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to turn at 0.75 rad/s
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 0.75;
    if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;
    
    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0,0,1);
    if (!clockwise) desired_turn_axis = -desired_turn_axis;
    
    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh_.ok())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_link", "odom", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      tf::Vector3 actual_turn_axis = 
        relative_transform.getRotation().getAxis();
      double angle_turned = relative_transform.getRotation().getAngle();
      if ( fabs(angle_turned) < 1.0e-2) continue;

      if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
        angle_turned = 2 * M_PI - angle_turned;

      if (angle_turned > radians) done = true;
    }
    if (done)
    {
    	base_cmd.angular.z = 0;
    	cmd_vel_pub_.publish(base_cmd);
    	return true;
    }
    return false;
  }
};

//end RobotDriver Class definition

// networking bits
int sockfd,n;
struct sockaddr_in servaddr,cliaddr;

struct sockaddr_in myaddr;
struct sockaddr_in remaddr;
socklen_t addrlen = sizeof(remaddr);
int recvlen;
int fd;

// waits for a UDP message from Unity to arrive and turns message into a string
void receiveUnityCommand(char* command);

// processes the command to find out and return:
// how much to turn in what direction
// how far to travel
void processCommand(char* command, double &distance, double &radius, bool &clockwise);

// publishes the wanted command to pioneer odometry
// distance is the distance wanted traversed
// radius is the turning radius wanted
// clockwise is what direction to turn
void commandPioneer(RobotDriver* driver, const double distance, const double radians, const bool clockwise);

//callback for receiving unit ID
void callback(const std_msgs::StringConstPtr& str);

//RobotDriver test main
int main (int argc, char** argv)
{
	int fakeArgc = 1;

	ros::init(fakeArgc, argv, "pioneer_command_publisher");
	ros::NodeHandle n;

	ros::Rate r(10);

	RobotDriver driver(n);
	double dist;
	double rad;
	bool clockw;
	
	char clientIP[256];
	char buffer[256];
	char myCommand[256];

	//get IP address from terminal to connect to correct Unity Client
	if(argc == 2)
	{
		sprintf(clientIP, "%s", argv[1]);
	}
 	else
  	{
		printf("Please enter the clients IP address in the cmd arguments\n");
		return 0;
	}
	
	// init receive socket
	if((fd = socket(AF_INET,SOCK_DGRAM,0)) < 0)
	{
		printf("ERROR CREATING SOCKET\n");
		return 0;
	}
	
	// bind receive socket to read port 8054
	memset((char*)&myaddr,0,sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(8054);

	if(bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0)
	{
		printf("ERROR BINDING SOCKET\n");
		return 0;
	}
	
	
	
	//subscribe to id_pub which recieves the ID of the current unit
	ros::Subscriberid_sub = n.subscribe("current_ID", 1, callback);
	
	//while ros is running continue to recieve commands and relay to pioneer
	while(ros::ok())
	{
		receiveUnityCommand(myCommand);
		printf("%s", myCommand);
		//if (myCommand[0] == 1)	//this is an id assignment
		//
		//if (myCommand[0] == 2)    //this is a move command, do below
		processCommand(myCommand, dist, rad, clockw);
		//commandPioneer(&driver, dist, rad, clockw);
		
		//should also include later
		//if (myCommand[0] == --anything else--)    //does other things
	}
	
	

	//driver.driveForwardOdom(1.0);
	//driver.turnOdom(false, 1.4);
	
	return 0;
}

void receiveUnityCommand(char* command)
{
	char *myCommand = new char[256];
	char buffer[256];
	bool waiting = true;
	while (waiting)
	{
		
		recvlen = recvfrom(fd, buffer, 256, 0, (struct sockaddr*)&remaddr, &addrlen);
		buffer[recvlen] = '\0';
		
		//if the message is a command message
		if(buffer[0] == '2')
		{
			waiting = false;
			printf("Command Received!\n");
			int i = 2;
			int j = 0;
			while(buffer[i] != '\0' && buffer[i] < 123)
			{
				myCommand[j] = buffer[i];
				i++;
				j++;
			}
			myCommand[j] = '\0';
			printf("My Command %s\n", myCommand);
		}
	}
	strcpy(command, myCommand);
	delete myCommand;
}

void processCommand(char* command, double &distance, double &radius, bool &clockwise)
{
}

void commandPioneer(RobotDriver* driver, const double distance, const double radians, const bool clockwise)
{
	(*driver).driveForwardOdom(distance);
	(*driver).turnOdom(clockwise, radians);
}

void callback(const std_msgs::StringConstPtr& str)
{
//finish this
}
