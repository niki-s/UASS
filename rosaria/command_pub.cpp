/*
	Command Publisher
	Accepts commands from Unity Client through UDP connection
	Computes desired angle to turn and distance to cover
	Uses tf and odometry to control movement
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <cmath>
//#include <mutex>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <iostream>
#include <string.h>
#include <vector>

#define PI 3.14159265

using namespace std;

//Robot Driver
//used to publish direction and angle commands to the pioneer
//appologies in advance for not creating more appropriate functions for command
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
		printf("moved: %f\n",dist_moved);
      if(dist_moved >= distance) done = true;
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
    while(radians < 0) radians += 2*PI;
    while(radians > 2*PI) radians -= 2*PI;

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
      printf("angle turned: %f\n", angle_turned);
      
      if ( fabs(angle_turned) < 1.0e-2) continue;

      if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
        angle_turned = 2 * PI - angle_turned;

      if (angle_turned >= radians) done = true;
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

char myID [256];

// waits for a UDP message from Unity to arrive and turns message into a string
void receiveUnityCommand(char* command);

// processes the command to find out and return:
// how much to turn in what direction
// how far to travel
void processCommand(char* command, double& dist, double& rad);

//given position wanted, returns calculated distance from position variables
double calculateDistance(double x, double y, double z);

//given position wanted, returns calculated radians to turn to face new position
double calculateAngle(double x, double y, double z);

// publishes the wanted command to pioneer odometry
// distance is the distance wanted traversed
// radius is the turning radius wanted
// clockwise is what direction to turn
void commandPioneer(RobotDriver* driver, const double distance, const double radians, const bool clockwise);

//callback for receiving unit ID
//callback sets global variables for position x, y, and z
void callback(const std_msgs::StringConstPtr& str);

//callback for odometry information
void poseCallback(const nav_msgs::Odometry::ConstPtr& odomsg);

//tokenizer to parse udp message
vector<string> Tokenizer(string str);


//--globals
//position
double x, y, z = 0;

//**********//

int main (int argc, char** argv)
{
	int fakeArgc = 1;

	ros::init(fakeArgc, argv, "pioneer_command_publisher");
	ros::NodeHandle n;

	ros::Rate r(10);

	RobotDriver driver(n);
	double dist;
	double rad;
	bool clockw = true;
	
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
	else
	{
		printf("Socket bound\n");
	}
	
	// bind receive socket to read port 8054
	memset((char*)&myaddr,0,sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(8052);

	if(bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0)
	{
		printf("ERROR BINDING SOCKET\n");
		return 0;
	}
	else
	{
		printf("Socket bound\n");
	}
	
	string name = n.resolveName("/RosAria/Pose");
	
	//subscribe to id_sub which recieves the ID of the current unit
	ros::Subscriber id_sub = n.subscribe("current_ID", 1, callback);
	
	//subscribe to odometry information for current position
	//ros::Subscriber odo_sub = n.subscribe<nav_msgs::Odometry>(name, 1, poseCallback);
	ros::Subscriber odo_sub = n.subscribe<nav_msgs::Odometry>("RosAria/pose", 1, poseCallback);
	//while ros is running continue to recieve commands and relay to pioneer
	
	ros::spinOnce();
	
	while(1)
	{		
		ros::spinOnce();
		
		receiveUnityCommand(myCommand);
		printf("%s\n", myCommand);
		
		processCommand(myCommand, dist, rad);
		commandPioneer(&driver, dist, rad, clockw);
		
		ros::spinOnce();
		
		printf("New Position: (%.4f,%.4f,%.4f) \n", x, y, z);
	}
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
		
		printf(".");
		
		//if the message is a command message
		if(buffer[0] == '2')
		{
			waiting = false;
			printf("\nCommand Received!\n");
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

void processCommand(char* command, double& dist, double& rad)
{
	double cmndX, cmndY, cmndZ = 0.0;
	const char * temp;
	
	//first, extract the wanted position from the command using tokenizer
	vector<string> tokens = Tokenizer(command);
	
	//use a temporary const char* to convert string in vector to cstring
	//then convert this cstring into a float for further command processing
	temp = tokens[2].c_str();
	cmndY = atof(temp);
	temp = tokens[3].c_str();
	cmndZ = atof(temp);
	temp = tokens[4].c_str();
	cmndX = atof(temp);
	
	printf("%f, %f, %f\n", cmndX, cmndY, cmndZ);
	
	//determine the distance to rotate
	//determines direction to rotate through clockw(ise) boolean
	rad = calculateAngle(cmndX, cmndY, cmndZ);	
	//determine the distance to travel
	dist = calculateDistance(cmndX, cmndY, cmndZ);
}

double calculateDistance(double cmndX, double cmndY, double cmndZ)
{
	double dist = 0;
	dist = sqrt(pow((cmndX - x),2)+pow((cmndY - y),2));
	printf("Distance: %f\n", dist);
	return dist;
}

double calculateAngle(double cmndX, double cmndY, double cmndZ)
{
	double rad = 0;
	printf("Commanded: %f, %f, %f\nCurrent: %f, %f, %f\n", cmndX, cmndY, cmndZ, x, y, z);
	
	
	rad = atan2((cmndX - x), (cmndY - y));
	//adjust for lack of sensors, it generally overshoots right now
	rad = rad - (rad*0.1);
	printf("Angle to move: %f\n", rad);
	return rad;

}

void commandPioneer(RobotDriver* driver, const double distance, const double radians, const bool clockwise)
{
	//slight movement even when called with 0 rad
	if (radians != 0)
	{
		(*driver).turnOdom(clockwise, radians);
	}
	(*driver).driveForwardOdom(distance);
	
}

void callback(const std_msgs::StringConstPtr& str)
{
	char id [256];
	int index = 0;
	while((*str).data[index] != '\0')
	{
		id[index] = (*str).data[index];
		index++;
	}
	id[index] = '\0';
	
	if (strcmp(id, myID) != 0)
	{
		strcpy(myID, id);
	}
}

void poseCallback(const nav_msgs::Odometry::ConstPtr& odomsg)
{	
	x = odomsg->pose.pose.position.x;
	y = odomsg->pose.pose.position.y;
	z = odomsg->pose.pose.position.z;
	ROS_INFO("Odo");
}

vector<string> Tokenizer(string str)
{
	vector<string> tokens;
	// Skip delimiters at beginning.
	string::size_type lastPos = str.find_first_not_of(" ", 0);
	// Find first "non-delimiter".
	string::size_type pos = str.find_first_of(" ", lastPos);
	
	while (string::npos != pos || string::npos != lastPos)
	{
		// Found a token, add it to the vector.	
		tokens.push_back(str.substr(lastPos, pos - lastPos));
		// Skip delimiters. Note the "not_of"
		lastPos = str.find_first_not_of(" ", pos);
		// Find next "non-delimiter"
		pos = str.find_first_of(" ", lastPos);
	}
	return tokens;
}
