/*
	Position Listener
 Position listener is a subscriber node that listens to RosAria odemetry information and relays
it to unity server
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

#include<cstdio>
#include<cstdlib>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// networking bits
int sockfd,n;
struct sockaddr_in servaddr,cliaddr;

struct sockaddr_in myaddr;
struct sockaddr_in remaddr;
socklen_t addrlen = sizeof(remaddr);
int recvlen;
int fd;
//////////////////////////

char myID[256];

//callback
//input: navigation message from odometry
//output: sends udp packet 
void poseCallback(const nav_msgs::Odometry::ConstPtr& odomsg)
{
	char output[2048];
	output[0] = '\0';

	//create a quaternion to convert to euler angles
	tf::Quaternion quat(odomsg->pose.pose.orientation.x,
						odomsg->pose.pose.orientation.y,
						odomsg->pose.pose.orientation.z,
						odomsg->pose.pose.orientation.w);
	
	//euler angles
	double roll, pitch, yaw;
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	
	
	//save odometry into "output"
	//"3 MyID: %s x: %.4f y: %.4f z: %.4f Qua-x: %.4f Qua-y: %.4f Qua-z: %.4f Qua-w: %.4f\n"
	sprintf(output, "3 %s %.4f %.4f %.4f %.4f %.4f %.4f\n",
		myID,
		-odomsg->pose.pose.position.y,
		odomsg->pose.pose.position.z,
		odomsg->pose.pose.position.x,
		(roll*(180/3.1415)),
		(pitch*(180/3.1415)),
		180 - (yaw*(180/3.1415))); 
		
	/*sprintf(output, "3 %s %.4f %.4f %.4f %.4f %.4f %.4f %.4f \n",
		myID,
		-odomsg->pose.pose.position.y,
		odomsg->pose.pose.position.z,
		odomsg->pose.pose.position.x,
		odomsg->pose.pose.orientation.y,
		odomsg->pose.pose.orientation.z,
		odomsg->pose.pose.orientation.x,
		odomsg->pose.pose.orientation.w); 	
	*/

	//print position to terminal for visualization purposes
	ROS_INFO(output);

	//send UDP packet
	sendto(sockfd,output,strlen(output),0, (struct sockaddr *)&servaddr,sizeof(servaddr));
}



int main(int argc, char** argv)
{
	int fakeArgc = 1;
	bool stillWaiting = true;
	myID[0] = '\0';

	ros::init(fakeArgc, argv, "pioneer_listener");
	ros::NodeHandle n;

	ros::Rate r(10);

	char clientIP[256];
	char buffer[256];

	//get IP address from terminal
	if(argc == 2)
	{
		sprintf(clientIP, "%s", argv[1]);
	}
 	else
  	{
		printf("Please enter the clients IP address in the cmd arguments\n");
		return 0;
	}

	//udp socket networking, connects to clientIP through port 8052
	sockfd = socket(AF_INET,SOCK_DGRAM,0);
	bzero(&servaddr,sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr(clientIP);
	servaddr.sin_port = htons(8053);
//////new connection information
	// send join message to clientIP
	char message[256];

	// 0 - message join
	// 1 - unit type: pioneer
	sprintf(message, "0 1"); 

	// send output to socket
	sendto(sockfd,message,strlen(message),0,(struct sockaddr *)&servaddr,sizeof(servaddr));


	// init receive socket
	if((fd = socket(AF_INET,SOCK_DGRAM,0)) < 0)
	{
		printf("ERROR CREATING SOCKET\n");
		return 0;
	}

	// bind receive socket to read port 8051
	memset((char*)&myaddr,0,sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(8051);

	if(bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0)
	{
	printf("ERROR BINDING SOCKET\n");
	}

	// wait for client to send back unit ID
	while(stillWaiting)
	{
		recvlen = recvfrom(fd, buffer, 256, 0, (struct
                          sockaddr*)&remaddr, &addrlen);
		buffer[recvlen] = '\0';
		printf("received: %s\n", buffer);
		if(buffer[0] == '1')
		{
			stillWaiting = false;
			int i = 2;
			int j = 0;
			while(buffer[i] != '\0' && buffer[i] != ' ' && buffer[i] < 123)
			{
				myID[j] = buffer[i];
				i++;
				j++;
			}
			myID[j] = '\0';
		}
	}

      printf("My ID: %s\n", myID);
//////
  //test return 0

	//subscribe to pose info
	ros::Subscriber pose_sub = n.subscribe<nav_msgs::Odometry>("RosAria/pose", 1, poseCallback);

	
	//publish this unit's ID
	ros::Publisher id_pub = n.advertise<std_msgs::String>("current_ID", 1);
	std_msgs::String pubID;
	pubID.data = myID;
	id_pub.publish(pubID);
  
	//ros continues its own loop
	ros::spin();
}
