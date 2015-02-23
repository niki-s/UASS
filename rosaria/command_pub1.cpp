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

class command_pub

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

int main(int argc, char** argv)
{
	int fakeArgc = 1;
	bool stillWaiting = true;
	myID[0] = '\0';

	ros::init(fakeArgc, argv, "pioneer_command_publisher");
	ros::NodeHandle n;

	ros::Rate r(10);

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


	// wait for client to send commands while ros is still running
	while(ros::ok())
	{
		//upon receiving a message
		while(stillWaiting)
		{
			recvlen = recvfrom(fd, buffer, 256, 0, (struct
		                      sockaddr*)&remaddr, &addrlen);
			buffer[recvlen] = '\0';
			
			//if the message is a command message
			if(buffer[0] == '2')
			{
				stillWaiting = false;
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
		stillWaiting = true;
	}
	return 0;
}
