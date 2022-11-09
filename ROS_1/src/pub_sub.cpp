#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>
#include "ros/ros.h"
#include <sstream>

#define NAME "pub_sub"
#define PUB_MESSAGE pub_msg_class
#define SUB_MESSAGE sub_msg_class
#define PUB_TOPIC "/pub_topic"
#define SUB_TOPIC "/sub_topic"
#define PUB_QUEUE 1
#define SUB_QUEUE 1
#define NODE_RATE 10

using namespace std;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
	cout << "Caught signal " << signum << endl;
	// Terminate program
	exit(signum);
}

void sub_callback(const SUB_MESSAGE& msg) {
	// ----- retrieve data from message ----- //
	// code here!
	// ----- end ----- //
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, NAME);

	ros::NodeHandle node_handle;

	ros::Publisher pub = node_handle.advertise<PUB_MESSAGE>("PUB_TOPIC", PUB_QUEUE);

	ros::Subscriber sub_cmd =  node_handle.subscribe("SUB_TOPIC", SUB_QUEUE, &sub_callback);

	ros::Rate loop_rate(NODE_RATE);

	MESSAGE_CLASS msg;

	signal(SIGINT, signal_callback_handler);	// ctrl+c

	while (ros::ok()){
		
		ros::spinOnce();	// run callbacks
	
		// ----- build message ----- //
		msg.header.stamp = ros::Time::now();
		// code here!
		// ----- end ----- //
		pub.publish(msg);

		loop_rate.sleep();
	}
	return 0;
}