
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>
#include "ros/ros.h"
#include <sstream>

using namespace std;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
	cout << "Caught signal " << signum << endl;
	// Terminate program
	exit(signum);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "menu");

	ros::NodeHandle node_handle;

	ros::Rate loop_rate(100);   

	XmlRpc::XmlRpcValue params;

	Eigen::MatrixXd data;

	signal(SIGINT, signal_callback_handler);

	ros::Time t_init;
	double t = 0;
	double tf;
	int choice;
	int demo = -1;
	int yaml = 0;

	while (ros::ok()){
		demo = -1;
		if (yaml==1){
			choice = 5;
		}else{
			cout<<"choice:   (1: , 2: , 3: , 4:demos, 5:yaml) "<<endl;
			cin>>choice;
		}
		if (choice == 1){
			cout<<"time_f "<<endl;
			cin>>tf;
		}else if (choice == 2){
			cout<<"sorry, not implemented yet"<<endl;
		}else if (choice == 3){
			cout<<"sorry, not implemented yet"<<endl;
		}else if (choice == 4){
			cout<<"select demo:   (1:  , 2:  , 3:  , ...-soon other demos-"<<endl;
			cin>>demo;
			cout<<"insert time_f: "<<endl;
			cin>>tf;
		}else if (choice == 5){
			if (yaml==0){
				yaml = 1;
				if(!node_handle.getParam("/params", params)){		// load params from yaml file
					ROS_ERROR("Could not get the XmlRpc value.");
				}
				if(!parseParameter(params, data, "DATA")){			// extract data from DATA entry
					ROS_ERROR("Could not parse params.");
				}
			}
			if(1){		// stop yaml condition 
				yaml = 0;
			}
		}

		ros::spinOnce();

		t_init = ros::Time::now();
		if(yaml==0){
			// yaml code here!
		}
		t = (ros::Time::now() - t_init).toSec();

		while (t <= tf){
			if (choice == 1){
				// code here !
			} else if (choice == 2){
				// code here !
			}
		}

		loop_rate.sleep();

		t = (ros::Time::now() - t_init).toSec();
	}
	return 0;
}