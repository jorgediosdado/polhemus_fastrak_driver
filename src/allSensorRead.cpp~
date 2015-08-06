
#include <cstdio>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include "fastrak_serialport.h"
#include <sstream>
using namespace std;
using namespace polyhemus;

int main(int argc, char**argv)
{
	ros::init(argc,argv,"testfastrak_one_sensor");
	/*if(argc < 2)
	{
		ROS_ERROR("Need device name as aaaaaargument");
		ROS_INFO("fastrak <serial_device_file> <tf_root_name_with_starting_slash> <tf_sensor_names_with_starting_slash>");
		return -1;
	}*/

	ros::NodeHandle n;
	string serialport;

	//const char* constante=serialport.c_str();   //casting for fastrak function
	//n.getParam("serial", serialport);	    //It can be better to use param

	string defserial="/dev/ttyS0";		    //This is the default value for the serial port
	n.param("serial", serialport,defserial);
	Fastrak ftrk(serialport.c_str());  		   	

	

	tf::TransformBroadcaster tfBr;  //TransformBroadcaster object 
					// hardcoding the sensor reading rate -- check
	

	
	ros::Rate r(50);
	std::ostringstream os;
	const vector<StationData> &sensTransform = ftrk.getTransform();
	if(ftrk.init()==false)
	{
		return -1;
	}

	string base;
	string sensor;

	string def1="/fastrak_transmitter";
	string def2="/polhemus_current_frame";
	n.param("base_frame",base,def1);
	n.param("current_sensor",sensor,def2);



	//if(argc >= 4)
	//{
	//	sensor = (char *)argv[3];
	//	base = argv[2];
	//}
	//else if (argc >= 3)
	//{
	//	base = argv[2];
	//}
	//string p ("Base: "+base+" Sensor: "+sensor);
	//ROS_INFO(p.c_str());

	
	
	while(ros::ok())
	{
		// read and publish tf
		// actually reads from the serial port
		ftrk.getSensPosition();
		int numTransforms = sensTransform.size();
		//cout <<"size :: " <<sensTransform.size() << endl;
		ROS_DEBUG("Number of Transforms %u", sensTransform.size());//numTransforms);
		for(uint i = 0; i<sensTransform.size();i++)
		{
			
			ROS_DEBUG_COND(i==0,"SENSORS READ");
			os.str("");
			os << sensor <<sensTransform[i].first;
			// publishes the data that has been read already into the topic
			tfBr.sendTransform(tf::StampedTransform(sensTransform[i].second, ros::Time::now(), base, 				os.str()));

						
			ROS_WARN("%d Base:%s Sensor:%s", sensTransform[i].first,base.c_str(),os.str().c_str() );
		}
		int cleared = ftrk.clearSensorTransforms();
		ROS_DEBUG("Cleared %u values in Transform Vector",cleared);
		ros::spinOnce();
		r.sleep();
	}
}

