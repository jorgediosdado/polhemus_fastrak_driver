
#include <cstdio>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include "fastrak_serialport.h"
#include <sstream>
using namespace std;
using namespace polyhemus;

calibration::pointError pointError[100];

void st()
{
   
}

int main(int argc, char**argv)
{
	ros::init(argc,argv,"testfastrak_one_sensor");
	ros::NodeHandle n;
	ros::Subscriber subs1;
        int i;

	string serialport;
	string defserial="/dev/ttyUSB0";		    //This is the default value for the serial port
	n.param("serial", serialport,defserial);
	Fastrak ftrk(serialport.c_str(), n);  		   	
	
	
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
	
	subs1=n.subscribe("/calibrationpoints", 100, st);
	
	//for(i=0;i<101;i++)
	//{
        //  ftrk.pointError[i]=pointError[i];
        //}
	
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

