#include <cstdio>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include "fastrak_serialport.h"
#include <sstream>
#include "leica_ts12/tsData.h"
#include "tf2_msgs/TFMessage.h"


using namespace std;
using namespace polhemus;

calibration::pointError pointError[14];
geometry_msgs::TransformStamped polhi;
geometry_msgs::Point polhpoint;
leica_ts12::tsData leicapoint;
int activatecalibration=1;

int i=0;
static int j=0;

float offsetx=0;
float offsety=0;
float offsetz=0;


void storing(const calibration::pointError& msg)
{

 if(j<14){
     pointError[j]=msg;
     ROS_ERROR("Callback, %d %f, %f, %f", j, pointError[j].point.x, pointError[j].point.y, pointError[j].point.z);
     j++;
}
}


int main(int argc, char**argv)
{
	ros::init(argc,argv,"testfastrak_one_sensor");
	ros::NodeHandle n;
	ros::Subscriber subs1;
  
	string serialport;
        string base;
        string sensor;
        string defaultport="/dev/ttyUSB0";
        string defaultbase="defaultbase";	
        string defaultsensor="defaultsensor";
	
	
      	//get parameter with default value   
	n.param("serial", serialport, defaultport);


	Fastrak ftrk(serialport.c_str(), n);
	
	tf::TransformBroadcaster tfBr;  
				
	ros::Rate r(60);
	std::ostringstream os;
	const vector<StationData> &sensTransform = ftrk.getTransform();
	if(ftrk.init()==false)
	{
		return -1;
	}


	n.param("/fastrak/base_frame",base,defaultbase);
	n.param("/fastrak/current_sensor",sensor,defaultsensor);
	

	subs1=n.subscribe("/calibrationpoints", 15, storing);
	ros::spinOnce();

	while(ros::ok())
	{
		
		ftrk.getSensPosition();
		int numTransforms = sensTransform.size();
		
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
		ROS_DEBUG("j= %d", j);
		if(j>=14)
                {
                  for(int i=0;i<14;i++)
                   {
                   ftrk.pointError[i]=pointError[i];
                   }
                   ftrk.calibrationcomplete=1;
                }
		
		ros::spinOnce();
		r.sleep();
	}
}

