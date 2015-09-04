#include <cstdio>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include "fastrak_serialport.h"
#include <sstream>
#include "leica_ts12/tsData.h"
#include "tf2_msgs/TFMessage.h"


using namespace std;
using namespace polyhemus;

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
	string defserial="/dev/ttyUSB0";		    //This is the default value for the serial port
	n.param("serial", serialport,defserial);
	Fastrak ftrk(serialport.c_str(), n);
	
	tf::TransformBroadcaster tfBr;  //TransformBroadcaster object 
					// hardcoding the sensor reading rate -- check
	ros::Rate r(60);
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
	subs1=n.subscribe("/calibrationpoints", 15, storing);
	ros::spinOnce();
	
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




/*


	if(activatecalibration==1)
        {
	   ros::Subscriber sub = n.subscribe("/ts_data", 1, tsCallback);
           ros::Subscriber sub2 = n.subscribe("/tf", 1, polhemusCallback);
	   ros::Rate c(150);

	   ros::spinOnce(); 
	   offsetx=leicapoint.point.x-polhpoint.x;
           offsetx=leicapoint.point.y-polhpoint.y;
           offsetx=leicapoint.point.z-polhpoint.z;

	   while(ros::ok){
             if(i>99)
                {
                  for(int j=0;j<100;j++)
                   {
                   ftrk.pointError[i]=pointError[i];
                   }
                break;
                }
             ros::spinOnce();
             c.sleep();
                }
           activatecalibration=0;
           
    	   }*/


// void polhemusCallback(const tf2_msgs::TFMessage& polhmsg)
// {
//   polhi=polhmsg.transforms.back(); 
//   polhpoint.x=polhi.transform.translation.x;
//   polhpoint.y=polhi.transform.translation.y;
//   polhpoint.z=polhi.transform.translation.z;
// }
// 
// void tsCallback(const leica_ts12::tsData& tsmsg)
// {
//    if(polhi.header.stamp.sec==tsmsg.header.stamp.sec){
//    pointError[i].point=tsmsg.point;
//    pointError[i].error.x=tsmsg.point.x-offsetx-polhpoint.x;
//    pointError[i].error.y=tsmsg.point.y-offsety-polhpoint.y;
//    pointError[i].error.z=tsmsg.point.z-offsetz-polhpoint.z;
//    i++;}    
// }
