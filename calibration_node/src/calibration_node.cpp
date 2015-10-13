#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include <vector>
#include "leica_ts12/tsData.h"
#include "tf2_msgs/TFMessage.h"
#include "tf/tf.h"
#include "calibration/pointError.h"


//calibration::pointError pointError[100];

calibration::pointError pointError;

geometry_msgs::TransformStamped polhi;
geometry_msgs::Point polhpoint;
leica_ts12::tsData leicapoint;
bool pointAcquired=false;

static int numberofpoints=0;

float offsetx=0;
float offsety=0;
float offsetz=0;

int offsets=0;

void polhemusCallback(const tf2_msgs::TFMessage& polhmsg)
{
  
  polhi=polhmsg.transforms.back(); 
  polhpoint.x=polhi.transform.translation.x;
  polhpoint.y=polhi.transform.translation.y;
  polhpoint.z=polhi.transform.translation.z;
}

void tsCallback(const leica_ts12::tsData& tsmsg)
{
   //This assures they are the same point, since they are collected at the same time(at the same second, at least)
   //minus offsets. Remeber
   if(polhi.header.stamp.sec==tsmsg.header.stamp.sec){
   pointError.point=polhpoint;
   pointError.error.x=tsmsg.point.x+offsetx-polhpoint.x;
   pointError.error.y=tsmsg.point.y+offsety-polhpoint.y;
   pointError.error.z=tsmsg.point.z+offsetz-polhpoint.z;

   ROS_ERROR("Polhemus without offsets: %f %f %f",polhpoint.x+offsetx, polhpoint.y+offsety, polhpoint.z+offsetz);
   pointAcquired=true;
   numberofpoints++;
  }    
}

int main(int argc, char **argv)
{   
  int input=0;
  ros::init(argc, argv, "calibration_n");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/ts_data", 1, tsCallback);
  ros::Subscriber sub2 = n.subscribe("/tf", 1, polhemusCallback);
  ros::Publisher publisher = n.advertise<calibration::pointError >("calibrationpoints",300);
  ros::Rate r(150);
  ros::Rate c(10);
  
//ros::spin();
//   while(1){    
//   
//   if(offsets==1)
//   {
//       offsetx=pointError.error.x;
//       offsety=pointError.error.y;
//       offsetz=pointError.error.z;
//       break;
//       }
//   }
//    
//  ROS_ERROR("oFFSET ACQUIRED. Move!");
//  ROS_ERROR("offsets: %f %f %f",offsetx, offsety, offsetz);
  
    
  while(ros::ok){
  ROS_ERROR("Enter 1!"); 
  std::cin >> input;
  
  if(input==1){
    
     while(pointAcquired==false)
     {
       ros::spinOnce();
     }  
     publisher.publish(pointError);
     ROS_ERROR("Point Acquired!");
     pointAcquired=false;
     input=0;       
   }      
  } 
  
  exit(1);

}
