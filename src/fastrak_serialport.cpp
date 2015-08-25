
#include "fastrak_serialport.h"
using namespace polyhemus;


Fastrak::Fastrak(const char *_deviceName, ros::NodeHandle& n)
{
	leftOver = 0;
	deviceName =_deviceName;// ="/dev/ttyS0";
	cb.set_capacity(CIRC_BUFFER_SIZE);
	
}

Fastrak::~Fastrak() {}

void Fastrak::calibration()
{
	
	
}


void Fastrak::setDistanceDivisionFactor(float factor_)
{
	factor=factor_;
}


void Fastrak::calibrateSensorValue(Data &_dat, Data &dat)
{

	
	//f(x) = p1*x + p2
	ROS_DEBUG("Sensor Values UnCalibrated: %d x:%f y:%f z:%f    xq:%f yq:%f zq:%f wq:%f",_dat.station_num-48, 		_dat.x,_dat.y,_dat.z,_dat.xq,_dat.yq,_dat.zq,_dat.wq);
	
	float p1 = 1, p2 = 0*factor; 			//No calibration
	dat = _dat;
	dat.x = (_dat.x*p1 + p2)/factor;
	dat.y = (_dat.y*p1 + p2)/factor;
	dat.z = (_dat.z*p1 + p2)/factor;
}

bool Fastrak::init()
{
	
	setDistanceDivisionFactor(100);	// Convert the units. cm-->factor=1, m--> factor=100
	memset(&tio,0,sizeof(tio));
	tio.c_iflag=0;
	tio.c_oflag=0;
	tio.c_cflag=CS8|CREAD|CLOCAL; // set as per FASTRAK manual
	tio.c_lflag=0;
	tio.c_cc[VMIN]=47;
	tio.c_cc[VTIME]=5;
	
	//ROS_ERROR("PUERTO %s", deviceName);
	
	tty_fd=open(deviceName.c_str(), O_RDWR | O_NONBLOCK);
	if(tty_fd == -1 ) 
	{
		std::string er ("Device:"+deviceName +" could be opened");
		ROS_ERROR(er.c_str() ) ;
		ROS_ERROR("Device could not be opened");
		return false;
	}
	cfsetospeed(&tio,B115200); // Baud rate = 115200 
	cfsetispeed(&tio,B115200); // Baud rate = 115200 

	tcsetattr(tty_fd,TCSANOW,&tio);
					

	
			
    	char *continuous= "C\n";
	//std::string format("Ox,52,61,50\n");		//This is set to be the default configuration for the 								station 1. If there is a need to change it, there is a 								need to change the data structure, as well as the method 								to capture data
	//char *metric= "u\n";			
	//char *binary= "f\n"; 	

   	if(
	//write(tty_fd,metric,strlen(metric) ) <=0 ||
   	//write(tty_fd,binary, strlen(binary)) <=0 ||
	write(tty_fd,continuous, strlen(continuous)) <=0) 
   	
   	 {
   	     ROS_ERROR("could not write to serial port ");
    	    return false;
  	  }

	//write the ascii value of the sensor
   	//for(int i = 0 ; i < MAX_NUM_RECEIVERS; i++)
   	//{
   	//     format[1] = (char)('1' + i);
   	//     if (write(tty_fd, format.c_str(), format.length() ) <= 0)
   	//     {
     	//       ROS_ERROR("could not write to serial port ");
     	//       return false;
     	//     }
	//}

     

	return true;
}
bool Fastrak::checkDataConsistency(Data &dat)
{
	if( dat.record_type==48					// 0 number is starting character
		&& dat.station_num >= 48+1  			// min number of receivers in ASCII		
		&& dat.station_num <= 48+MAX_NUM_RECEIVERS  	// max number of receivers in ASCII
		&& dat.end_char == 32 )				// space character 
		return true;
	else 
		return false;
}

int Fastrak::moveToConsistentData(char *arr,int arrSize)
{
	int i = 0 ;
	while(i <= arrSize - (int)sizeof(Data))
	{
		void *p = reinterpret_cast<void*>(arr+i);
		Data *d = reinterpret_cast<Data*>(p);
		if(checkDataConsistency(*d) ==true ) 
			return i;
		i++;
	}
	// no consistent segment obtained , so return -1 
	return -1;
}
void Fastrak::populateTf(Data &_dat, std::vector<StationData> & vecTransform)
{
	Data dat;				
	calibrateSensorValue(_dat,dat);
	ROS_DEBUG("populating TF station: %d x:%f y:%f z:%f    xq:%f yq:%f zq:%f wq:%f",dat.station_num-48, 		dat.x,dat.y,dat.z,dat.xq,dat.yq,dat.zq,dat.wq);
	StationData stData;			
	stData.first = dat.station_num-48; 	// obtain the ASCII value 
	
	Eigen::Vector3d euler;
	//Broadcast transform

	tf::Transform tf; //tf is a transform object
	//tf.setOrigin( tf::Vector3(dat.x/factor,dat.y/factor,dat.z/factor) ); removing the factorization to 											calibration
	tf.setOrigin( tf::Vector3(dat.x,dat.y,dat.z) ); //setOrigin Returns the origin vector translation


	//Return a quaternion representing the rotation.
	tf.setRotation(tf::Quaternion(dat.xq,dat.yq,dat.zq,dat.wq) );

	//This section is only for debugging purposes
	quat.w()=dat.wq;
	quat.x()=dat.xq;
	quat.y()=dat.yq;
	quat.z()=dat.zq;		
	euler = Fastrak::quaternionToEulerAnglesZYX(quat);
	//ROS_ERROR("%f,%f,%f", euler[0]*180/3.1416, euler[1]*180/3.1416, euler[2]*180/3.1416);


	stData.second = tf; //tf se almacena en stData
	vecTransform.push_back(stData);
	ROS_DEBUG("Transform Vector Size: %d",vecTransform.size());
}

void Fastrak::getSensPosition ()
{
	char buf[CIRC_BUFFER_SIZE];
	// obtain the linearized version of the circular buffer
	cb.linearize();
	int num_char_to_read =( (cb.reserve())/sizeof(Data)  ) ;
	// reduce the number by one so as to allow leftOvers
	if(num_char_to_read > 1)
		num_char_to_read-=1;
	num_char_to_read*=sizeof(Data);
	num_char_to_read+= leftOver;
	ROS_DEBUG("Reading %d characters",num_char_to_read);
	// pass the buffer address to serial port read
	// read max amount of data from the serial port in multiples of the data size
	
	int bytesRead = read(tty_fd, buf , num_char_to_read);
	ROS_DEBUG("Sizeof(Data):%d  bytesRead:%d cb.size():%d ",sizeof(Data),bytesRead,cb.size() ) ;
	if(bytesRead == -1)
		ROS_WARN("Read Error from Serial port");
	if(bytesRead ==0)
		ROS_WARN("Read Error due to EOF");
	// write buf values to circular buffer  
	for(int i = 0 ; i < bytesRead; i ++ ) 
	{
		cb.push_back(buf[i]);
	}
	// linearizing - just in case
	
	char * arr = cb.linearize();
	// store how much data is left over and obtain the rest of the size in multiples of 
	// data size
	int cbSize = cb.size(); // the size of array and keeps track of size after reduction in while loop
	int move;
	int offset=0;		// the offset to move to find the next data structure
	while(true)
	{
		move = moveToConsistentData(arr+offset,cbSize ) ;
		// if no valid Data found
		if (move ==-1)
		{
			// pop the first bytes till the old one 
			//     xxxxxx[x|  |  |  |  | ]  .... delete the x's
			if(offset+cbSize !=cb.size() )
				ROS_ERROR("offset should be equal to popCount");
			// popCount could be negative too , accounts here
			// condition that the current offset could be at the last section of memory that has 				exactly the sizeof(Data)
			int popCount;		
			if(cbSize < sizeof(Data) ) 
			{
				popCount = offset;
				leftOver = sizeof(Data) - cbSize;
			}
			else 
			{
				popCount = cb.size()-sizeof(Data)+1;
				leftOver = 1; // coz if the size is greater than sizeof(Data), this would be 							returned 
			}
			for(int j = 0 ;j < popCount;j++)
			{
				cb.pop_front();
			}
			break;
		}
		ROS_DEBUG("Bytes Skipped: %d",move);
		// move the start pointer of the array forward
		offset+=move;
		Data *dat = (Data*)(arr + offset);
		cbSize-=move;
		populateTf(*dat,sensTransform);
		// reduce the size
		cbSize-=sizeof(Data);
		// move the pointer forward 
		offset+=sizeof(Data);
	}


}



Eigen::Vector3d Fastrak::quaternionToEulerAnglesZYX(Eigen::Quaterniond q)
 {
   // Implementation from RPG quad_tutorial
   Eigen::Vector3d euler_angles;
   euler_angles(0) = atan2(2*q.w()*q.x() + 2*q.y()*q.z(), q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());
   euler_angles(1) = -asin(2*q.x()*q.z() - 2*q.w()*q.y());
   euler_angles(2) = atan2(2*q.w()*q.z() + 2*q.x()*q.y(), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
   return euler_angles;
 }


void Fastrak::getSensPosition (std::vector <StationData>& transforms)
{
	getSensPosition();
	transforms = sensTransform;
}



