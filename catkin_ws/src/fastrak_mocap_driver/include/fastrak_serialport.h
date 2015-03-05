/*
Author: Eohan George, Pushkar Kolhe 
Nov 17, 2010

Revised by: Jorge Diosdado
February 2015
*/

#ifndef FASTRAK_SERIALPORT_H
#define FASTRAK_SERIALPORT_H
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>

// the max number of receivers
#define MAX_NUM_RECEIVERS 4
// the max characters to pass to check for correct data consistency
#define MAX_TRIES_CORRECT_CBUFFER 3
// max size of circ buffer - I think this is the max the serial can output too
#define CIRC_BUFFER_SIZE 500
namespace polyhemus
{
	typedef std::pair<int,tf::Transform > StationData;
#pragma pack(1)
	struct Data
	{
		char record_type;
		// the sensor number that transmitts
		char station_num;
		char space;
		// x,y,z position
		float x,y,z;
		// xq,yq,zq,wq quarternions
		float wq,xq,yq,zq;
		// this is a delimiting character for checking;
		char end_char;
	};

#pragma pack()

	class Fastrak
	{
		std::vector <StationData> sensTransform;
		// the serial port structure from termios
		struct termios tio;
		int tty_fd;
		std::string deviceName;
		// pragma pack required so as to enable the data structure to be packed by each byte
		int a;

		Eigen::Quaterniond quat;
		boost::circular_buffer <char> cb;
		// the number of characters missing from earlier data
		int leftOver;
		/* returns the number of steps skipped to obtain a valid Data segment
		 *  returns -1 if no valid ones obtained
		 */
		float factor; // handles the centimetre to metre conversion
		int moveToConsistentData(char *arr,int arrSize);
		void populateTf(Data &dat, std::vector<StationData> & vecTransform);
		void setDistanceDivisionFactor(float factor_);
		void calibrateSensorValue(Data &_dat, Data &dat);
	public:
		Fastrak(const char *_deviceName)
		{
			leftOver = 0;
			deviceName =_deviceName;// ="/dev/ttyS0";
			cb.set_capacity(CIRC_BUFFER_SIZE);
		}

		Eigen::Vector3d quaternionToEulerAnglesZYX(Eigen::Quaterniond q);

		const std::vector <StationData>& getTransform()
		{
			return sensTransform;
		}
		bool checkDataConsistency(Data &dat);
		// init the serial port for reading
		bool init();
		// reads the sensor value and writes to the vector
		void getSensPosition(std::vector <StationData>& transforms);
		// reads the sensor values and writes to the class member variable
		void getSensPosition();
		int clearSensorTransforms()
		{
			int size = sensTransform.size();
			sensTransform.clear();
			return size;
		}
	};
}
#endif
