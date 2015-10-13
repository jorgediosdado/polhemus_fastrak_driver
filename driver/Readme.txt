This driver allows to use four Fastrak transmitter over ROS. The information is published over a tf topic. For information on the configuration of the sensor, check the manual on how to change parameters. 

How to calibrate the Polhemus for noisy environments, with a total station or any other sensor that's accurate enough.

	1.- Launch the ROS fastrak driver.
	2.- Set the Leica reference frame roughy the same as the polhemus reference frame (This is done by 		moving the Leica prism to two positions in space, forming one of the axis, see manual). The 		orientation of both referenec frames must be identical. 
 	3.- Stick together the prism and the Polhemus sensor and take note of the offsets (with velcro, for 		example)(the position reported by the polhemus for the transmitter in the origin) This point will be 		the origin of both reference frames.

	3.- Launch the total station driver

	4.- Input the offsets in the calibration_node and compile it. Make sure both sensors are reporting 		cero for the origin by rostopic-echoing the topics.  

	5.- run calibration_node and collect 8 points as in a cube, or another shape preferred. 

	6.- Enjoy.

Note: The automatic removal of the offsets is written, but not tested. 
