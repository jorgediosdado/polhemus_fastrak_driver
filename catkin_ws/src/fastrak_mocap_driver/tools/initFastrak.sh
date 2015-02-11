if [ $1 ]
then
	echo "Setting read format settings to polyhemus fastrak "
	echo O1,52,61,50  > $1 # /dev/ttyS0
	echo O2,52,61,50  > $1 #/dev/ttyS0
	echo O3,52,61,50  > $1 #/dev/ttyS0
	echo O4,52,61,50  > $1 #/dev/ttyS0
	echo u  > $1 #metric
	echo f  > $1 #binary
	echo C > $1 #continuous
else
		echo "Error: Give device filename as arg 1 "
fi

