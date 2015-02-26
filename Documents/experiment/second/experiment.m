polhemus=load('s_Polh_tf.txt');     #Load data from Polhemus  (tf2_msgs/TFMessage)
ts=load('s_ts_pose.txt');           #Load data from total station (geometry_msgs/PoseStamped= std_msgs/Header+geometry_msgs/Pose)
imu=load('imu.txt');                                   #geometry_msgs/Pose= geometry_msgs/Point + geometry_msgs/Quaternion 
tstimedata=load('s_ts_data.txt');        #Load time data. It needs to be loaded from ts_data                      
                          
                    
                 
              
              
 NUMDATAPOLH=4138;             #Number of data from the two different sensors.
 NUMDATATS=356;
 NUMDATAIMU=9152;
 
cte=1423458500000000000*ones(NUMDATAPOLH,1);     #This constant may be changed with different data!!!!
              
              
              
              
#Polhemus data
trans_x_polh=polhemus(:,4);
trans_x_polh=trans_x_polh*2.54;               #inches-->cm
trans_y_polh=polhemus(:,5);
trans_y_polh=trans_y_polh*2.54;               
trans_z_polh=polhemus(:,6);
trans_z_polh=trans_z_polh*2.54;
totaltranspolh=sqrt(trans_x_polh.^2 + trans_y_polh.^2 + trans_z_polh.^2);

or_polh_x=polhemus(:,7);                      #Orientation from polhemus sensor
or_polh_y=polhemus(:,8);
or_polh_z=polhemus(:,9);
or_polh_w=polhemus(:,10);

time_polh=polhemus(:,3)-cte;                  #cte is used to scale the plot





#Total station data  

time_ts=tstimedata(:,1)-cte(1:NUMDATATS,1);        #cte is used to scale the plot

trans_x_ts=100*ts(:,4);                      #meters->cm
offsetx=trans_x_ts(2)-trans_x_polh(2);        #offset
offsetvectorx=offsetx*ones(NUMDATATS,1);
trans_x_ts=trans_x_ts-offsetvectorx;

trans_y_ts=100*ts(:,5); 
offsety=trans_y_ts(2)-trans_y_polh(2);        
offsetvectory=offsety*ones(NUMDATATS,1);
trans_y_ts=trans_y_ts-offsetvectory;

trans_z_ts=100*ts(:,6);
offsetz=trans_z_ts(2)-trans_z_polh(2);        
offsetvectorz=offsetz*ones(NUMDATATS,1);
trans_z_ts=trans_z_ts-offsetvectorz;

totaltransts=sqrt(trans_x_ts.^2+trans_y_ts.^2+trans_z_ts.^2);


#IMU data

or_imu_x=-imu(:,4);                               #Different reference frames!!
offsetorvectorx=(or_imu_x(10)-or_polh_x(10))*ones(NUMDATAIMU,1);
or_imu_x=or_imu_x-offsetorvectorx;

or_imu_y=-imu(:,5);
offsetorvectory=(or_imu_y(2)-or_polh_y(2))*ones(NUMDATAIMU,1);
or_imu_y=or_imu_y-offsetorvectory;

or_imu_z=-imu(:,6);
offsetorvectorz=(or_imu_z(2)-or_polh_z(2))*ones(NUMDATAIMU,1);
or_imu_z=or_imu_z-offsetorvectorz;

or_imu_w=-imu(:,7);
offsetorvectorw=(or_imu_w(2)-or_polh_w(2))*NUMDATAIMU;
or_imu_w=or_imu_w-offsetorvectorw;

time_imu=imu(:,1);


cte_imu=1423458500000000000*ones(NUMDATAIMU,1);
time_imu=time_imu-cte_imu;


#Error
trans_polh_interpx=interp1(time_polh,trans_x_polh,time_ts); #data from polhemus sensor are interpolated in order to find the error with a simple substraction
errorx=trans_x_ts-trans_polh_interpx;

trans_polh_interpy=interp1(time_polh,trans_y_polh,time_ts); 
errory=trans_y_ts-trans_polh_interpy;

trans_polh_interpz=interp1(time_polh,trans_z_polh,time_ts); 
errorz=trans_z_ts-trans_polh_interpz;

totaltransinterp=interp1(time_polh,totaltranspolh,time_ts); 
errortotal=abs(totaltransts-totaltransinterp);              #absolute diference between both sensors
 

 
figure
subplot(2,1,1); plot(time_polh,totaltranspolh,'r'); title('distance vs time'); xlabel('time'); ylabel('distance in cm'); hold on; grid on;
plot(time_ts,totaltransts, 'b'); hold off;
subplot(2,1,2); plot( trans_x_ts, errortotal);title('error-distance'); grid on; xlabel('distance'); ylabel('absolute error');
#subplot(3,1,2); plot(time_ts,errortotal); title('error-time'); This plot is a bit useless

#subplot(3,1,1); plot(time_polh,trans_x_polh,'r'); title('trans_x vs time'); hold on;grid on;xlabel('time'); ylabel('distance in cm');
#plot(time_ts,trans_x_ts); hold off;
#subplot(3,1,2); plot(time_ts,errorx); title('error-time');
#subplot(3,1,3); plot(trans_x_ts, errorx);

#subplot(3,1,2); plot(time_polh,trans_y_polh,'r'); title('trans_y vs time'); hold on;grid on;xlabel('time'); ylabel('distance in cm');
#plot(time_ts,trans_y_ts); hold off;
#subplot(3,3,5); plot(time_ts,errory); title('error-time');
#subplot(3,3,6); plot(trans_y_ts, errory);


#subplot(3,1,3); plot(time_polh,trans_z_polh,'r'); title('trans_z vs time'); hold on;grid on;xlabel('time'); ylabel('distance in cm');
#plot(time_ts,trans_z_ts); hold off;
#subplot(3,1,2); plot(time_ts,errorz); title('error-time');
#subplot(3,1,3); plot(trans_z_ts, errorz);



figure
plot(time_polh, or_polh_x,'g'); hold on;
plot(time_imu,or_imu_x); hold off;

figure
plot(time_polh, or_polh_y,'g'); hold on;
plot(time_imu,or_imu_y); hold off;

figure
plot(time_polh, or_polh_z,'g'); hold on;
plot(time_imu,or_imu_z); hold off;

figure
plot(time_polh, or_polh_w,'g'); hold on;
plot(time_imu,or_imu_w); hold off;

