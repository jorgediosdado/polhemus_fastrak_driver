polhemus=load('fo_Polh_tf.txt');     #Load data from Polhemus  (tf2_msgs/TFMessage)
ts=load('fo_ts_pose.txt');           #Load data from total station (geometry_msgs/PoseStamped= std_msgs/Header+geometry_msgs/Pose)
                                    #geometry_msgs/Pose= geometry_msgs/Point + geometry_msgs/Quaternion 
tstimedata=load('fo_ts_data.txt');        #Load time data. It needs to be loaded from ts_data                      
                          
                       
 NUMDATAPOLH=5468;             #Number of data from the two different sensors.
 NUMDATATS=549;                                           
                                 
              
                          
cte=1423458500000000000*ones(NUMDATAPOLH,1);     #This constant may be changed with different data!!!!
              
              
#Polhemus data
trans_x_polh=polhemus(:,4);
trans_x_polh=trans_x_polh*2.54;               #inches-->cm


trans_y_polh=polhemus(:,5);
trans_y_polh=trans_y_polh*2.54;               

trans_z_polh=polhemus(:,6);
trans_z_polh=trans_z_polh*2.54;
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



#Error
trans_polh_interpx=interp1(time_polh,trans_x_polh,time_ts); #data from polhemus sensor are interpolated in order to find the error with a simple substraction
errorx=trans_x_ts-trans_polh_interpx;

trans_polh_interpy=interp1(time_polh,trans_y_polh,time_ts); 
errory=trans_y_ts-trans_polh_interpy;

trans_polh_interpz=interp1(time_polh,trans_z_polh,time_ts); 
errorz=trans_z_ts-trans_polh_interpz;


 
#Graphics  #Maybe one without removing the offset
subplot(3,1,1); plot(time_polh,trans_x_polh); title('trans_x vs time'); hold on;
plot(time_ts,trans_x_ts); hold off;
subplot(3,1,2); plot(time_ts,errorx); title('error');
subplot(3,1,3); plot(trans_x_ts, errorx);

#subplot(3,3,4); plot(time_polh,trans_y_polh); title('trans_y vs time'); hold on;
#plot(time_ts,trans_y_ts); hold off;
#subplot(3,3,5); plot(time_ts,errory); title('error');
#subplot(3,3,6); plot(trans_y_ts, errory);

#subplot(3,3,7); plot(time_polh,trans_z_polh); title('trans_z vs time'); hold on;
#plot(time_ts,trans_z_ts); hold off;
#subplot(3,3,8); plot(time_ts,errorz); title('error');
#subplot(3,3,9); plot(trans_z_ts, errorz);






