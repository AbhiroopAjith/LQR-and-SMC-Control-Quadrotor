clear; close; clc;rosshutdown;

addpath('/home/mithulesh/catkin_ws/src/mav_comm/matlab_msg_gen_ros1/glnxa64/install/m')
savepath

clear classes
rehash toolboxcache

syms z1 z2 z3 
syms g F m k 

m = 5;
g = 10;
x11d = 10;

% x11  = z position
% x12 = z velocity
% 

Kftz = 6.3540;
q11 = 1;
k11 = 1;
dx11d = 0;
alpha11 = 1;
q6 = 1;
k6 = 1;
a11 = -Kftz/m;
ddzd = 0;
dzd = 0;


% z11 = x11d - x11;
% z12 = x12-dx11d-alpha11*z11;
% Sz = z12;
% U = m*(-q6*sign(Sz) - k6*Sz-a11x12+ddzd+alpha11*(dzd-x12)+g)/(cos(phi)*cos(theta));






rosinit;

j1_effort = rospublisher('/ardrone/command/motor_speed');

JointStates = rossubscriber('/ardrone/ground_truth/odometry');

tau1 = rosmessage(j1_effort);
tau2 = rosmessage(j1_effort);




% tau1.AngularVelocities = [685,685,685,685];
% tau2.AngularVelocities = [0,0,0,0];
% 
% send(j1_effort,tau1);
% pause(5)
% send(j1_effort,tau2);

% client = rossvcclient('/mav_msgs/Actuator');
% req = rosmessage(client);
% req.ModelName = 'rrbot';
% req.UrdfParamName = 'robot_description';
% req.JointNames = {'joint1','joint2,joint3,joint4'};
% req.JointPositions = [deg2rad(30), deg2rad(45)];
% resp = call(client,req,'Timeout',3);

tic;
t = 0;
counter=1;

while(t < 30)
    t = toc;
     
    % read the joint states
    jointData = receive(JointStates);
    x = [jointData.Pose.Pose.Position.Z, jointData.Pose.Pose.Position.Z, jointData.Twist.Twist.Linear.X,jointData.Twist.Twist.Linear.Y,jointData.Twist.Twist.Linear.Z];
    % x11  = z position
    % x12 = z velocity
    % 
    x11 = x(1);
    x12 = x(2);

    phi  = 1;
    theta = 1;
    ct = 7.732 * 10^-6;
    

    

    z11 = x11d - x11;
    z12 = x12-dx11d-alpha11*z11;
    Sz = z12;
    U = m*(-q6*sign(Sz) - k6*Sz-a11*x12+ddzd+alpha11*(dzd-x12)+g)/(1*1);
    interim = sqrt(U/4*ct)*15800*3

    tau1.AngularVelocities = [interim, interim, interim, interim];
    send(j1_effort,tau1);

%     k1=[23.5850,5.8875,5.1470,2.6104];
%     k2= [5.8875, 4.9875,1.5543,0.9970];
%    
%    tau1.Data = -k1*x;
%    tau2.Data = -k2*x;
   send(j1_effort,tau1);
%    send(j2_effort,tau2);
%    
%    g1(counter)=jointData.Position(1);
%    g2(counter)=jointData.Position(2);
%    g3(counter)=jointData.Velocity(1);
%    g4(counter)=jointData.Velocity(2);
%    force1(counter)=tau1.Data;
%     
%    force2(counter)=tau2.Data;  
%    
   time(counter)=t;
% 
%    counter=counter+1;

    % you can sample data here to plot at the end




    
       
   
  
end

tau2.AngularVelocities = [0,0,0,0];
send(j1_effort,tau2);


%disconnect from roscore
rosshutdown;