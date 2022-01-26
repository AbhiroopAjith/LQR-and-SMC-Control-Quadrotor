
clear; close; clc;rosshutdown
% ROS Setup

% foldername = '/home/mithulesh/catkin_ws/src/mav_comm/';
% rosgenmsg(foldername);
addpath('/home/mithulesh/catkin_ws/src/mav_comm/matlab_msg_gen_ros1/glnxa64/install/m')
savepath

clear classes
rehash toolboxcache


rosinit;


j1_effort = rospublisher('/ardrone/command/motor_speed');

JointStates = rossubscriber('/ardrone/ground_truth/odometry');

tau1 = rosmessage(j1_effort);
tau2 = rosmessage(j1_effort);






m = 0.445;
l = 0.125;
Ix = 2.7e-3;
Iy = 2.9e-3;
Iz = 5.3e-3;
Jr = 2.03e-5;
Ax = 2.5;
Ay = 1.3;
Az = 1.3;
g = 9.8;
k = 3e-6;
b = 1.14e-6;

lk = l*k;



A = [0 1;
     0 0];


B = [0;1];

lambda = [-3 -3];

Q = [1 0;0 1];
R = 1;

K = lqr(A,B,Q,R);



tic;
t = 0;
counter=1;

while(t < 20)
    t = toc;
     
    % read the joint states
    jointData = receive(JointStates);


    X1 = jointData.Pose.Pose.Position.Z;
    X2 = jointData.Twist.Twist.Linear.Z;

    X = [X1 X2]' - [10 ;0];
    Kn = [-0.3 -0.4];
    F = K*X;
    if((m*g-m*F)/b) > 0
        vel = sqrt((m*g-m*F)/b);
    else
        vel =0;
    end

    vel = double(sqrt((m*g-m*F)/b));
    tau1.AngularVelocities = [vel/4,vel/4,vel/4,vel/4];
    send(j1_effort,tau1);

    % you can sample data here to plot at the end
    Pos_data(counter) = X1;
    Vel_data(counter) = X2;
    Vel_input(counter) = vel/4;
    time_data(counter) = t;
    
    counter = counter + 1;




end

tau2.AngularVelocities = [0,0,0,0];
send(j1_effort,tau2);

% figure();
% subplot(2,1,1)
% plot(time_data,Pos_data);
% title("Position of Z")
% ylabel("Position (m)")
% xlabel("Time (s)")
% subplot(2,1,2)
% plot(time_data,Vel_data)
% title("Velocity of Z")
% ylabel("Velocity (m/s)")
% xlabel("Time (s)")
% 
% figure()
% plot(time_data,Vel_input)
% title("Input Velocity of Z")
% ylabel("Velocity (rpm)")
% xlabel("Time (s)")


%disconnect from roscore
rosshutdown;
% 


