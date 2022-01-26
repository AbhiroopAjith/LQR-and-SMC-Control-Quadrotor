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


u1 = 0;
u2 = 0;
u3 = 0;
u4 = 0;

o1 = 0;
o2 = 0;
o3 = 0;
o4 = 0;


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
d = 1.14e-7;

lk = l*k;



A1 = [0 1;
     0 0];




B1 = [0;1];




Q1 = [1 0;0 1];

R1 = 1;


K1 = lqr(A1,B1,Q1,R1);
prev = 0;


tic;
t = 0;
counter=1;
count = 0;

while(t < 20)
    t = toc;
    time_step = t - prev;
   
     
    % read the joint states
    jointData = receive(JointStates);


    X1 = jointData.Pose.Pose.Position.Z;
    X2 = jointData.Twist.Twist.Linear.Z;

    P1 = jointData.Pose.Pose.Orientation.X;
    P2 = jointData.Pose.Pose.Orientation.Y;
    P3 = jointData.Pose.Pose.Orientation.Z;
    P4 = jointData.Twist.Twist.Angular.X;
    P5 = jointData.Twist.Twist.Angular.Y;
    P6 = jointData.Twist.Twist.Angular.Z;

    if P1<0.001
        P1 = 0;
    end

    if P2<0.001
        P2 = 0;
    end
    
    if P3<0.001
        P3 = 0;
    end
        
    if P4<0.001
        P4 = 0;
    end
    
    if P5<0.001
        P5 = 0;
    end
    
    if P6<0.001
        P6 = 0;
    end
    

    X = [X1 X2]' - [10 ;0];

    P = [P1 P2 P3 P4 P5 P6]' - [0 0 0 0 0 0];
    Q = [P1 P2 P3 P4 P5 P6]';


    F1 = K1*X;

   


   

    vel = double(sqrt((m*g-m*F1)/b*cos(P1)*cos(P2)));
    tau1.AngularVelocities = [vel/4,vel/4,vel/4,vel/4];
    send(j1_effort,tau1);


    if (time_step > 11)
        prev = t;
        A2 = [0 0 0 1 0 0;
              0 0 0 0 1 0;
              0 0 0 0 0 1;
              0 0 0 0 0 0;
              0 0 0 0 0 0;
              0 0 0 0 0 0];

%         vel = 650;
        vel = vel*0.06;
        
        x_t = (2*b)*(l/Ix)*vel;
        y_t = (2*b)*(l/Iy)*vel;
        z_t = (2*d)*(l/Iz)*vel;
        
        
        B2 = [0 0 0 0;0 0 0 0;0 0 0 0;0 -x_t 0 x_t;-y_t 0 y_t 0;-z_t z_t -z_t z_t];
        Q2 = eye(6)*1;
        R2 = eye(4)*0.01;
        
        K2 = lqr(A2,B2,Q2,R2);
        
        F2 = K2*P;
        
        u1 = F2(1,1);
        u2 = F2(2,1);
        u3 = F2(3,1);
        u4 = F2(4,1);
        
       
        
        
        o1 = double(sqrt(abs((u1/(4*b)) + (u3/(2*l*b)) + (u4/(4*d)))));
        o2 = double(sqrt(abs((u1/(4*b)) - (u2/(2*l*b)) - (u4/(4*d)))));
        o3 = double(sqrt(abs((u1/(4*b)) - (u3/(2*l*b)) + (u4/(4*d)))));
        o4 = double(sqrt(abs((u1/(4*b)) + (u2/(2*l*b)) - (u4/(4*d)))));

        o1 = o1 * 9.5;
        o2 = o2 * 9.5;
        o3 = o3 * 9.5;
        o4 = o4 * 9.5;
        
%         if o1>1000
%             o1 = 650;
%         end
%         if o2>1000
%             o2 = 650;
%         end
%         if o3>1000
%             o3 = 650;
%         end
%         if o4>1000
%             o4 = 650;
%         end
        
        
        tau1.AngularVelocities = [01,02,03,04];
        send(j1_effort,tau1);
    
    
    
    end

    % you can sample data here to plot at the end
    Pos_data(counter) = P1;
    Vel_data(counter) = P2;
    Vel_input(counter) = P3;
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
