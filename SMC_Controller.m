clear all
clc
[t,x]=ode45(@Quadrotor,[0,15],[0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]);
figure(1)
plot3(x(:,7),x(:,9),x(:,11),'LineWidth',2);
hold on
plot3(t,0*(t),sin(t),'--','LineWidth',2)
legend('Trajectory with SMC','DesiredTrajectory')
hold off
figure(2)
subplot(2,2,1)
plot(t,x(:,7),'LineWidth',2)
hold on
plot(t,t,'--','LineWidth',2)
subplot(2,2,2)
plot(t,x(:,9),'LineWidth',2)
hold on
plot(t,0,'--','LineWidth',2)
subplot(2,2,3)
plot(t,x(:,11),'LineWidth',2)
hold on
plot(t,sin(t),'--','LineWidth',2)






function dx = Quadrotor(t,x)
%Real Parameters of the Quadrotors
Kp=0.000029842; Kd=0.00003230; m=0.486; d=25e-2;g=9.81;
Kfa=[0.00055670;0.00055670;0.00063540];b=280.19;
Kft=[0.00055670;0.00055670;0.00063540];
J=[0.0038278;0.0038288;0.0076566];
Jr=0.000028385;
Beta=[189.63,6.0612,0.0122];
beta0=189.63;
beta1=6.0612;
beta2=0.0122;  %x=t z=sint
z=zeros(12,1);
xd=[0 0 0 0 0 0 t 1 0 0 sin(t) cos(t)];
xddot=[0 0 0 0 0 0 1 0 0 0 cos(t) -sin(t)];

    z(1)=xd(1)-x(1);
    z(2)=x(2)-xddot(1)-z(1);
    z(3)=xd(3)-x(3);
    z(4)=x(4)-xddot(3)-z(3);
    
    z(5)=xd(5)-x(5);
    z(6)=x(6)-xddot(5)-z(5);

    z(7)=xd(7)-x(7);
    z(8)=x(8)-xddot(7)-z(7);
    
    z(9)=xd(9)-x(9);
    z(10)=x(10)-xddot(9)-z(9);
    
    z(11)=xd(11)-x(11);
    z(12)=x(12)-xddot(11)-z(11);

Ix=3.827e-3;Iy=3.827e-3;Iz=7.65e-3;
%calculating the a and b values for the state space derivation
a1=(Iy-Iz)/Ix; a2=-Kfa(1)/Ix; a3=-Jr/Ix;
a4=(Iz-Ix)/Iy; a5=-Kfa(2)/Iy; a6=Jr/Iy;
a7=(Ix-Iy)/Iz; a8=-Kfa(3)/Iz; a9=-Kft(1)/m ;
a10=-Kft(2)/m; a11=-Kft(3)/m;

b1=d/Ix;       b2=d/Iy;        b3=1/Iz;

%Inputs
V1=z(1)^2/2;
V3=z(3)^2/2;
V2=(V1+z(2)^2)/2;
V4=(V3+z(4)^2)/2;

%Sliding Surfaces
S1=z(2);
S2=z(4);
S3=z(6);
S4=z(8);
S5=z(10);
S6=z(12);

% Synthezised stabilising laws
% Taking arbitrary values for K
OmegaBar=x(13)-x(14)+x(15)-x(16);
q1=1.5; q2=1.3; q3=1.2; q4=1.1;q5=1.4;q6=1;
k1=1.1; k2=1.2; k3=1.15; k4=1.08; k5=1.4; k6=.9;
U2=1/b1*(-sign(S1)-k1*S1-a1*x(4)*x(6)-a2*x(2)^2-a3*OmegaBar*x(4)+xddot(2)+(xd(2)-x(2)));
U3=1/b2*(-sign(S2)-k2*S2-a4*x(2)*x(6)-a5*x(4)^2-a6*OmegaBar*x(2)+xddot(4)+(xd(4)-x(4)));
U4=1/b3*(-sign(S3)-k3*S3-a7*x(2)*x(4)-a8*x(6)^2+xddot(6)+(xd(6)-x(6)));
U1=m/(cos(x(1))*cos(x(2)))*(-sign(S6)-k6*S6-a11*x(12)+xddot(12)+(xd(12)-x(12))+g);

Ux=m/U1*(-sign(S4)-k4*S4-a9*x(8)+xddot(8)+(xd(8)-x(8)));
Uy=m/U1*(-sign(S5)-k5*S5-a10*x(10)+xddot(10)+(xd(10)-x(10)));

        
x1dot=x(2);
x2dot=a1*x(4)*x(6)+a2*x(2)^2+a3*OmegaBar*x(4)+b1*U2;
x3dot=x(4);
x4dot=a4*x(2)*x(6)+a5*x(4)^2+a6*OmegaBar*x(2)+b2*U3;
x5dot=x(6);
x6dot=a7*x(2)*x(4)+a8*x(6)^2+b3*U4;
x7dot=x(8);
x8dot=a8*x(8)*x(4)+Ux*(U1/m);
x9dot=x(10);
x10dot=a10*x(10)+Uy*(U1/m);
x11dot=x(12);
x12dot=a11*x(12)+cos(x(1))*cos(x(3))*U1/m-g;
w1dot=b*V1-beta0-beta1*x(13)-beta2*x(13)^2;
w2dot=b*V2-beta0-beta1*x(14)-beta2*x(14)^2;
w3dot=b*V3-beta0-beta1*x(15)-beta2*x(15)^2;
w4dot=b*V4-beta0-beta1*x(16)-beta2*x(16)^2;
dx=[x1dot;x2dot;x3dot;x4dot;x5dot;x6dot;x7dot;x8dot;x9dot;x10dot;x11dot;x12dot;w1dot;w2dot;w3dot;w4dot];
end
