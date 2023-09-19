clc;
clear;
%% data normal configuration

Ixx=0.6465;
Iyy=0.3048;
Izz=0.9359;
Ixz=0.0006584;

gravity=9.81;

%% Airplane Properties
m=4; %kg
S=0.471; %Wing Area m^2
b=1.7; %Wing Span m
c=0.28; %Mean Wing Chord m
CX0=-0.0094304;
CXu=-(0.158*0)-0.0094304;
CXa=(0.42565-0.21544);
CXe=0;
CXt=1;

%% ---------------- Y force--------------------
CYb=-0.16533;
CYp=-0.00068802; %sign change
CYr=0.15502; %sign Charge
CYru=-0.10317; %sign Change12

%% ---------------- Z force--------------------
CZ0=-0.42565;
CZu=-((0.052968^2)/(1-(0.052968)^2))*0.42565;
CZa=-(4.8375+0.0094304);
CZa_d=1.3359; %lt/c=2.5
CZq=-7.3583; %-(-9.96/2.5)*2*u0/c;
CZe=-0.56118;
CZt=0;

%% ---------------- L Moment--------------------
CLb=0.029196; %signChange2
CLp=-0.44965;
CLr=0.0067709;
CLru=0.003592;
CLai=0.32935; %SignChange2

%% ---------------- M Moment--------------------
CMu=0.158*0;
CMa=-0.1651;
CMa_d=-6.5034;
CMq=-11.7782;
CMe=-1.3977;
CMt=0;

%% ---------------- N Moment--------------------
CNb=0.066996; %sign Change
CNp=-0.030754;
CNr=-0.064937;
CNru=-0.045652;
CNai=0.036215; %SignChange2



%% Assume
V_tot=287;
theta0=5.1*pi/180; %in rad
V0=18;
rho=1.225; %kg/m^3
Q=1/2*rho*V0^2;
u0=sqrt(V0^2/(1+(tan(theta0))^2));
w0=u0*tan(theta0);
q0=0;
alpha0=2.3;

d_elevator0=-0.6319*pi/180;
d_Thrust0=5;
u=[d_elevator0 d_Thrust0] ;
w_d0=0;

%% table2
Xu=(Q*S/u0)*(2*CX0+CXu)*u0;
Xw=(Q*S/u0)*CXa*w0;
Xde=S*CXe*d_elevator0;%Xds
Xdth=S*CXt*d_Thrust0;

Zu=(Q*S/u0)*(2*CZ0+CZu)*u0;
Zw=S*CZa*w0/u0;
Zwd=c/(2*u0)*S*CZa_d* w_d0/u0;
Zq=S*c/(2*u0)*CZq*q0;
Zde=S*CZe*d_elevator0;
Zdth=S*CZe*d_Thrust0;

Mu=S*c*CMu*Iyy;
Mw=S*c*CMa*w0;
Mwd=S*c^2/(2*u0^2)*CMa_d*w_d0;
Mq=S*c^2/(2*u0)*CMq*q0;
Mde=S*c*CMe*d_elevator0;
Mdth=S*c*CMt*d_Thrust0;




%%  longitudinal
A_long_full=[Xu                     Xw                    -w0                        -gravity*cos(theta0)
    Zu/(1-Zwd)             Zw/(1-Zwd)            (Zq+u0)/(1-Zwd)            (-gravity*sin(theta0))/(1-Zwd)
    Mu+(Mwd*Zu)/(1-Zwd)    Mw+(Mwd*Zw)/(1-Zwd)   Mq+(Mwd*(Zu+u0))/(1-Zwd)   (-Mwd*gravity*sin(theta0))/(1-Zwd)
    0                      0                     1                           0];
B_long_full=[Xde                         Xdth
    Zde/(1-Zwd)                 Zdth/(1-Zwd)
    Mde+(Mwd*Zde)/(1-Zwd)       Mdth+(Mwd*Zdth)/(1-Zwd)
    0                           0];
C_long_full=eye(4);
D_long_full=zeros(4,2);
long_full_state_space= ss(A_long_full,B_long_full,C_long_full,D_long_full);
[V_long_full,D_long_full] = eig(A_long_full);
[d_long_full,ind_long_full]=sort(diag(D_long_full));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %% short period
A_short_period=[Zw/(1-Zwd)           (Zq+u0)/(1-Zwd)
    Mw+(Mwd*Zw)/(1-Zwd)   Mq+(Mwd*(Zu+u0))/(1-Zwd)];
B_short_period=[ Zde/(1-Zwd)          Zdth/(1-Zwd)
    Mde+(Mwd*Zde)/(1-Zwd) Mdth+(Mwd*Zdth)/(1-Zwd)];
C_short_period=[0 1];
D_short_period=[0 0];
short_period_state_space=ss(A_short_period,B_short_period,C_short_period,D_short_period);
[V_short_period,D_short_period] = eig(A_short_period);
[d_short_period,ind_short_period]=sort(diag(D_short_period));

disp('Short Period Appromiation System Info');
%damp(A_short_period);
S_short_period=stepinfo(short_period_state_space(1))


% r = pid(700,500,0,0);          %pid(Kp,Ki,Kd,Tf)
% response = getPIDLoopResponse(r,short_period_state_space(1),'closed-loop');   %getPIDLoopResponse(C,G,looptype)
% figure;
% step(short_period_state_space(1),response);
% legend("Natural Response","PID Response");
% disp('Short Period Appromiation System Info after PID');
% stepinfo(response)

for i=100:100:1000
    for j=100:100:1000
        r = pid(i,j,0,0);          %pid(Kp,Ki,Kd,Tf)
        response = getPIDLoopResponse(r,short_period_state_space(1),'closed-loop');   %getPIDLoopResponse(C,G,looptype)
        %figure;
        %step(short_period_state_space(1),response);
        %legend("Natural Response","PID Response");
        disp('Short Period Appromiation System Info after PID');
        stepinfo(response)
    end
end

% figure;
% step(short_period_state_space(1));
% title('Short Period Response to Step Input');


% figure;
% x0=[u0;theta0]; %%%%%%%%%%%%%
% initial(short_period_state_space(1),x0)
% title('Short Period Response to Initial Conditions');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% transfer function
%long full
long_full_tf=tf(long_full_state_space);
long_full_u_de=      long_full_tf(1,1);
long_full_w_de=      long_full_tf(2,1);
long_full_alpha_de=  long_full_w_de/u0;
long_full_q_de=      long_full_tf(3,1);
long_full_theta_de=  long_full_tf(4,1);
long_full_u_dT=      long_full_tf(1,2);
long_full_w_dT=      long_full_tf(2,2);
long_full_alpha_dT=  long_full_w_dT/u0;
long_full_q_dT=      long_full_tf(3,2);
long_full_theta_dT=  long_full_tf(4,2);


%short period
%  short_period_tf=tf(short_period_state_space);
%  short_period_u_de=    short_period_tf(1,1);
%  short_period_theta_de=short_period_tf(2,1);
%  short_period_u_dT=    short_period_tf(1,2);
%  short_period_theta_dT=short_period_tf(2,2);

