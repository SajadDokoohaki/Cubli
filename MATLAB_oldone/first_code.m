clear all ; 
close all ; 
clc ; 
%% Constant Specification
 mb = 0.419 ; %% kg : pendulum body  
 mw = 0.204 ; %% kg : wheel masses
 l = 0.085 ; %% m : Distance between the motor axis and the pivot point
 lb = 0.075 ; %% m  : Distance between the center of amss of the pendulum body and the pivot point 
 Ib = 3.34e-3 ; %% kg.m2 : Moment of inertia of the pendulum body 
 Iw = 0.57e-3 ; %% kg.m2 : Moment of inertia of the wheel and the motor rotor around the rotational axis of the motor
 Cb = 1.02e-3 ; %% kg.m2.s-1 : Dynamic friction coefficient of the wheel 
 Cw = 0.05e-3 ; %% kg.m2.s-1 : Dynamic friction coefficient of the pendulum body 
 %%Tm ; %% N.m : Torque pruduced by the Motor
 Km = 25.1e-3 ; %% N.m.A-1 : Torque constant of the brushless DC motor 
 %%u  ; %% A : Current input
 g = 9.81 ; %% m.s-2 : Gravitational Constant
 dh = 0.5 ; %% N.m : Disturbant Torque
% syms thetab ; %% rad : Angular velocity estimate of the body from the rate gyro
% syms thetaw ; %% rad : Angular velocity estimate of the momentum wheel from the hall sensors
% syms thetabf_d ; %% rad : First derivative of thetab
% syms thetawf_d ; %% rad : First derivative of thetaw
% syms thetabs_d ; %% rad : Second derivative of thetab
% syms thetaws_d ; %% rad : Second derivative of thetaw

%  set_param('preliminary_Model' , 'mb' ,'0.419' , 'mw' ,'0.204' , 'lb','0.075' ,...
%            'l' ,'0.085', 'Ib' ,'3.34e-3' , 'Iw','0.57e-3' , 'Cb' ,'1.02e-3' ,...
%            'Cw','0.05e-3' ,'g' ,'9.81','Km','25.1e-3') ;
% 
%  set_param(model,'SimulationCommand','Update') ;
% 
% tFinal = 5 ; 
% t_in = [0 , tFinal , 100] ; 
% 
% U_in = zeros(size(t_in)) ; 
% 
% for k = length(t_in) 
%     if (t_in(k) < 10) 
%         U_in(k) = t_in(k) ;
%     else
%         U_in(k) = sin(t_in(k)) ;
%     end
% end
% 
% 
% simT = [mb, mw, l , lb  , Ib , Iw , Cb , Cw , Km , g , U_in] ; 
% 



%% Signification
close all ; 
A = [               0                ,        1        ,        0       ;
    (mb*lb + mw*l)*g/(Ib + mw*l^2) , -Cb/(Ib+mw*l^2) , Cw/(Ib+mw*l^2) ;
    -(mb*lb + mw*l)*g/(Ib+mw*l^2) , Cb/(Ib+mw*l^2) , -Cw*(Ib + Iw + mw*l^2)/(Iw*(Ib+mw*l^2)) ];

B = [ 0 ; -Km/(Ib+mw*l^2) ; Km*(Ib + Iw + mw*l^2)/(Iw*(Ib + mw*l^2)) ] ;

C = [1 0 0; 0 1 0; 0 0 1];

D = [0; 0; 0];


s = tf('s') ;

G = C*inv(s*eye(3) - A)*B + D

% figure ;
% 
% bode(G(1)) ;
% 
% title('Bode Diagram of $\theta_b$', 'Interpreter' , 'latex');
% 
% 
% figure ;
% 
% bode(G(2)) ;
% 
% title('Bode Diagram of $\dot{\theta}_b$', 'Interpreter' , 'latex')
% 
% 
% figure ;
% 
% bode(G(3)) ;
% 
% title('Bode Diagram of $\dot{\theta}_w$', 'Interpreter' , 'latex')
% 
% 
timeinterval = 0:0.0002:5 ;
c = step(G(1),timeinterval);

figure;

plot(timeinterval, c);

title('Step-response of $\theta_b$','Interpreter' , 'latex')


% Defining our PID controller 

PID1 = 21 + 59/s + 0.66 * (1925 / (1 + 1925 /s)) ;

c2 = step( G(1)*PID1 / ( G(1)* PID1 + 1) , timeinterval )  



% Root Locus of G Matrix :
figure
rlocus(G(1)* PID1 /(1 + G(1) * PID1)) ;
title('Root-Locus of  $\theta_b$','Interpreter' , 'latex')

figure 
rlocus(G(2)) ; 
title('Root-Locus of $\dot{\theta}_b$','Interpreter' , 'latex')

figure 
rlocus(G(3)) ; 
title('Root-Locus of $\dot{\theta}_w$','Interpreter' , 'latex')


%  dt = 0.02 ; % sampling time
%  t = 0:0.02:39.98 ; % total time
%
%   u = sin(t)  ;
%x0 = [ pi/4 ; 0 ; 0 ] ;
% %  y = [ pi/4 ; 0 ; 0 ] ;
% %  for k = 1:1999
% %      y(k) = C*x + D*u(k) ;
% %      x = A*x + B*u(k) ;
% %  end
%
%
% [b,a] = ss2tf(A,B,C,D)
% thetab = tf(b(1,:),a)
% thetab_dot = tf(b(2,:),a)
% thetaw_dot = tf(b(3,:),a)
% b
% a
% yt = filter(b(1,:),a,u);
% stem(t,yt,'filled')
% xlabel('t')

%% analytical approach 
% Q = (Ib/Iw + 1 + mw*l^2/Iw); 
% M = (mb*lb + mw*l)*g ; 
% N = Ib + mw*l^2 ; 
% s = tf('s') ; 
% syms thetab_s ;
% syms thetaw_s ; 
% % syms U_s ; 
% % syms U_s ;
% U_s = 1/s ;
% thetaw_s = ( Q*Km/N*U_s +M/N*(Cb*s-g) - M*Cb*pi/(4*N) )/(s*(s-Q*Cw/N)) ; 
% thetab_s = ( pi/4*s + Cb*pi/4 + Cw*s*thetaw_s - Km*U_s )/(s^2 + Cb*s - M/N) 
% G_s = thetab_s / U_s 

%% Another way 
% ltiSys = ss(A,B,C,D) ; 
% sys = tf(ltiSys) 

%% another way 
% s = tf('s') ; 
% F = C*inv(s*eye(3)-A)*B ; 
