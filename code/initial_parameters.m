
clc ;
% This Script is for initiallizing paramteres and constants

vars = struct();
vars.l = 0.085 ; % in meters( m)
vars.l_b = 0.075 ; % in meters( m)
vars.m_b = 0.419 ; % in Kilograms( Kg)
vars.m_w = 0.204 ; % in Kilograms( Kg)
vars.I_b = 3.34e-3 ; % in Kilograms dot Square meters( Kg.m^2)
vars.I_w = 0.57e-3 ; % in Kilograms dot Square meters( Kg.m^2)
vars.C_b = 1.02e-3 ; % in Kilograms dot Square meters per second( Kg.m^2.s^(-1))
vars.C_w = 0.05e-3 ; % in Kilograms dot Square meters per second( Kg.m^2.s^(-1))
vars.g = 9.81 ; % in Kilograms dot meteres per square second( Kg.m.s^(-2))
vars.u = 0 ;
vars.K_m = 1 ; 
vars.T_m = vars.K_m * vars.u ;

nom_1 = ( vars.m_b * vars.l_b + vars.m_w * vars.l) * vars.g ;
nom_2 = vars.I_b + vars.I_w + vars.m_w * vars.l^2 ;
den_1 = vars.I_b + vars.m_w * vars.l^2 ;


A = [                0,                     1,                                         0 ;
         nom_1 / den_1,     -vars.C_b / den_1,                          vars.C_w / den_1 ;
        -nom_1 / den_1,      vars.C_b / den_1,      -vars.C_w * nom_2/ ( vars.I_w * den_1) ];

B = [                                       0;
                            -vars.K_m / den_1;
        vars.K_m * nom_2 / ( vars.I_w * den_1) ];

C = [   1,  0,  0];

D = 0 ;

[NUM, DEN] = ss2tf(A,B,C,D);
H =  tf(round(NUM,4), round(DEN,4))
K_p = 4.45;
rlocus(H)
figure;
%% Hand-Derived
theta_b_t0 = pi/100 ;
U_s = 0 ;
NUM_hd = [theta_b_t0, (0.7*theta_b_t0 - 207.93*vars.K_m*U_s), (21.2*theta_b_t0 - 5.55*vars.K_m*U_s) ];
DEN_hd = [1, 0.911, -120.34, 9859.5];
H2 = tf(NUM_hd, DEN_hd)
rlocus(H2)

% 
% P = -0.932846441737061 ;
% I = -1.2461120525887 ;
% D = -0.0983312267409919 ;
% N = 1928.74774944726 ;
% num1 = [P+N*D , P*N+I , I*N];
% den1 = [1 , N, 0];
% G_c = tf(num1,den1) ;
% G_s = ( G_c * H)/ (1 + G_c * H) ;
% 
% 
% t = 0:0.2:20; 
% 
% c = step( G_s, t);
% 
% plot(t,c,'-',t,t,'--');
% 


