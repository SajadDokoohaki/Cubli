%%
clear all;
close all;
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

num_1 = ( vars.m_b * vars.l_b + vars.m_w * vars.l) * vars.g ;
num_2 = vars.I_b + vars.I_w + vars.m_w * vars.l^2 ;
den_1 = vars.I_b + vars.m_w * vars.l^2 ;


A = [                0,                     1,                                         0 ;
    num_1 / den_1,     -vars.C_b / den_1,                          vars.C_w / den_1 ;
    -num_1 / den_1,      vars.C_b / den_1,      -vars.C_w * num_2/ ( vars.I_w * den_1) ];

B = [                                       0;
    -vars.K_m / den_1;
    vars.K_m * num_2 / ( vars.I_w * den_1) ];

C = [   1,  0,  0];

D = 0 ;
%% This section computes the transfer fuction G(s) = Y(s)/R(s)
[NUM, DEN] = ss2tf(A,B,C,D);

NUM_new = round(NUM, 4);
DEN_new = round(DEN, 4);

%H =  tf(NUM_new, DEN_new)


% syms s ;

% NUM_new_s = NUM_new(1) * s^3 + NUM_new(2) * s^2 + NUM_new(3) * s^1 + NUM_new(4) ;
% DEN_new_s = DEN_new(1) * s^3 + DEN_new(2) * s^2 + DEN_new(3) * s^1 + DEN_new(4) ;

% H_t = NUM_new_s / DEN_new_s

% [Y, T] = step(H) ;
% plot(T, Y)
% cla;
% rlocus(H)
%
% figure;
% title('H(s)');

%% Try ...
% computing the system and system response to initial condition

H_new = ss(A,B,C,D);

x0 = [0; 0; pi/3;0] ;
% input("Please Enter the initial condition in this form [thetab, thetabdot, thetawdot /n");

%_____________________________________________________________
% Controller Design (PI, PD, PID)
% PID form : (P + I * 1/s + D* N /( 1+ N/s))

% PI Controller Design
N = 1000;
Tf = 1/N;
D = 0;

tic
for i=1:4

    for j=1:4

        %subplot(4,4,j+(i-1)*4)
        subplot('Position', [mod(j-1,4)*0.24+0.05, floor((i-1)/4)*0.24+0.05, 0.2, 0.18])

        for P = -10^(i-1) : 10^(i-2) : -10^(i-2)

            for I = 10^(j-2) : 10^(j-2) : 10^(j-1)

                 G_c = pid(P, I, D, Tf);

                sys_mul = series(H_new, G_c) ;

                initial(sys_mul, x0) ;

                hold on;

            end

        end



        title({[num2str(-10^(i-1)),'$<P<$',num2str(-10^(i-2))] ...
            ,[num2str(-10^(j-1)),'$<I<$',num2str(-10^(j-2))]}, 'Color', 'Blue', 'Interpreter', 'latex', 'FontSize', 8);
        
        ax = gca;
       
        

    end

end      
toc

% for P = linspace(0.05,1,10)
% 
%     for I = linspace(0.5, 1, 10)
% 
%         %       NUM_i = [P, (P*N + D*N + I), I*N];
%         %       DEN_i = [1, N, 0];
%         %       [A2, B2, C2, D2] = tf2ss(conv(NUM_i,NUM_new), conv(DEN_i,DEN_new) );
%         %       G = ss(A2,B2,C2,D2);
% 
%         G_c = pid(P, I, D, Tf);
% 
%         sys_mul = series(H_new, G_c) ;
% 
%         initial(sys_mul, x0) ;
% 
%         hold on;
% 
%     end
% 
% end
% title('0.05 < P < 1')
% 
% subplot(2,2,2);
% 
% for P = linspace(1,10,10)
% 
%     for I = linspace(0.5, 1, 10)
% 
%         %       NUM_i = [P, (P*N + D*N + I), I*N];
%         %       DEN_i = [1, N, 0];
%         %       [A2, B2, C2, D2] = tf2ss(conv(NUM_i,NUM_new), conv(DEN_i,DEN_new) );
%         %       G = ss(A2,B2,C2,D2);
% 
%         G_c = pid(P, I, D, Tf);
% 
%         sys_mul = series(H_new, G_c) ;
% 
%         initial(sys_mul, x0) ;
% 
%         hold on;
% 
%     end
% 
% end
% title('1 < P < 10')
% 
% subplot(2,2,3);
% 
% for P = linspace(10,100,10)
% 
%     for I = linspace(0.5, 1, 10)
% 
%         %       NUM_i = [P, (P*N + D*N + I), I*N];
%         %       DEN_i = [1, N, 0];
%         %       [A2, B2, C2, D2] = tf2ss(conv(NUM_i,NUM_new), conv(DEN_i,DEN_new) );
%         %       G = ss(A2,B2,C2,D2);
% 
%         G_c = pid(P, I, D, Tf);
% 
%         sys_mul = series(H_new, G_c) ;
% 
%         initial(sys_mul, x0) ;
% 
%         hold on;
% 
%     end
% 
% end
% title('10 < P < 100')
% 
% subplot(2,2,4);
% for P = linspace(100,1000,10)
% 
%     for I = linspace(0.5, 1, 10)
% 
%         %       NUM_i = [P, (P*N + D*N + I), I*N];
%         %       DEN_i = [1, N, 0];
%         %       [A2, B2, C2, D2] = tf2ss(conv(NUM_i,NUM_new), conv(DEN_i,DEN_new) );
%         %       G = ss(A2,B2,C2,D2);
% 
%         G_c = pid(P, I, D, Tf);
% 
%         sys_mul = series(H_new, G_c) ;
% 
%         initial(sys_mul, x0) ;
% 
%         hold on;
% 
%     end
% 
% end
% title('100 < P < 1000')
% 
M = gcf;

exportgraphics(M, 'initial_response_V04.png', 'Resolution',600);



%% Contoller Design and Testing


%% Hand-Derived
theta_b_t0 = pi/100 ;
U_s = 0 ;
NUM_hd = [theta_b_t0, (0.7*theta_b_t0 - 207.93*vars.K_m*U_s), (21.2*theta_b_t0 - 5.55*vars.K_m*U_s) ];
DEN_hd = [1, 0.911, -120.34, 9859.5];
H2 = tf(NUM_hd, DEN_hd);
cla;
rlocus(H2)
title('H2(s)');
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


