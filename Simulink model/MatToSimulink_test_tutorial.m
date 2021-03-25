%test  of how to interact with simulink model from a matlab script

clear 
clc
close all

%Define model constant 
g = [0;0;9.81];
m = 1.2;

I = eye(3); % inertia of the drone


% Running the simulink model command
sim('Quadrotor_model')

%you can use "output" blocks to send data back to matlab script starting
%from simulink model
% extract data coming form simulink model (time history and time information)

comingoutDataSignal = yout.getElement('name of the variable coming out from the simulink model');
time_comingoutData = comingoutDataSignal.Value.Time;
comingoutData = comingoutDataSignal.Value.Data;


%% Bebop2 parameters 2 body vertical model
clear 
clc
close all


m1 = 0.3;
m2 = 0.2; 


deltax0 = 0.025;
g = 9.81;

k = 1.1 * m2 *g / (deltax0 * 0.2);
c = 2 * sqrt(k*m2) * 0.8;


omega_motor = 50;
x1_0 = 0;
x2_0 = deltax0 - m2*g/k;

if x2_0 < 0
    error('negative initial delta position ')
end

h_des = 2;

open('bebop_vert')
sim('bebop_vert')


%% bebop 2 full 3D model


clear 
clc
close all

g = 9.81;

mb = 0.3;
mc = 0.2;

omega_c_propeller = 50;

Ixxb = 10;
Iyyb = 10;
Izzb = 10;

Ib = diag([Ixxb, Iyyb, Izzb]); % includes motor position
iIb =  Ib \ eye(3);

% motor quantities
% motor layout
% 
%    4          1
%     \        /
%      \      /
%       \____/
%       |    |
%       |____|
%      /      \
%     /        \
%    /          \
%   3            2


l = 0.08; % [m] motor excursion along x-body axis
b = 0.1;  % [m] motor excursion along y-body axis
k_t = 0.01; % [N*s/rad] thrust constant
k_tau = 0.05; % [ N*m*s/rad] torque constant

ld = 0.052; % [m] damper excursion along x-body axis (for both bodies - motor frame & camera)
bd = 0.011; % [m] damper excursion along y-body axis (for both bodies - motor frame & camera)
delta_h_damper = 0.025; % [m] damper excursion along z-body axis (for both bodies - motor frame)
M_c = [b*k_t; l*k_t; k_tau] .* [-1 1 1 -1; 1 1 -1 -1; 1 -1 1 -1];

Body_pos_damper = [ld; bd; 0] .* [1, -1, -1, 1;...
                                  1, 1, -1, -1;...
                                  0,0,0,0];
delta_l0_body_frame = [ld; bd; delta_h_damper] .* [1,-1,-1, 1;...
                                                   1, 1,-1,-1;
                                                   1, 1, 1, 1];

Elastic_matrix_damper = eye(3) .* 100; % [N/m]
Damping_matrix_damper = eye(3) .* 1; % [N * s/m]

open('bebop_full')
sim('bebop_full')


%% SYSTEM DYNAMIC TESTS
clear 
clc
close all

m = 0.5;




m1 = 0.3;
m2 = 0.2; 
omega_c_propeller = 50;
deltax0 = 0.025;
g = 9.81;

k = 1.1 * m2 *g / (deltax0 * 0.2);

c = 2 * sqrt(k*m2) * 0.8;

A = [0,0,1,0,0;...
     0,0,0,1,0;...
     k/m1, -k/m1, c/m1, -c/m1, 1/m1;...
     -k/m2, k/m2, -c/m2, c/m2, 0;...
     0,0,0,0, -omega_c_propeller];
 
B = [0,0,0,0,omega_c_propeller]';
C = [0,1,0,0,0];
D = 0;

PID = tf(10,1) + tf(1, [1,1]) + tf ([10000,0], [1,10000]);


% TESTING/TUNING SPRING DAMPER PARAMETERS
% b1gdl = c/m2;
% c1gdl = k/m2;
% omega2 = sqrt(c1gdl);
% sm = b1gdl / (2 * omega2);
% test = tf([c1gdl], [1,b1gdl, c1gdl]);
% 
% figure
% bode(test)
% figure
% pzmap(test)
% grid on



syst = ss(A,B,C,D);

syst_f = feedback(syst, -500);

figure
pzmap(feedback(PID * syst, 1))

figure
bode(syst)
hold on
bode(syst_f)
grid on
legend('open loop', 'close loop')

figure
pzmap(syst)
hold on
pzmap(syst_f)
legend('open loop', 'close loop')


figure
rlocus(syst)
hold on
rlocus(syst_f)
legend('open loop', 'close loop')

