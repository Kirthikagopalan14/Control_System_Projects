%%
pause(3);
clear all;
close all;
clc;

addpath(genpath('CLSS Praxis'),genpath('hudaqlib'))
dev = HudaqDevice('MF634');
mn = 25;
mkdir Messung_25;


s = 2000;  %total experiment sample
Ts = 0.02; % sampling period
time=40;

x = zeros(s,4); %states during experiment
z2 = zeros(4,1);% sample states
x(1,:) = [(AIRead(dev,1)/0.15/100) 0.01*round(-13.1*AIRead(dev,3)) (-AIRead(dev,2)/0.96*pi/180) 0]; % inital values
Force = zeros(s,1); 


%%%%%%%%% your part
q   = zeros(2,s); % position and angle
qd  = zeros(2,s); % linear and angular velocity
qdd = zeros(2,s); % linear and angular acceleration
q(:,1) = [0.4; -0.1]; % inital values for position and angle
tau   = zeros(1,time/Ts); %voltage
% Time vector
time_vec = 0:Ts:time - Ts;
% Define a desired cart position trajectory (e.g., sinusoidal or any other desired path)
cart_position_ref = square(0.1*pi*5*time_vec) * 0.1;  % Sinusoidal reference for cart position
pendulum_angle_ref = 0;  % Pendulum stays at upright position (angle = 0)
%q_r(1) = 0;  % reference value of position
q_r(2) = 0;  % reference value of angle

qd_r = zeros(2,1); % reference value of linear and angular velocity

%% State-space
%%
m_p     = 0.329;m_w     = 3.2;l_sp    = 0.44;f_w     = 6.2; 
f_p     = 0.009;gra       = 9.81;j_a     = 0.072;Ts = 0.02; 

A_c = [ 0   1                               0                   0
        0   -f_w/(m_w+m_p)                  0                   0
        0   0                               0                   1
        0   (f_w*m_p*l_sp)/(j_a*(m_w+m_p)) (m_p*l_sp*gra)/j_a     -f_p/j_a];   
B_c = [0  ;   1/(m_w+m_p) ;   0   ;   -m_p*l_sp/((m_w+m_p)*j_a)];
C_c = [   1   0   0   0
        0   1   0   0
        0   0   1   0
        0   0   0   1];
D_c = [0;0;0;0];

sys_cont = ss(A_c,B_c,C_c,D_c);
sys_d = c2d(sys_cont,Ts);

A = sys_d.A;B = sys_d.B;C = sys_d.C;D = sys_d.D;

%% Implementation

%%%%%%%%%%%%%%%%%%%%%
Q = diag([100, 100, 100, 100]); % State weight matrix
R = 0.01;                    % Input weight scalar
K = lqr(A_c, B_c, Q, R);    % Compute LQR gain for continuous system
x_r = zeros(s,4);
x_r(:,1) = cart_position_ref;

for i = 1:s-1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  CONTROL ALGORITHM
    % Reference state (desired position, velocity, angle, angular velocity)
%     x_r = [cart_position_ref(i); 0; pendulum_angle_ref; 0];

    %  CONTROL ALGORITHM
    % Full state vector: [position, velocity, angle, angular velocity]
    %x(:,i) = [q(1,i); qd(1,i); q(2,i); qd(2,i)];
    
    % Error state
    e = x(i,:)' - x_r(i,:)';

    % Control Input using LQR
    Force(i) = -K * e; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   

   
    % save the data
    z2(1) = AIRead(dev,1)/0.15/100; % position of cart (meter)
    z2(2) = 0.01*round(-13.1*AIRead(dev,3)); % speed of cart (m/s)
    z2(3) = -AIRead(dev,2)/0.96*pi/180; % angle of pendulum (radian)

    
%%%%%%%%%%%%%%%%%%%%%% voltage limitation
    if abs(Force(i))>10
       Force(i) = sign(Force(i))*10; 
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%%%%%%%%%%%% apply calculated voltage    
    tic
    while toc<Ts
        DOWriteBit(dev,1,2,1)           % Freischaltung Pendel
        DOWriteBit(dev,1,2,0)           % channel 1 besteht aus DO0..DO7
        DOWriteBit(dev,1,2,1)           % DO2 benötigt kontinuierlichen Impuls
        AOWrite(dev, 2, Force(i));      % apply calculated voltage
    end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
 
 %%%%%% angular speed calculation (derivative) 
    z2_winkel = -AIRead(dev,2)/0.96*pi/180;
    z2(4) = (z2_winkel-z2(3))/Ts; % angular speed of pendulum
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    


      
 
    
    
    x(i+1,:) = z2';
    
    if abs(z2(1)) > 0.3 || abs(z2(3)*180/pi) > 10    % Der Pendel ist ausser Bereich
        disp('Please bring me back !');
        pause(3);                 % wait 3 second
    end
    
end

%% Results Plotting
figure;
sgtitle('Iterative LQR Results for sine trajectory');

% Plot cart position
subplot(3, 1, 1);
plot(time_vec, x_r(:,1), 'r--', 'LineWidth', 1.5); hold on;
plot(time_vec, x(:,1), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Position (m)');
title('Cart Position (Reference vs Actual)');
legend('Reference Position', 'Actual Cart Position', 'Location', 'Best');

% Plot pendulum angle
subplot(3, 1, 2);
plot(time_vec, rad2deg(x(:,3)), 'r-', 'LineWidth', 1.5); % Convert angle to degrees
grid on;
xlabel('Time (s)');
ylabel('Angle (\circ)');
title('Pendulum Angle');

% Plot control input
subplot(3, 1, 3);
plot(time_vec, Force, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Input (V)');
title('Control Input vs. Time');
grid on;