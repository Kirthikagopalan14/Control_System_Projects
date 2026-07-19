clear all;
clc
%% Parameters and Initialization
Ts = 0.02; % sampling Time
time = 20; % total simulation time
alpha = 6;  
beta = 1.8; % variable that changes the speed of switching resp the duty cycle
var_k = 10;

q   = zeros(2,time/Ts); % position and angle
qd  = zeros(2,time/Ts); % linear and angular velocity
qdd = zeros(2,time/Ts); % linear and angular acceleration
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

%% pendulum parameters

KF=2.6;M0=3.2;M1=0.329;M=M0+M1;ls=0.44;inert=0.072;N_val=0.1446;
N01_sq=0.23315;Fr=6.2;C=0.009;gra=9.81;

a32 = -N_val^2/N01_sq*gra ; a33 = -inert*Fr/N01_sq; a34 = N_val*C/N01_sq; 
a35 = inert*N_val/N01_sq; a42 = M*N_val*gra/N01_sq; a43 = N_val*Fr/N01_sq; a44 = -M*C/N01_sq;
a45 = -N_val^2/N01_sq; b3=inert/N01_sq; b4=-N_val/N01_sq;
b3_hat = inert/N01_sq+0.1;b4_hat = -N_val/N01_sq+0.1;

%% Control
%% LQR Design
Q = diag([100, 100, 100, 100]); % State weight matrix
R = 0.01;                    % Input weight scalar
K = lqr(A_c, B_c, Q, R);    % Compute LQR gain for continuous system
x_r = zeros(4, time/Ts);
x_r(1,:) = cart_position_ref;

for k = 1:time/Ts
    % Reference state (desired position, velocity, angle, angular velocity)
%     x_r = [cart_position_ref(k); 0; pendulum_angle_ref; 0];

    %  CONTROL ALGORITHM
    % Full state vector: [position, velocity, angle, angular velocity]
    x(:,k) = [q(1,k); qd(1,k); q(2,k); qd(2,k)];
    
    % Error state
    e = x(:,k) - x_r(:,k);

    % Control Input using LQR
    tau(:,k) = -K * e; 

%%%%%%%%%% voltage limitation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if abs(tau(:,k))>10
        tau(:,k) = sign(tau(:,k))*10;
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%% inverted pendulum math. model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    beta_x2 = (1+N_val^2/N01_sq*(sin(q(2,k)))^2)^(-1);
    qdd(:,k+1) = [beta_x2*(a32*sin(q(2,k))*cos(q(2,k))+a33*qd(1,k)+...
                a34*cos(q(2,k))*(qd(2,k))+a35*sin(q(2,k))*qd(2,k)^2+b3*tau(:,k));
                 beta_x2*(a42*sin(q(2,k))+a43*cos(q(2,k))*qd(1,k)+...
                a44*(qd(2,k))+a45*cos(q(2,k))*sin(q(2,k))*(qd(2,k))^2+b4*cos(q(2,k))*tau(:,k))];

    qd(:,k+1) = qd(:,k) + qdd(:,k+1)*Ts;        
    q(:,k+1) = q(:,k) + qd(:,k+1)*Ts;
    q(2,k+1) = mod(q(2,k+1)+pi,2*pi)-pi;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    error_deviation(k) = cart_position_ref(k) - x(1, k);
end

%% Results Plotting
figure;
sgtitle('LQR Results for sine trajectory');

% Plot cart position
subplot(3, 1, 1);
plot(time_vec, x_r(1,:), 'r--', 'LineWidth', 1.5); hold on;
plot(time_vec, x(1, :), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Position (m)');
title('Cart Position (Reference vs Actual)');
legend('Reference Position', 'Actual Cart Position', 'Location', 'Best');

% Plot pendulum angle
subplot(3, 1, 2);
plot(time_vec, rad2deg(x(3, :)), 'r-', 'LineWidth', 1.5); % Convert angle to degrees
grid on;
xlabel('Time (s)');
ylabel('Angle (\circ)');
title('Pendulum Angle');

% Plot control input
subplot(3, 1, 3);
plot(time_vec, tau, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Input (V)');
title('Control Input vs. Time');
grid on;
