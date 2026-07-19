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
cart_position_ref = 0.1 * sin(0.25 * time_vec);  % Sinusoidal reference for cart position
pendulum_angle_ref = 0;  % Pendulum stays at upright position (angle = 0)

% Cost weights
Q = diag([100, 0.1, 100, 0.1]); % State cost (penalizing cart position and pendulum angle errors heavily)
R = 0.001; % Control cost
Qn = 10000 * eye(4);
lambda = 1e-2; % Regularization term for Riccati recursion

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

%% iLQR Initialization
N = time / Ts; % Time horizon
u = zeros(1, N-1); % Initial control sequence
x_i = zeros(4, N); % State trajectory (position, velocity, angle, angular velocity)
x_i(:, 1) = [q(1, 1); 0; q(2, 1); 0]; % Initial state: [position; velocity; angle; angular velocity]
x_t = cell(10,1);

q_r = sin(0.5 * time_vec); % Sine trajectory for cart position
qd_r = [0; 0]; % Reference velocities
angle_ref = 0; % Target pendulum angle (radians)
% Desired state trajectory
x_r = [q_r; zeros(1, N); angle_ref * ones(1, N); zeros(1, N)]; % [cart position; velocity; pendulum angle; angular velocity]

%% LQR algorithm
K = lqr(A_c, B_c, Q, R);    % Compute LQR gain for continuous system

for k = 1:time/Ts

    %  CONTROL ALGORITHM
    % Full state vector: [position, velocity, angle, angular velocity]
    x_i(:,k) = [q(1,k); qd(1,k); q(2,k); qd(2,k)];
        
    % Error state
    e = x_i(:,k) - x_r(:,k);

    % Control Input using LQR
    tau(:,k) = -K * e; 

%%%%%%%%%% voltage limitation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if abs(tau(:,k))>10
        tau(:,k) = sign(tau(:,k))*10;
    end
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
end

%% Initial rollout->dynamics
xtraj = x_i;
utraj = tau;
J = cost(xtraj, x_r, utraj, N);

%% iLQR Algorithm
% Initialization
P = zeros(4, 4, N);         % Cost-to-go Hessian
p = zeros(4, N);            % Cost-to-go gradient
K = zeros(1, 4, N-1);       % Feedback gain
d = ones(1, N-1);           % Feedforward control
dJ = 0.0;

gx = zeros(1, 4);
gu = 0.0;
Gxx = zeros(4, 4);
Guu = 0.0;
Gxu = zeros(1, 4);
Gux = zeros(1, 4);

max_iter = 100;
for iter = 1:max_iter
      p(:, N) = Qn * (xtraj(:,N) - x_r(:,N));
    P(:, :, N) = Qn;
    % Backward pass
    for k = N-1:-1:1
        % Calculate Derivatives
        q = Q * (xtraj(:,k) - x_r(:, k));
        r = R * utraj(:,k);
        
        gx = q + A' * p(:,k+1);
        gu = r + B' * p(:,k+1);

        Gxx = Q + A' * P(:,:,k+1) * A;
        Guu = R + B' * P(:,:,k+1) * B;
        Gxu = A' * P(:,:,k+1) * B;
        Gux = B' * P(:,:,k+1) * A;
        
        d(:, k) = Guu \ gu;
        K(:, :, k) = Guu \ Gux;
        
        p(:, k) = gx - K(:, :, k)' * gu + K(:, :, k)' * Guu  * d(:, k) -  Gxu * d(:, k);
        P(:, :, k) = Gxx + K(:, :, k)' * Guu * K(:, :, k) - K(:, :, k)' * Gux - Gxu * K(:, :, k);
        
        dJ = dJ + gu' * d(:, k);

    end
    % Forward Pass
    xn = zeros(4,N);
    un = zeros(1,N-1);
    xn(:,1) = xtraj(:,1);
    alpha = 1.0;
    for k = 1:N-1
        % Control update with feedback and feedforward terms
        un(:, k) = utraj(:, k) - alpha * d(:, k) - K(:, :, k) * (xn(:, k) - x_r(:, k));
        xn(:, k+1) = A * xn(:, k) + B * un(:, k);

        % Control input saturation
        if abs(un(:, k)) > 10
            un(:, k) = sign(un(:, k)) * 10;
        end

    end
    Jn = cost(xn, x_r, un, N);

    % line search
    max_line_search_iters = 10; % Maximum number of line search iterations
    iter_count = 0;

    alpha_min = 1e-4; % Minimum allowable step size
    while Jn > (J - 1e-2 * alpha * dJ)
        alpha = 0.5 * alpha;
        iter_count = iter_count + 1;

        if alpha < alpha_min || iter_count > max_line_search_iters
            % disp('Exiting line search due to step size or iteration limit.');
            break;
        end

        for k = 1:N-1
            un(:, k) = utraj(:, k) - alpha * d(:, k) - K(:, :, k) * (xn(:, k) - x_r(:, k));
            xn(:, k+1) = A * xn(:, k) + B * un(:, k);
            
        end
        Jn = cost(xn, x_r, un, N);
    end
    % Convergence check: State trajectory difference
    state_diff = max(max(abs(xtraj(1,:) - x_r(1,:))));
    if state_diff < 1e-4
        disp(['Converged at iteration ', num2str(iter), ' with state trajectory difference: ', num2str(state_diff)]);
        break;
    end

    J = Jn;
    xtraj = xn;
    utraj = un;
    x_t{iter,1} = xn(1,:);
end
for i = 1:s-1
    x_r = [cart_position_ref(i); 0; pendulum_angle_ref; 0];
    e = x(i,:)' - x_r;
    Force(i) = -K(1,:,N) * e;
    
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
plot(time_vec, x_r(1,:), 'r--', 'LineWidth', 1.5); hold on;
plot(time_vec, xtraj(1, :), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Position (m)');
title('Cart Position (Reference vs Actual)');
legend('Reference Position', 'Actual Cart Position', 'Location', 'Best');

% Plot pendulum angle
subplot(3, 1, 2);
plot(time_vec, rad2deg(xtraj(3, :)), 'r-', 'LineWidth', 1.5); % Convert angle to degrees
grid on;
xlabel('Time (s)');
ylabel('Angle (\circ)');
title('Pendulum Angle');

% Plot control input
subplot(3, 1, 3);
plot(time_vec(1:end-1), utraj, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Input (V)');
title('Control Input vs. Time');
grid on;

%%
figure
plot(x_t{1,1}, 'k--', 'LineWidth', 3);
hold on
plot(x_t{max_iter/4,1}, 'k-', 'LineWidth', 1.5);
hold on
plot(x_t{0.5 * max_iter,1}, 'b-', 'LineWidth', 1.5);
hold on
plot(x_t{max_iter,1}, 'r-', 'LineWidth', 1.5);
hold on;
plot(x_r(1,:), 'g--', 'LineWidth', 1.5);

%% functions
% stage cost 
function [stagecost] = stage_cost(x, x_r, u)
    Q = diag([1, 0.1, 1, 0.1]); % State cost (penalizing cart position and pendulum angle errors heavily)
    R = 0.01; % Control cost

    stagecost = (0.5 *((x - x_r)' * Q * (x - x_r)) + 0.5 * R * u * u); 
end

% Terminal cost
function [terminal_cost] = terminal_cost(x, x_r)
    Qn = 10000 * eye(4);
    terminal_cost = 0.5 *((x - x_r)' * Qn * (x - x_r)); 
end

% overall cost
function [J] = cost(xtraj, x_r, utraj, N)
    J = 0;
    for k = 1:N-1
        J = J + stage_cost(xtraj(:,k), x_r(:,k), utraj(:,k));
    end
    J = J + terminal_cost(xtraj(:,k), x_r(:,k));
end

