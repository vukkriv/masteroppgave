%% This file simulates and tries to control the suspended pendulum system in 5DOF

% Set up some parameters
pL = 0.7;
pm_c = 2.5;
pm_L = 0.2;
pg = 9.81;
pd = 0.01;
epsilon = 0.001;

% Set Case Number!
cTracking = 1;
cBox      = 2;
cBoxShape = 3;


% which to run?
doCase = 3;

% Activate the plot at all
doInteractivePlot = 1;
doOnlyLastFrame = 0;


% Simulation Options
tend = 10;

% Control parameters
k1 = 1;
k2 = 2;
gamma_m = 500;

% Control program flow
constantRef = 1;

startAltitude = 0;
endAltitude = -2;
timeToMaxAltitude = tend-8;

% Use adaptive controller
useAdaptiveController = 0;

% Check if use reference model
useReferenceModel = 1;

% Use input shaper?
useInputShaperZVD = 0;
useInputShaperZV  = 0;

% Do Wps. % Only used in constant ref!
doWP = 1;
WPs = [0 1 0;
       1 1 -1; 
       1 0 -1; 
       0 0 0;]';
   
% WPs = [0 1 0;
%        1 1 0; 
%        1 0 0; 
%        0 0 0;]';   
   
if doCase == cTracking
    constantRef = 0;
    useReferenceModel = 1;
    % Use input shaper?
    useInputShaperZVD = 0;
    useInputShaperZV  = 0;
    
    tend = 20;
    timeToMaxAltitude = tend-8;
    
else
    constantRef = 1;
    useReferenceModel = 1;
    
    tend = 14;
    
    % Use input shaper?
    
    if doCase == cBoxShape
        useInputShaperZVD = 0;
        useInputShaperZV  = 0;
    end
end
    


% Initialize anon functions
addpath('generated/');
M = @(eta)(f_5dof_MassMatrix(eta, pL, pm_c, pm_L));
C = @(eta, nu)(f_5dof_CoreolisMatrix(eta, nu, pL, pm_c, pm_L));
G = @(eta)(f_5dof_Gravity(eta, pL, pm_c, pm_L, pg));
D = diag([0 0 0 pd pd]);
H = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 1 0 0];
 
 
 
% Restate for singularity avoidance
M = @(eta)(f_5dof_MassMatrix_singularity_avoidance(eta, pL, pm_c, pm_L, epsilon));
C = @(eta, nu)(f_5dof_CoreolisMatrix_singularity_avoidance(eta, nu, pL, pm_c, pm_L, epsilon));
 
 
M_hat = @(eta, m_L)(f_5dof_MassMatrix(eta, pL, pm_c, m_L));
C_hat = @(eta, nu, m_L)(f_5dof_CoreolisMatrix(eta, nu, pL, pm_c, m_L));
G_hat = @(eta, m_L)(f_5dof_Gravity(eta, pL, pm_c, m_L, pg));
 
M_L = @(eta)(f_5dof_MassMatrix(eta, pL, 0, 1));
C_L = @(eta, nu)(f_5dof_CoreolisMatrix(eta, nu, pL, 0, 1));
G_L = @(eta)(f_5dof_Gravity(eta, pL, 0, 1, pg));


 
K1 = diag([k1 k1 k1]);
K2 = diag([k2 k2 k2 k2 k2]);
 
% Simulation options

h = 0.05;

time = 0:h:tend;

% Initial conditions
eta_0 = [0, 0, 0, 0.00, 0.00]';
nu_0  = [0, 0, 0, 0, 0]';

% Desired positions.


desiredPos = [-4,1.5, 1]';

r   = zeros(3, length(time));
dr  = zeros(3, length(time));
d2r = zeros(3, length(time));



if constantRef
   r = repmat(desiredPos, 1, length(time));
   
   
   % Try to do convolution!
   

   
   
else
   f = 1.5;
   
   f = 3;
   
   altDiff = endAltitude - startAltitude;
   altFreq = altDiff/timeToMaxAltitude;
   
   % Create a nice spiral
   % Get altitude theta, as time up untin timeToMaxAltitude
   % Get Index:
   altMaxTimeIndex = find(time == timeToMaxAltitude);
   altTime = [time(1:altMaxTimeIndex) time(altMaxTimeIndex)*ones(1, length(time)-altMaxTimeIndex)];
   
   r =  2*pL*[cos(f*time); sin(f*time); altFreq*altTime];
   
   
   % Change so that we go to senter pos after that time
   r = [r(1:2, 1:altMaxTimeIndex) zeros(2, length(time)-altMaxTimeIndex);
        r(3, :)];
   
   r(1,:) = eta_0(1) + r(1,:);
   r(2,:) = eta_0(2) + r(2,:);
   r(3,:) = eta_0(3) + r(3,:);
   
   dr = 2*pL*f*[-sin(f*time); cos(f*time); zeros(1, length(time))];
   d2r = 2*pL*f^2*[-cos(f*time); -sin(f*time); zeros(1, length(time))];
end



% Initialize logger
addpath('helpers');
log = Logger();

log.init(time, h);
log.add('Eta', 5);
log.add('Nu', 5);
log.add('tau', 5);
log.add('tau_45', 2);
log.add('dNu', 5);
log.add('alpha', 5);
log.add('m_L_hat', 1);
log.add('r', 3);
log.add('r_preconv',3);
log.add('r_prerefmodel', 3);

% Store that reference before ref. model
log.set('r_prerefmodel', r);
% some options
m_L_min = 0.001;
m_L_max = 10;

% start simulation loop
eta = eta_0;
nu  = nu_0;
alpha_45 = [0; 0];
m_L_hat = 0.001;



% Use reference model in stead of reference derivatives
refModelState_x = [eta_0(1); 0; 0];
refModelState_y = [eta_0(2); 0; 0];
refModelState_z = [eta_0(3); 0; 0];

fprintf('*** Starting Simulation ***\n');
c_sec = tic;
c_total = tic;
reverseString = '';
step = 0;
t_sec = 0;


       
curWP = 1;
for k = 1:length(time)
   t = time(k);
   
   
   
   % Check if we should apply the reference model
   if useReferenceModel
        w0=2; % this a go starting point increase for faster decrease for slower
        zeta=1/sqrt(2);
        %zeta=0.9;

        Add=[   0 1 0 ;
                0 0 1;
                -w0^3 -(2*zeta+1)*w0^2 -(2*zeta+1)*w0];
        Bdd=[0 0 w0^3]';
        Cdd=eye(3);
        Ddd=[ 0 0 0]';
        
        f = @(x, u)(Add*x + Bdd*u);
        
        % Integrate 
        
        if doWP && constantRef
            refModelState_x = rk4(f, refModelState_x, WPs(1, curWP), h);
            refModelState_y = rk4(f, refModelState_y, WPs(2, curWP), h);
            refModelState_z = rk4(f, refModelState_z, WPs(3, curWP), h);
        else
            refModelState_x = rk4(f, refModelState_x, r(1, k), h);
            refModelState_y = rk4(f, refModelState_y, r(2, k), h);
            refModelState_z = rk4(f, refModelState_z, r(3, k), h); 
        end
        
        % Update desired positions
        r(1,k) = refModelState_x(1);
        r(2,k) = refModelState_y(1);
        r(3,k) = refModelState_z(1);
        
        dr(1,k) = refModelState_x(2);
        dr(2,k) = refModelState_y(2);
        dr(3,k) = refModelState_z(2);

        
        d2r(1,k) = refModelState_x(3);
        d2r(2,k) = refModelState_y(3);
        d2r(3,k) = refModelState_z(3);

   end
   
   if norm(r(:,k) - WPs(:,curWP)) < 0.1 && curWP < size(WPs, 2)
       curWP = curWP + 1
       k
   end
   
end
% Log 
log.set('r_preconv', r);

if useInputShaperZV || useInputShaperZVD
    % Create shaper
    omega_n = sqrt(pg/pL);

    xi = 0.2; % this is just a guess!
    xi = pd/(2*omega_n*pm_L);

    omega_d = omega_n*sqrt(1-xi^2);

    Td = 2*pi/omega_d;

    K = exp(-(xi*pi)/sqrt(1-xi^2));
end

% Apply shaper
if useInputShaperZVD
    

    A1 = 1/(1 + 2*K + K^2);
    A2 = 2*K/(1 + 2*K + K^2);
    A3 = K^2/(1 + 2*K + K^2);

    t2 = Td/2;
    t3 = Td;

    % Then, create a sequence at these times. 

    % signal length:
    k3 = find(time >= t3,1);
    k2 = find(time >= t2,1);

    shaper = zeros(1, k3);
    shaper(k3) = A3;
    shaper(k2) = A2;
    shaper(1)  = A1;   
    
    
    % Try to "up" the shaper by initial condition
    r_n = [];
    r_n(1,:) = conv(r(1,:), shaper+ r(1,1)*ones(size(shaper)));
    r_n(2,:) = conv(r(2,:), shaper+ r(2,1)*ones(size(shaper)));
    r_n(3,:) = conv(r(3,:), shaper+ r(3,1)*ones(size(shaper)));
    %r = r_n;
    
    % Does not work as expected, using zero initialized version
    r = convn(r, shaper);
    dr = convn(dr, shaper);
    d2r = convn(d2r, shaper);
    
elseif useInputShaperZV
    
    A1 = 1/(1 + K);
    A2 = K/(1 + K);

    t2 = Td/2;
    

    % Then, create a sequence at these times. 

    % signal length:
   
    k2 = find(time >= t2,1);

    shaper = zeros(1, k2);
    shaper(k2) = A2;
    shaper(1)  = A1;  
    
    % Try to "up" the shaper by initial condition
    r_n = [];
    r_n(1,:) = conv(r(1,:), shaper+ r(1,1)*ones(size(shaper)));
    r_n(2,:) = conv(r(2,:), shaper+ r(2,1)*ones(size(shaper)));
    r_n(3,:) = conv(r(3,:), shaper+ r(3,1)*ones(size(shaper)));
    kkk1 = d2r;
   
    %r = r_n;
    r = convn(r, shaper);
    dr = convn(dr, shaper);
    d2r = convn(d2r, shaper);
    
    kkk2 = d2r;
end

log.set('r', r(:,1:length(time)));

% Start dynamic simulation
for k = 1:length(time)
   t = time(k);
   
   
   
   % Do some control
   tau = zeros(5,1); 
   %tau(3) = - pg*(pm_L + pm_c);
   
   % Feedback
   if t > 5
        omega_n = sqrt(pg/pL);

        xi = 0.2; % this is just a guess!
        xi = pd/(2*omega_n*pm_L);

        omega_d = omega_n*sqrt(1-xi^2);

        Td = 2*pi/omega_d;
        tau_d = 0.325*Td;
        Gd    = 0.325;
        oldAngles = [0; 0];
        olddAngles = [0; 0];
        oldd2Angles = [0; 0];
        if t > tau_d
            % Find time
            oldTimeIndex = find(time>t-tau_d,1,'first');
            oldAngles = log.get('Eta', 4:5, oldTimeIndex);
            olddAngles = log.get('Nu', 4:5, oldTimeIndex);
            oldd2Angles = log.get('dNu', 4:5, oldTimeIndex);
        end
        
        %Re-define current desired pos. 
        r(1,k) = r(1,k) + Gd*pL*sin(oldAngles(2));
        r(2,k) = r(2,k) - Gd*pL*sin(oldAngles(1));
        
        dr(1,k) = dr(1,k) + Gd*pL*cos(oldAngles(2))*olddAngles(2);
        dr(2,k) = dr(2,k) - Gd*pL*cos(oldAngles(1))*olddAngles(1);
        
        d2r(1,k) = d2r(1,k) + Gd*pL*(-sin(oldAngles(2))*olddAngles(2) + cos(oldAngles(2))*oldd2Angles(2));
        d2r(2,k) = d2r(2,k) - Gd*pL*(-sin(oldAngles(1))*olddAngles(1) + cos(oldAngles(1))*oldd2Angles(1));
   end
   
    % Do some control
    tau = zeros(5,1); 
    Kp = 1;
    Kd = 10;

    % pos error
    e_pos = r(:,k) - eta(1:3)
    % vel error
    e_vel = dr(:,k) - nu(1:3)


    tau(1:3) = Kp*e_pos + Kd*e_vel;
    tau(3) = tau(3,1) - pg*(pm_L + pm_c);

   
                                                    % 
                                                    %    
                                                    %    % Define error variables
                                                    %    z1 = H*eta - r(:,k);
                                                    %    
                                                    %    % Virtual control 1-2
                                                    %    alpha_13 = dr(:,k) - K1 * z1;
                                                    %    alpha = [alpha_13; alpha_45];
                                                    %    
                                                    %    z2 = nu - alpha;
                                                    %    
                                                    %    dalpha_13 = d2r(:,k) - K1*H*z2 + K1*K1*z1;
                                                    %    
                                                    %    % Shorthands
                                                    %    z45 = z2(4:5);
                                                    %    theta_L = eta(5);
                                                    %    phi_L = eta(4);
                                                    %    
                                                    %    G45 = G(eta);
                                                    %    G45 = G45(4:5);
                                                    %    
                                                    %    K45 = K2(4:5,4:5);
                                                    %    
                                                    %    M4513 = M(eta);
                                                    %    M4513 = M4513(4:5,1:3);
                                                    %    
                                                    %    Malpha = M(eta);
                                                    %    Malpha = Malpha(4:5, 4:5);
                                                    %    
                                                    %    Calpha = C(eta, nu);
                                                    %    Calpha = Calpha(4:5, 4:5);
                                                    %    
                                                    %    Dalpha = D(4:5, 4:5);
                                                    %    
                                                    %    % Define residual
                                                    %    gamma = @(eta, pm_L)(-G45 + K45*z45 - M4513*dalpha_13);
                                                    %    
                                                    %    
                                                    %    
                                                    %    dalpha_45 = @(alpha_45, ~)( Malpha\(-Dalpha*alpha_45 - Calpha*alpha_45 + gamma(alpha_45, pm_L)));
                                                    %    
                                                    %    
                                                    %    dalpha = [dalpha_13; dalpha_45(alpha_45, 0)];
                                                    %    
                                                    %    % Main control part
                                                    %     tau = C(eta, nu)*alpha + D*alpha + G(eta) - H'*z1 + M(eta)*dalpha - K2*z2;
                                                    %    
                                                    %    % For fun, print third value
                                                    %    %tau(3)
                                                    %    

   
   % Log third value
   log.store('tau_45', tau(4:5), k);
   
   % aand, set to zero
   tau(4:5) = [0; 0];
   
   
   if t > tend/2
       %tau(1:2) = [0; 0];
   end
   
   % Integrate to get alpha_3
%   alpha_45 = rk4(dalpha_45, alpha_45, 0, h);
   
   % Simulate position integration
   f = @(eta, nu)(nu);
   eta_next = rk4(f, eta, nu, h);
   
   % Integrate velocity
   f = @(nu, tau)(M(eta)\(tau - C(eta, nu)*nu - G(eta) - D*diag(abs(nu))*nu));
   dnu = f(nu, tau);
   nu_next  = rk4(f, nu, tau, h);
   
   % Update current state
   eta = eta_next;
   nu  = nu_next;
   
   % Log
   log.store('Eta', eta, k);
   log.store('Nu', nu, k);
   log.store('dNu', dnu, k);
   log.store('tau', tau, k);
   
   
   
   % Print
   checkEveryInterval = 0.4;
    if ~mod(time(k), checkEveryInterval) 
        %time(t)
        step = time(k);
        t_sec = toc(c_sec)/checkEveryInterval;
        if time(k) == 0
            t_sec = 1;
        end
        %t_sec = toc(c_total);
        
        c_sec = tic;
    end
    
    outline = sprintf(['* Simulation time: %.2fs of %ds. \n' ...
                       '* Computational time: %.2fs. \n' ...
                       '* Time remaining: %.1fs. \n'], time(k), tend, toc(c_total), abs(t_sec*tend - toc(c_total))); 
    fprintf([reverseString, outline]);
    reverseString = repmat(sprintf('\b'), 1, length(outline));

   
   
end
%% Print Done!
t_total = toc(c_total);
outline = sprintf(['*** Simulation complete *** \n' ...
         '* Total computation time: %f \n' ...
         '* Average time per second: %f \n' ...
         '* System Equations: %d \n'...
         '***\n' ...
         ], t_total, t_total/tend, 5);
fprintf([reverseString, outline]);
reverseString = repmat(sprintf('\b'), 1, length(outline));

%% Store logs

switch doCase
    case cTracking
        log_cTracking = log;
    case cBox
        log_cBox = log;
    case cBoxShape
        log_cBoxShape = log;
end


%% Save logs
timeNow = datestr(now, 'yyyy.mm.dd-HH.MM.SS');

% save the log!
switch doCase
    case cTracking
        storeName = 'Tracking_';
        logName = 'log_cTracking';
    case cBox
        storeName = 'Box_';
        logName = 'log_cBox';
    case cBoxShape
        storeName = 'BoxShape_';
        logName = 'log_cBoxShape';
end

save(['logs/' 'sim_' storeName timeNow '.mat'], logName);

%% PLOT
% Do some simple plotting!
figure(2); clf

subplot(3,1,1);
title('x');
plot(time, log.get('Eta', 1));

subplot(3,1,2);
title('y');
plot(time, log.get('Eta', 2));

subplot(3,1,3);
title('\theta_L');
plot(time, log.get('Eta', 3));



% Activate super-screen? 
activateSuperScreen = 1;





p = Plotter;
p.setSize(2,4);
p.activate('main');
p.activate('theta');
p.activate('pos_error');
p.activate('tau');
p.activate('pos');
p.activate('m_L_hat');
% p.activate('loadConstraint');
% p.activate('mu');





% Prepare some data
load = 1;


if doInteractivePlot


fig = figure(1); clf; %subplot(3,3,[1 2 4 5 7 8]); hold on;



[on, subcell] = p.isActive('main');
if on
    subplot(subcell{1}{:})
    view(3);
    set(gca, 'ZDir', 'reverse');
    set(gca, 'YDir', 'reverse');
    hold on;
    
    copter_x = log.get('Eta', 1, :);
    copter_y = log.get('Eta', 2, :);
    copter_z = log.get('Eta', 3, :);
    lim_x = [min(copter_x)-1.1*pL max(copter_x)+1.1*pL]
    lim_y = [min(copter_y)-1.1*pL max(copter_y)+1.1*pL]
    lim_z = [min(copter_z)-1.1*pL max(copter_z)+1.1*pL]
    axis([ lim_x lim_y lim_z]);

    axis equal;
    xlabel('x'); ylabel('y'); zlabel('z');
    grid on;
end
% 
% lim_x = [min(eta(1,:))-0.5 max(eta(1,:))+0.5]
% lim_y = [min(eta(2,:))-0.5 max(eta(2,:))+0.5]
% lim_z = [min( [min(eta(3,:))-0.1 min(eta_l(3,:))]) max([max(eta(3,:)+0.1) max(eta_l(3,:))])] 
%
% 
% axis([ lim_x lim_y lim_z]);
% Plot the result of convolution

figure(2); clf
hold on;
plot(time, log.get('r_preconv', 1,:), 'b');
plot(time, log.get('r', 1,:), 'b--');


figure(1);
% Plot nicely

old_copter = [];
old_plot = [];
old_load = [];
skips = 10;
timeShifter = 1;

t = 1;

if doOnlyLastFrame
    t = length(time)-1
end

movie = 0;

if movie
   M =  cell(1, length(time));
end
while t < length(time)
    
    %profile on
    timeToDraw = tic;
  
    p.deleteOld();
    
    % PLOT MAIN COPTER VIEW
    [on, subcell] = p.isActive('main');
    if on
        subplot(subcell{1}{:})


        if all(ishandle(old_copter))
            delete(old_copter);
        end
        
        % Plot current copter position

        
        p.plot3(log.get('Eta', 1, t), log.get('Eta', 2, t), log.get('Eta', 3, t), 'ro');
        
        % Plot line to the load
        x_copter = log.get('Eta', 1, 1:t);
        y_copter = log.get('Eta', 2, 1:t);
        z_copter = log.get('Eta', 3, 1:t);
        
        
        x_load = x_copter + pL*sin(-log.get('Eta', 3, 1:t));
        y_load = y_copter + pL*cos(-log.get('Eta', 3, 1:t));
        y_load = y_copter + pL*cos(-log.get('Eta', 3, 1:t));
        
        
        phi_L_now = log.get('Eta', 4, t);
        theta_L_now = log.get('Eta', 5, t);
        
        p_load = log.get('Eta', 1:3, t) + f_5dof_Rload(phi_L_now, theta_L_now)*[0; 0; pL];

        h_line = p.line([x_copter(end) p_load(1)], [y_copter(end) p_load(2)], [z_copter(end) p_load(3)]);
        set(h_line, 'color', 'k');
        
        % Plot load
        p.plot3(p_load(1), p_load(2), p_load(3), 'go');
        
        % Plot history of load
        %p.plot(x_load, y_load, 'k--');
        
        % Plot history copter
        p.plot3(x_copter, y_copter, z_copter, 'b--');
        
        % Plot desired position
        p.plot3(r(1,1:t), r(2,1:t), r(3,1:t), 'g');

        

        axis equal;
        axis([ lim_x lim_y lim_z]);
        if movie
            % Trhee moving
            %axis([-2.0913    1.0328   -1.5262    1.2083   -1.8269    1.3284]);
            
            % two demo
            %axis([-3.1999    1.3166   -0.8530    0.9999   -1.7931    1.6479]);
            
            
            % Wierd one no control
            %axis([ -2.8706    1.2828   -4.0354    2.8947   -1.7311    6.2465]);
            
            % 3 tracking test
            %axis([    1.0000   12.8226    0.6557    3.3443    1.0010    3.8016]);

            
            
            
        end
    end
    
    
    % PLOT Theta_L and phi_L
    [on, subcell] = p.isActive('theta');
    if on
        subplot(subcell{1}{:})
        
        hold on;
        axis fill
        xlim([0 tend]);
        title('Theta_L and Phi_L'); xlabel('time [s]'); ylabel('theta_L [deg]');
        p.plot(time(1:t), pi\180*log.get('Eta', 4, 1:t));
        p.plot(time(1:t), pi\180*log.get('Eta', 5, 1:t), 'r');
        %p.plot(time(1:t), pi\180*sys.Copters(infoCopter).Eta_r_storage(4, 1:t), 'r');
        %p.plot(time(1:t), pi\180*sys.Copters(infoCopter).Eta_comp_storage(4, 1:t), 'g');
        
    end
    
    % PLOT pos_error
    [on, subcell] = p.isActive('pos_error');
    if on
        subplot(subcell{1}{:})
        
        hold on;
        axis fill
        xlim([0 tend])
        title('Position error'); xlabel('time [s]'); ylabel('error');
        p.plot(time(1:t), log.get('Eta', 1,1:t) - r(1,1:t), 'b');
        p.plot(time(1:t), log.get('Eta', 2,1:t) - r(2,1:t), 'g');
        p.plot(time(1:t), log.get('Eta', 3,1:t) - r(3,1:t), 'r');
        %p.plot(time(1:t), pi\180*sys.Copters(infoCopter).Eta_r_storage(4, 1:t), 'r');
        %p.plot(time(1:t), pi\180*sys.Copters(infoCopter).Eta_comp_storage(4, 1:t), 'g');
        
        grid on;
        
    end
    
    % PLOT taur
    [on, subcell] = p.isActive('tau');
    if on
        subplot(subcell{1}{:})
        
        hold on;
        axis fill
        xlim([0 tend])
        title('Control input'); xlabel('time [s]'); ylabel('tau');
        p.plot(time(1:t), log.get('tau', 1,1:t), 'b');
        p.plot(time(1:t), log.get('tau', 2,1:t), 'g');
        p.plot(time(1:t), log.get('tau', 3,1:t), 'r');
        p.plot(time(1:t), log.get('tau_45', 1,1:t), 'k--');
        p.plot(time(1:t), log.get('tau_45', 2,1:t), 'k--');
        %p.plot(time(1:t), pi\180*sys.Copters(infoCopter).Eta_r_storage(4, 1:t), 'r');
        %p.plot(time(1:t), pi\180*sys.Copters(infoCopter).Eta_comp_storage(4, 1:t), 'g');
        
    end
    
    
    % PLOT m_L_hat
    [on, subcell] = p.isActive('m_L_hat');
    if on
        subplot(subcell{1}{:})
        
        hold on;
        axis fill
        xlim([0 tend]);
        title('Payload mass estimate'); xlabel('time [s]'); ylabel('m_L_hat');
        p.plot(time(1:t), log.get('m_L_hat', 1, 1:t));
        p.plot(time, pm_L*ones(size(time)), '--');
        
        %p.plot(time(1:t), pi\180*sys.Copters(infoCopter).Eta_comp_storage(4, 1:t), 'g');
        
    end
    
    % PLOT position
    [on, subcell] = p.isActive('pos');
    if on
        subplot(subcell{1}{:})
        
        hold on;
        axis fill
        xlim([0 tend])
        title('Position'); xlabel('time [s]'); ylabel('error');
        p.plot(time(1:t), log.get('Eta', 1,1:t), 'b');
        p.plot(time(1:t), r(1,1:t), 'b--');
        p.plot(time(1:t), log.get('Eta', 2,1:t), 'r');
        p.plot(time(1:t), r(2,1:t), 'r--');
        p.plot(time(1:t), log.get('Eta', 3,1:t), 'g');
        p.plot(time(1:t), r(3,1:t), 'g--');
        %p.plot(time(1:t), pi\180*sys.Copters(infoCopter).Eta_r_storage(4, 1:t), 'r');
        %p.plot(time(1:t), pi\180*sys.Copters(infoCopter).Eta_comp_storage(4, 1:t), 'g');
        
        grid on;
        
    end
    
    infoCopter = 1;
    
  
    
    drawnow;
    %profile viewer
    %return
    
    if movie
        M{t} = getframe(gcf);
    end
    
    timeToDraw = toc(timeToDraw);
    
    steps = int32(timeToDraw/h);
    if steps < 1
        steps = 1;
    end
    %steps = 1;
    
    if movie
        steps = 5;
    end
    
    t = t + steps;
    
    %pause(timeShifter*skips*h)
    
end

end % interactive plot
%%
