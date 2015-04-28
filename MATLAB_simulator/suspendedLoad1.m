%% Clear
clear(); clc; close all;
clear nextState; % clear the state


%% configurations - Settings
whileloop = true;
EstimatedState_freq = 20; % 5 Hz is max?
EulerAngles_freq = 20; 
plot_EulerAngles = false;
plot_LoadToNEDAngle = true;


%% Initiate IMC
javaaddpath('imcjava/dist/libimc.jar')
import pt.lsts.imc.*
import pt.lsts.imc.net.*
imc = IMCProtocol();
pause(3)                
imc.systems()

%% Initial Variables

r2d = 180/pi;
d2r = pi/180;
newEstimatedState = true;
newEulerAngles = true;
total_loop = 1;
Estate_loop = 1;
Euler_loop = 1;
angloop = 1;

total_time = 0;
timestamp = 0;
timestamp_minus = 0;
timestamp_L = 0;
timestamp_L_minus = 0;
nextEstimate_time = 0;
nextEuler_time = 0;

new_estate = false;
new_euler = false;

phi = 0;
theta = 0;
psi = 0;
phi_L = 0;
theta_L = 0;

%% Logg
samplesizes = 1000000;
time = zeros(samplesizes,1);
Estate_time = zeros(samplesizes,1);
Euler_time = zeros(samplesizes,1);
ang_time = zeros(samplesizes,1);
attitude_logg = zeros(3,samplesizes);
euler_logg = zeros(2,samplesizes);
Ang_logg = zeros(2,samplesizes);


%% Rotation matrices
Rx = @(phi) [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi);];
Ry = @(theta) [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta);];
Rz = @(psi) [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1;];

%% Define figures
figure(1); clf; hold on;
figure(2); clf; hold on; axis equal;
set(gca, 'ZDir', 'reverse');
    set(gca, 'YDir', 'reverse');
    hold on;

    axis equal;
    xlabel('x'); ylabel('y'); zlabel('z');

%% Loop
while whileloop
    tic
        
    %% Estimated State
    EstimatedState = imc.state('ntnu-hexa-004').get('EstimatedState');
    if nextEstimate_time > 1/EstimatedState_freq
        if  ~isempty(EstimatedState)
            if (EstimatedState.getTimestamp() - timestamp_minus) ~= 0 % if new measurement 
               phi = EstimatedState.getPhi();     % Roll
               theta = EstimatedState.getTheta(); % pith
               psi = EstimatedState.getPsi();     % yaw
               timestamp = EstimatedState.getTimestamp();
               fprintf('EstimatedState freq: %f \n imctime %f \n \n', 1/nextEstimate_time,EstimatedState.getTimestamp() - timestamp_minus);
               nextEstimate_time = 0;
               attitude_logg(:,Estate_loop) = [phi*r2d;theta*r2d;psi*r2d];
               Estate_time(Estate_loop) = total_time;
               Estate_loop = Estate_loop + 1;
               new_estate = true;
            end
            timestamp_minus = EstimatedState.getTimestamp();
        end
    end
    
    %% Euler Angles
    oldPlots = [0, 0];
    EulerAngles = imc.state('ntnu-hexa-004').get('EulerAngles');
    if nextEuler_time > 1/EulerAngles_freq
         if  ~isempty(EulerAngles)
             if (EulerAngles.getTimestamp() - timestamp_L_minus) ~= 0 % if new measurement
                phi_L = (EulerAngles.getPhi());% - 180);     % about y axis
                theta_L = (EulerAngles.getTheta());% - 180); % about x axis
                timestamp_L = EulerAngles.getTimestamp();
                fprintf('EulerAngle freq: %f \n imctime: %f \n \n', 1/nextEuler_time, EulerAngles.getTimestamp() - timestamp_L_minus);
                nextEuler_time = 0;
                euler_logg(:,Euler_loop) = [phi_L*r2d;theta_L*r2d];
                Euler_time(Euler_loop) = total_time;
                Euler_loop = Euler_loop + 1;
                new_euler = true;
                
                % PLOTS
                if plot_EulerAngles
                    if ~any(oldPlots==0)
                        delete(oldPlots)
                    end
                    figure(1);
                    N_eulerloop = 1:Euler_loop-1;
                    oldPlots(1) = plot(Euler_time(N_eulerloop),euler_logg(1,N_eulerloop),'b');
                    oldPlots(2) = plot(Euler_time(N_eulerloop),euler_logg(2,N_eulerloop),'r');
                    drawnow
                end

             end
            timestamp_L_minus = EulerAngles.getTimestamp();
         end
    end
    
    oldPlots = [0, 0];
    oldPlots2 = [0];
    if new_euler || new_estate
        
        Rbn = Rzyx(phi,theta,psi);
        %Rbn = Rx(phi)*Ry(theta)*Rz(psi);
        
        %Rlb = Rzyx(phi_L*d2r,theta_L*d2r,0);
        Rlb = Rx(phi_L)*Ry(theta_L);
        
        wire_length = 1;
        load_pos = Rbn*Rlb * [0; 0; wire_length;];
       
        norm(load_pos)
        load_leng = norm(load_pos);
        %theta_Ln = atan(load_pos(1)/load_pos(2) );
        %theta_Ln = acos( load_pos(1) / sqrt(load_pos(1)^2 + load_pos(2)^2) );
        %phi_Ln = 0;%asin(load_pos(2) / load_leng);
        phi_Ln = atan(load_pos(2)/load_pos(3));
        theta_Ln = atan(load_pos(1)/load_pos(3));
        Ang_logg(:,angloop) = [phi_Ln*r2d; theta_Ln*r2d];
        
       
        new_euler = false;
        new_estate =  false;
        
        ang_time(angloop) = total_time;
        angloop = angloop +1;
        
        %% Plots
        if plot_LoadToNEDAngle
            if ~any(oldPlots2==0)  
                 delete(oldPlots)
                delete(oldPlots2);
            end
            figure(1); cla;
            N_angloop = 1:angloop-1;
            oldPlots(1) = plot(ang_time(N_angloop),Ang_logg(1,N_angloop),'b');
            oldPlots(2) = plot(ang_time(N_angloop),Ang_logg(2,N_angloop),'r');

            
            
            figure(2); cla;
            oldPlots2 = plotCopter([0,0,0]', [phi, theta, psi]');
            
            line([0 load_pos(1)], [0 load_pos(2)], [0, load_pos(3)]);
            drawnow
        end
        
        
        
    end
    
    
    
    
    %fprintf('Loop freq: %f \n', 1/toc);
    total_time = total_time + toc;
    nextEstimate_time = nextEstimate_time + toc;
    nextEuler_time = nextEuler_time + toc;
    total_loop = total_loop + 1;
    time(total_loop) = total_time;
  
    
    

    
end

%%
fprintf('Out of the loop \n');
fprintf('Closing ports... \n');
pause(1);
imc.stop()

%% PLOT

% Attitude
figure()
hold all;
N_Estateloop = 1:Estate_loop-1;
plot(Estate_time(N_Estateloop),attitude_logg(1,N_Estateloop),'.');
plot(Estate_time(N_Estateloop),attitude_logg(2,N_Estateloop),'.');
plot(Estate_time(N_Estateloop),attitude_logg(3,N_Estateloop),'.');
legend('\phi','\theta','\psi');
title('Load angle - Euler angle');
xlabel('Time (s)'); ylabel('Degree');
hold off;

% Euler Angles
figure()
hold all;
N_eulerloop = 1:Euler_loop-1;
plot(Euler_time(N_eulerloop),euler_logg(1,N_eulerloop),'.');
plot(Euler_time(N_eulerloop),euler_logg(2,N_eulerloop),'.');
legend('\phi_L','\theta_L');
title('Load angle - Euler angle');
xlabel('Time (s)'); ylabel('Degree');
hold off;

