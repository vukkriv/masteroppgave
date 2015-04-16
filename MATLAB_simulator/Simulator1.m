%% Clear
clear(); clc; close all;
clear nextState; % clear the state

%% configurations - Settings
simState_freq = 50;   
nextState_freq = 20;  % between 0-17 hz;
init_lat = 63.417349;
init_lon = 10.406316;
useOde45 = true;


%% Initiate IMC
javaaddpath('imcjava/dist/libimc.jar')
import pt.lsts.imc.*
import pt.lsts.imc.net.*
imc = IMCProtocol();
pause(3)                
imc.systems()

%% Initial conditions
whileloop = true;
eta = [0, 0, 0, 0.00, 0.00]';
nu  = [0, 0, 0, 0, 0]';
r = [0 0 0]';
dr = [0 0 0]';
d2r = [0 0 0]';
loop = 1;                   % loop counter
total_time = 0;             % total time
timestamp_last = 0;         % last received timestamp from nextState
simState_time_tracker = 0;  % time since last simsState msg sent
nextState_time_tracker = 0; % time since last nextState msg is sent
time_whitout_nextState = 0; % time whitout new state
h = 2;                      % initial stepsize set high
k = 1;                      % independent counter
tau = zeros(5,1);
eta_previous = [1, 1, 2, 1.00, 0.00]';

%% Loggers
samplesizes = 1000000;
timelogg = zeros(2,samplesizes);
etalogg  = zeros(5,samplesizes); %[x y z theta psi] 
nulogg = zeros(5,samplesizes);
desiredPath = zeros(6,samplesizes);
taulogg = zeros(5,samplesizes);
time = zeros(samplesizes,1);



%% Loop
while whileloop
    tic
%    try
        %% Dispatch simulated state
        if simState_time_tracker > 1/simState_freq
            %fprintf('Simstate freq: %f \n',1/simState_time_tracker);
            simState = SimulatedState();
            simState.setLat(init_lat * pi/180);
            simState.setLon(init_lon * pi/180);
            simState.setX(eta(1));
            simState.setY(eta(2));
            simState.setZ(eta(3));
            simState.setU(nu(1));
            simState.setV(nu(2));
            simState.setW(nu(3));
            % Use for euler angles?
            simState.setTheta(eta(5));
            simState.setPhi(eta(4));
            simState.setP(nu(5));
            simState.setQ(nu(4));
            imc.sendMessage('ntnu-hexa-004', simState);
            simState_time_tracker = 0;
        end
        
        
        %% Controller - update
        nextState = imc.state('ntnu-hexa-004').get('TranslationalSetpoint');
        if nextState_time_tracker > 1/nextState_freq
            %fprintf('nextState freq: %f \n',1/nextState_time_tracker);
            if  ~isempty(nextState)
                x = nextState.getX();
                y = nextState.getY();
                z = nextState.getZ();
                u = nextState.getU();
                v = nextState.getV();
                w = nextState.getW();
                timestamp = nextState.getTimestamp();
                
                % Integration parameters
                h = timestamp - timestamp_last;
                r = [x y z]';
                dr = [u v w]';
                if h ~= 0 % update last timestamp if new arrival
                    timestamp_last = timestamp;
                    time_whitout_nextState = 0;
                else
                    h = nextState_time_tracker;   
                    %fprintf('nextState_time_tracker = %f \n', h)
                    if time_whitout_nextState > 0.2
%                         r = [0 0 0]';
%                         dr = [0 0 0]';
%                         [eta,nu] = backstepping(r, dr, d2r,eta,nu,h,useOde45)
                        fprintf('No updates on nextState since:  %f \n', time_whitout_nextState);
                    end
                end
            end
            
            if h < 0.2 && h ~= 0 %&& ~isequal(eta_previous,eta) %new measurement
                fprintf('The measurement step h = %f \n', h);
                [eta,nu,tau] = backstepping(r, dr, d2r,eta,nu,h,useOde45);
                timelogg(:,k) = [total_time; h];
                k = k+1;

            else
                fprintf('Not integrating, h too high: h = %f \n',h);
            end
            nextState_time_tracker = 0;
        end

        
        % Handle timers

        loop = loop +1;
        etalogg(:,loop) = eta;
        nulogg(:,loop) = nu;
        desiredPath(:,loop) = [r; dr];  
        taulogg(:,loop) = tau;
        time_whitout_nextState = time_whitout_nextState + toc;
        nextState_time_tracker = nextState_time_tracker + toc;
        simState_time_tracker = simState_time_tracker + toc;
        total_time = total_time + toc;
        time(loop) = total_time;
        
        %% Logg
        timelogg(:,loop) = [h, 0];
%         etalogg(:,loop) = eta;
%         nulogg(:,loop) = nu;
%         desiredPath(:,loop) = [r; dr];  
      
%    catch
%          fprintf('\n ERROR in loop \n');
%          break;
%    end
end

fprintf('Out of the loop \n');
fprintf('Closing ports... \n');
pause(1);
imc.stop()

%%
%% Plots
% just interested in mission
for i = 1:loop
    if etalogg(1,i) ~= 0
        plotstart = i-100;
        break;
    end
end


% Desired pos/vel
N = plotstart:loop; % convert to seconds using timelogg?
k = time(N);
figure()
subplot(2,1,1)
hold on
plot(k,  desiredPath(1,N),'r');
plot(k,  desiredPath(2,N),'b');
plot(k,  desiredPath(3,N),'g');
hold off
title('pos');
legend('x','y','z');
subplot(2,1,2)
hold on;
plot(k,  desiredPath(4,N),'r');
plot(k,  desiredPath(5,N),'b');
plot(k,  desiredPath(6,N),'g');
hold off
title('vel');
legend('u','v','w');
suptitle('Desired pos/vel');
xlabel('loop');

% pos - eta, nu
figure()
subplot(2,1,1)
hold on
plot(k,  etalogg(1,N),'r');
plot(k,  etalogg(2,N),'b');
plot(k,  etalogg(3,N),'g');
hold off
title('pos');
legend('eta(1)','eta(2)','eta(3)');
subplot(2,1,2)
hold on;
plot(k,  nulogg(1,N),'r');
plot(k,  nulogg(2,N),'b');
plot(k,  nulogg(3,N),'g');
hold off
title('vel');
legend('nu(1)','nu(2)','nu(3)');
suptitle('Eta/NU');
xlabel('loop');


% error desired pos/vel
% pos - eta, nu
figure()
subplot(2,1,1)
hold on
plot(k,  desiredPath(1,N) - etalogg(1,N),'r');
plot(k,  desiredPath(2,N) - etalogg(2,N),'b');
plot(k,  desiredPath(3,N) - etalogg(3,N),'g');
hold off
title('pos');
legend('eta(1)','eta(2)','eta(3)');
subplot(2,1,2)
hold on;
plot(k,  desiredPath(4,N) - nulogg(1,N),'r');
plot(k,  desiredPath(5,N) - nulogg(2,N),'b');
plot(k,  desiredPath(6,N) - nulogg(3,N),'g');
hold off
title('vel');
legend('nu(1)','nu(2)','nu(3)');
suptitle('Error: desired pos/vel - Eta/NU');
xlabel('loop');



% pos - tau
figure()
plot(k,  taulogg(:,N)*180/pi);
title('Tau');
legend('tau(1)','tau(2)','tau(3)','tau(4)','tau(5)');
xlabel('loop');


% pos - eta, nu
figure()
subplot(2,1,1)
hold on
plot(k,  etalogg(4,N)*180/pi,'r');
%axis([N(1) N(end) -180 180])
plot(k,  etalogg(5,N)*180/pi,'b');
hold off
title('Pos');
legend('eta(4)','eta(5)');
subplot(2,1,2)
hold on;
plot(k,  nulogg(4,N)*180/pi,'r');
plot(k,  nulogg(5,N)*180/pi,'b');
hold off
title('vel');
legend('nu(4)','nu(5)');
suptitle('Load angle');
xlabel('loop');

%axis([N(1) N(end) -180 180])
