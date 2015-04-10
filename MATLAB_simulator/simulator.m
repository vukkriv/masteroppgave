%% Clear
clear(); clc; close all;
clear nextState; % clear the state


% GUI for breaking the loop
DlgH = figure;
GUI = uicontrol('Style', 'PushButton', ...
                    'String', 'Stop listening...', ...
                    'FontSize',25,...
                    'FontWeight','bold',...
                    'Position', [10 10 400 400],...
                    'BackgroundColor','red',...
                    'Callback', 'delete(gcbf)');
      
%% Initiate
javaaddpath('../../../imcjava/dist/libimc.jar')
import pt.lsts.imc.*
import pt.lsts.imc.net.*
imc = IMCProtocol();
listening = true;
pause(3) 
imc.systems()

% Initial conditions
eta = [0, 0, 0, 0.00, 0.00]';
nu  = [0, 0, 0, 0, 0]';
r = [0 0 0]';
dr = [0 0 0]';
d2r = [0 0 0]';
h = 0;
loop = 0; % count loop
tloop = 0; % time 
tinnerloop = 0; % time for innder loop
timestamp_minus = 0;

lastMsgTime = 0.1


%loggers
samplesizes = 1000000;
timelogg = zeros(2,samplesizes); %[h loop] 
etalogg  = zeros(5,samplesizes); %[x y z theta psi] 
nulogg = zeros(5,samplesizes);
desiredPath = zeros(6,samplesizes);

%% Loop
while (ishandle(GUI)) && (listening)
    if loop < 1
        tic
    end
     try
        % Get desired r,dr 
        nextState = imc.state('ntnu-hexa-004').get('TranslationalSetpoint');
        if ~isempty(nextState)
            x = nextState.getX();
            y = nextState.getY();
            z = nextState.getZ();
            u = nextState.getU();
            v = nextState.getV();
            w = nextState.getW();
            timestamp = nextState.getTimestamp();
            % Keep time between old and new received msg
            if timestamp == timestamp_minus
                tinnerloop =  tinnerloop + toc
            else
                tinnerloop = 0;
            end
            % If there is no new msg since lastMsgTime
            if loop > 1 && tinnerloop < lastMsgTime
                %fprintf('TranslationalSetpoint mottat \n');
                r = [x y z]';
                dr = [u v w]';
                [eta,nu] = backstepping(r, dr, d2r,eta,nu,0.01);
            end
            timestamp_minus = timestamp;
        else
            fprintf('TranslationalSetpoint is empty...\n');
        end
        % Dipatch Simulated state
        tloop = toc + tloop
        if true %% tidspegrensning?
            simState = SimulatedState();
            simState.setLat(pi/180*63.417349);
            simState.setLon(pi/180*10.406316);
            simState.setX(eta(1));
            simState.setY(eta(2));
            simState.setZ(eta(3));
            simState.setU(nu(1));
            simState.setV(nu(2));
            simState.setW(nu(3));
            % Use for euler angles?
            simState.setTheta(eta(4)*180/pi);
            simState.setPhi(eta(5)*180/pi);
            simState.setP(nu(4)*180/pi);
            simState.setQ(nu(5)*180/pi);
            imc.sendMessage('ntnu-hexa-004', simState);
            tloop = 0;
        else
            fprintf('Simulated state not dispatched \n');
           %break
        end
        tic
        % Logg
        loop = loop+1;
        timelogg(:,loop) = [h, 0];
        etalogg(:,loop) = eta;
        nulogg(:,loop) = nu;
        desiredPath(:,loop) = [r; dr];   
    catch
        fprintf('\n ERROR in loop \n');
        listening = false;
        break;
     end
     %pause(0.00001);
end

fprintf('Out of the loop \n');
fprintf('Closing ports... \n');
pause(1);
imc.stop()




%% Plots
% just interested in mission
plotstart = 0;
for i = 1:loop
    if etalogg(1,i) ~= 0
        plotstart = i-100;
        break;
    end
end


% Desired pos/vel
N = plotstart:loop; % convert to seconds using timelogg?
figure()
subplot(2,1,1)
hold on
plot(N,  desiredPath(1,N),'r');
plot(N,  desiredPath(2,N),'b');
plot(N,  desiredPath(3,N),'g');
hold off
title('pos');
legend('x','y','z');
subplot(2,1,2)
hold on;
plot(N,  desiredPath(4,N),'r');
plot(N,  desiredPath(5,N),'b');
plot(N,  desiredPath(6,N),'g');
hold off
title('vel');
legend('u','v','w');
suptitle('Desired pos/vel');
xlabel('loop');

% pos - eta, nu
figure()
subplot(2,1,1)
hold on
plot(N,  etalogg(1,N),'r');
plot(N,  etalogg(2,N),'b');
plot(N,  etalogg(3,N),'g');
hold off
title('pos');
legend('eta(1)','eta(2)','eta(3)');
subplot(2,1,2)
hold on;
plot(N,  nulogg(1,N),'r');
plot(N,  nulogg(2,N),'b');
plot(N,  nulogg(3,N),'g');
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
plot(N,  desiredPath(1,N) - etalogg(1,N),'r');
plot(N,  desiredPath(2,N) - etalogg(2,N),'b');
plot(N,  desiredPath(3,N) - etalogg(3,N),'g');
hold off
title('pos');
legend('eta(1)','eta(2)','eta(3)');
subplot(2,1,2)
hold on;
plot(N,  desiredPath(4,N) - nulogg(1,N),'r');
plot(N,  desiredPath(5,N) - nulogg(2,N),'b');
plot(N,  desiredPath(6,N) - nulogg(3,N),'g');
hold off
title('vel');
legend('nu(1)','nu(2)','nu(3)');
suptitle('Error: desired pos/vel - Eta/NU');
xlabel('loop');




% pos - eta, nu
figure()
subplot(2,1,1)
hold on
plot(N,  etalogg(4,N)*180/pi,'r');
plot(N,  etalogg(5,N)*180/pi,'b');
hold off
title('Pos');
legend('eta(4)','eta(5)');
subplot(2,1,2)
hold on;
plot(N,  nulogg(4,N)*180/pi,'r');
plot(N,  nulogg(5,N)*180/pi,'b');
hold off
title('vel');
legend('nu(4)','nu(5)');
suptitle('Load angle');
xlabel('loop');