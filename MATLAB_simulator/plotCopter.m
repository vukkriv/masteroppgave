function [ a ] = plotCopter( pos, orient )
%PLOTCOPTER Summary of this function goes here
%   Detailed explanation goes here

 
    phi = orient(1);
    theta = orient(2);
    psi = orient(3);
    
    Rnb = Rotmat(phi, theta, psi);
    
    % Ait! Plot a hexa-copter. 
    radii = 0.25;
    cboxDim  = [0.1, 0.1, 0.02]';
    cbox2Dim = [0.06, 0.06, -0.05]';
    
    % Bars, size
    barsDim  = [radii 0.015 0.015]';
    barsColours = ['b', 'k', 'k', 'k', 'k', 'b'];
    barsAngles = (0:(2*pi)/6:2*pi-0.01)+pi/6;
    barXOffset = 0.06;
 
    % Prepare array
    a = [];
    
    % Plot two center boxes
    a = [a plotCube(pos, orient, cboxDim, 'center', 'k') ];
    a = [a plotCube(pos, orient, cbox2Dim, 'xycenter', 'k')];
    
    % Plot the bars
    for i = 1:length(barsAngles)
        
       orient_i = orient + Rnb*[0 0 barsAngles(i)]';
       %orient_i(3) = orient_i(3) + barsAngles(i);
       orient_i = [orient' 0 0 barsAngles(i)]';
       pos_i = pos + Rotmat(phi, theta, psi)*Rotmat(0,0, barsAngles(i))*[barXOffset 0 0]';
       a = [a ...
           plotCube(pos_i, orient_i, barsDim, 'zycenter', barsColours(i)) ...
           ];
       
       % Plot helis
       rotor_pos_body = Rotmat(0,0, barsAngles(i)) * [barXOffset + radii-0.02 0 -0.03]';
       rotor_pos_i = pos + Rnb * rotor_pos_body;
       %rotor_pos_i = pos;
       rotor_orient_i = orient_i;
       rotor_size = 0.09;
       a = [a ...
            plotAnnulus(rotor_pos_i, rotor_orient_i, rotor_size, '', 'r');
        ];
       
    end
    




    


function a = plotAnnulus(pos, orientation, radi, reference, color)

    phi = orientation(1);
    theta = orientation(2);
    psi = orientation(3);
    
    Rpre = eye(3);
    if length(orientation) > 3
        Rpre = Rotmat(orientation(4),orientation(5),orientation(6));
    end

    % Make inner and outer boundaries
    t = linspace(0,2*pi);


    % Define a single face, and a lot of vertices
   
    
    rin  = radi;

        
    x = rin * cos(t);
    y = rin * sin(t);
    z = zeros(1, length(t));
    
    in = [x' y' z'];
    
    for row = 1:size(in, 1)
        in(row, :) = (  Rnb * Rpre* in(row, :)')';
        
    end
    
    
    



    a = plot3(in(:,1)+pos(1), in(:,2)+pos(2), in(:,3)+pos(3), 'LineWidth', 2, 'Color', color);
    
    


end




function R = Rotmat(phi, theta, psi)
% Rnb
cphi = cos(phi);
sphi = sin(phi);
cth  = cos(theta);
sth  = sin(theta);
cpsi = cos(psi);
spsi = sin(psi);
 
R = [...
   cpsi*cth  -spsi*cphi+cpsi*sth*sphi  spsi*sphi+cpsi*cphi*sth
   spsi*cth  cpsi*cphi+sphi*sth*spsi   -cpsi*sphi+sth*spsi*cphi
   -sth      cth*sphi                  cth*cphi ];

end


end