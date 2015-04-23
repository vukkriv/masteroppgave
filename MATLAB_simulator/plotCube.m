
function a = plotCube(pos, orientation, dimension, reference, color)
    [vert, face] = cube(pos, orientation, dimension, reference);
    
    if nargin < 5
        color = 'k';
    end
    
    a = patch('Vertices', vert, 'Faces', face,  ...
            'FaceVertexCData',[],'FaceColor',color);
end

function [vert, face] = cube(pos, orientation, dimension, reference)

    if nargin < 4
        reference = 'center';
    end
    
    lengths = dimension;
    offset = zeros(1,3);
    if strcmp(reference, 'zero')
        %nop;
    elseif strcmp(reference, 'center')
        offset = - 0.5*([lengths(1) lengths(2) lengths(3)]);
    elseif strcmp(reference, 'xycenter')
        offset = - 0.5*([lengths(1) lengths(2) 0]);
    elseif strcmp(reference, 'zycenter')
        offset = - 0.5*([0 lengths(2) lengths(3)]);
    end
    
    
    
    
    cube_vertices =[0 0 0; 
                    0 1 0; 
                    1 1 0; 
                    1 0 0; 
                    0 0 1; 
                    0 1 1; 
                    1 1 1; 
                    1 0 1] * diag(lengths);
                

    phi = orientation(1);
    theta = orientation(2);
    psi = orientation(3);

    pre_orient = 0;
    doPreOrient = 0;
    if length(orientation) > 3
        doPreOrient = 1;
        pre_orient = orientation(4:6);
    end
                
    for row = 1:size(cube_vertices, 1)
           cube_vertices(row, :) = cube_vertices(row, :) + offset;
    end
    
    
    for row = 1:size(cube_vertices, 1)
           if doPreOrient
               cube_vertices(row, :) = (Rotmat(pre_orient(1), ...
                   pre_orient(2), ...
                   pre_orient(3) ...
                                                )*cube_vertices(row, :)')';
           end
           cube_vertices(row, :) = (Rotmat(phi, theta, psi)*cube_vertices(row, :)')';
    end
    
    %cube_vertices= cube_vertices * Rotmat(phi, theta, psi);
    
    
    
    for row = 1:size(cube_vertices, 1) 
        cube_vertices(row, :) = cube_vertices(row, :) + pos' ;
    end
    
    
    vert = cube_vertices;
    face = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
    
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