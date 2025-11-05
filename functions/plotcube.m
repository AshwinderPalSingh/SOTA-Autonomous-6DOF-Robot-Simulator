function h = plotcube(ax, origin, sz, color)
% PLOTCUBE - Plots a 3D cube (patch object)
%
%   Inputs:
%       ax     - The axes to plot on
%       origin - [x, y, z] of the cube's "bottom-front-left" corner
%       sz     - [width, length, height] of the cube
%       color  - The face color (e.g., 'r' or [0.8 0 0.2])
%
%   Outputs:
%       h      - The handle to the patch object

    % Vertices of a unit cube
    v = [0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 0 1; 1 0 1; 1 1 1; 0 1 1];
    
    % Scale and translate
    v = v .* sz + origin;
    
    % Faces
    f = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    
    % Plot the patch
    h = patch(ax, 'Vertices', v, 'Faces', f, 'FaceColor', color, 'FaceAlpha', 0.8);
end
