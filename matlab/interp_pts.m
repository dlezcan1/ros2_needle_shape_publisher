%% interp_pts.m
%
% this is a function to interpolate the points along a 3D curve
%
% - written by: Dimitri Lezcano

% interpolate nurbs-pts for standardized ds
function [pts_std, varargout] = interp_pts(pts, s_interp)
    [~, ~, s_lu] = arclength(pts);
    s_interp(s_interp > max(s_lu)) = max(s_lu); % cap the lookup arclength
    
    % look-up for interpolation
    x_lu = pts(:,1); 
    y_lu = pts(:,2); 
    z_lu = pts(:,3);
    
    % interpolation
    x_interp = interp1(s_lu, x_lu, s_interp);
    y_interp = interp1(s_lu, y_lu, s_interp);
    z_interp = interp1(s_lu, z_lu, s_interp);
    
    % combine the output
    pts_std = [x_interp', y_interp', z_interp'];
    varargout{1} = s_interp; % return the interpolation arclength just in case

end