%% arclength.m
%
% this is a function to compute the arclength along a 3D curve
%
% - written by Dimitri Lezcano

% simple arclength integration
function [L, varargout] = arclength(pts)
    dpts = diff(pts, 1, 1); % pts[i+1] - pts[i]
    
    dl = vecnorm(dpts, 2, 2); % ||dpts||
    
    L = sum(dl);
    
    varargout{1} = dl; % ds
    varargout{2} = [0; cumsum(dl)]; % s
    
end