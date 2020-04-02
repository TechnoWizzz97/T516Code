function plot(obj)
%PLOT Plot the optimization solution time histories.
%
% Syntax:
%   obj.plot
%
% Description:
%   Plots all optimized state trajectories versus time in one plot and all
%   control inputs in another.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Get time response
	response = obj.getResponse;

	% Plot response
	response.plot;
end % plot
