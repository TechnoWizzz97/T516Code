%RUNUNITTESTS Run all example unit tests.
%
% Copyright 2014 Mikhail S. Jones

% Clean up the workspace
clear all; close all; clc;

% Currently, this script only runs on UNIX systems
if isunix
	% Find all unit test scripts in examples directory
	[status, str] = unix('find ./ -iname "unittest.m"');

	% Separate relative paths into cell array
	files = regexp(str, '\n', 'split');

	% Ignore the last entry because it is empty
	files = files(1:end-1);

	% Pre-allocate pass-fail results array
	result(numel(files)) = false;

	% Loop through each test
	for i = 1:numel(files)
		try
			% Attempt to run test
			t0 = tic;
			run(files{i});
			time(i) = toc(t0);

			% If we got here than we passed
			result(i) = true;
		catch exception
			% Otherwise it failed
			result(i) = false;
		end % try

		% Clean up the workspace
		drawnow; close all; clc; drawnow;
	end % for

	% Display passed tests
	fprintf('Passed Tests:\n')
	if all(~result)
		fprintf('  - None\n');
	else
		for i = find(result)
			fprintf('  - %s - %f sec\n', files{i}, time(i));
		end % for
	end % if

	% Display failed tests
	fprintf('\nFailed Tests:\n')
	if all(result)
		fprintf('  - None\n');
	else
		fprintf('  - %s\n', files{~result});
	end % if

else
	error('coalesce:examples:runAllExamples', ...
		'runAllExamples only works on UNIX systems for now.');
end % if
