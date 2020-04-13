function aboutCallback(this, varargin)
%ABOUTCALLBACK About callback function
%
% Copyright 2013-2014 Mikhail S. Jones

    % Create dialog box
    helpdlg({'Player provides playback settings, video/picture export tools, save/open functionality and renderer options for Scene class objects.', ...
        '', ...
        'Author: Mikhail Jones', ...
        sprintf('Version: %s', this.version), ...
        'Copyright 2013-2014 Mikhail Jones', ...
        ''}, ...
        'About');
end % aboutCallback