function initializeGui(this)
%INITIALIZEGUI Initialize graphical user
%
% Copyright 2013-2014 Mikhail S. Jones

	% Create new figure window and return handle
	this.handles.fig = figure(...
		'Color', [1 1 1], ...
		'CreateFcn', 'movegui west', ...
		'DefaultTextInterpreter', 'latex', ...
		'DefaultLineLineWidth', 2, ...
		'DefaultAxesLineWidth', 2, ...
		'DefaultPatchLineWidth', 2, ...
		'DeleteFcn', @this.deleteCallback, ...
		'DockControls', 'on', ...
		'KeyPressFcn', '', ...
		'Menubar', 'none', ...
		'Name', sprintf('Player v.%s', this.version), ...
		'PaperPositionMode', 'auto', ...
		'Position', this.options.size + [0 0 100 100], ...
		'Renderer', this.options.renderer, ...
		'ResizeFcn', @this.resizeCallback, ...
		'Toolbar', 'none', ...
		'Units', 'pixels', ...
		'Visible', 'off');

	% Create user interface menu and return handles
	this.handles.uimenu.media = uimenu(...
		'Label', 'Media', ...
		'Parent', this.handles.fig);
		this.handles.uimenu.open = uimenu(...
			'Accelerator', 'O', ...
			'Callback', @this.openCallback, ...
			'Label', 'Open...', ...
			'Parent', this.handles.uimenu.media);
		this.handles.uimenu.saveAs = uimenu(...
			'Accelerator', 'S', ...
			'Callback', @this.saveAsCallback, ...
			'Enable', 'off', ...
			'Label', 'Save As...', ...
			'Parent', this.handles.uimenu.media);
		this.handles.uimenu.close = uimenu(...
			'Callback', @this.closeCallback, ...
			'Label', 'Close', ...
			'Parent', this.handles.uimenu.media, ...
			'Separator', 'on');
		this.handles.uimenu.quit = uimenu(...
			'Accelerator', 'Q', ...
			'Callback', @this.quitCallback, ...
			'Label', 'Quit', ...
			'Parent', this.handles.uimenu.media);
	this.handles.uimenu.playback = uimenu(...
		'Label', 'Playback', ...
		'Enable', 'off', ...
		'Parent', this.handles.fig);
		this.handles.uimenu.loop = uimenu(...
			'Callback', @this.loopCallback, ...
			'Label', 'Loop', ...
			'Parent', this.handles.uimenu.playback);
		this.handles.uimenu.speed = uimenu(...
			'Label', 'Speed', ...
			'Parent', this.handles.uimenu.playback, ...
			'Separator', 'on');
			this.handles.uimenu.faster = uimenu(...
				'Callback', @this.fasterCallback, ...
				'Label', 'Faster', ...
				'Parent', this.handles.uimenu.speed);
			this.handles.uimenu.normal = uimenu(...
				'Callback', @this.normalCallback, ...
				'Label', 'Normal', ...
				'Parent', this.handles.uimenu.speed);
			this.handles.uimenu.slower = uimenu(...
				'Callback', @this.slowerCallback, ...
				'Label', 'Slower', ...
				'Parent', this.handles.uimenu.speed);
		this.handles.uimenu.play = uimenu(...
			'Callback', @this.playCallback, ...
			'Label', 'Play', ...
			'Parent', this.handles.uimenu.playback, ...
			'Separator', 'on');
		this.handles.uimenu.stop = uimenu(...
			'Callback', @this.stopCallback, ...
			'Label', 'Stop', ...
			'Parent', this.handles.uimenu.playback);
		this.handles.uimenu.rewind = uimenu(...
			'Callback', @this.rewindCallback, ...
			'Label', 'Rewind', ...
			'Parent', this.handles.uimenu.playback);
		this.handles.uimenu.fastForward = uimenu(...
			'Callback', @this.fastForwardCallback, ...
			'Label', 'Fast Forward', ...
			'Parent', this.handles.uimenu.playback);
		this.handles.uimenu.nextFrame = uimenu(...
			'Callback', @this.nextFrameCallback, ...
			'Label', 'Next Frame', ...
			'Parent', this.handles.uimenu.playback, ...
			'Separator', 'on');
		this.handles.uimenu.previousFrame = uimenu(...
			'Callback', @this.previousFrameCallback, ...
			'Label', 'Previous Frame', ...
			'Parent', this.handles.uimenu.playback);
	this.handles.uimenu.video = uimenu(...
		'Label', 'Video', ...
		'Enable', 'off', ...
		'Parent', this.handles.fig);
		this.handles.uimenu.smoothing = uimenu(...
			'Callback', @this.smoothingCallback, ...
			'Label', 'Smoothing', ...
			'Parent', this.handles.uimenu.video);
		this.handles.uimenu.settings = uimenu(...
			'Label', 'Settings', ...
			'Parent', this.handles.uimenu.video, ...
			'Separator', 'on');
			this.handles.uimenu.snapshotSettings = uimenu(...
				'Callback', @this.displaySettingsCallback, ...
				'Label', 'Display', ...
				'Parent', this.handles.uimenu.settings);
			this.handles.uimenu.snapshotSettings = uimenu(...
				'Callback', @this.snapshotSettingsCallback, ...
				'Label', 'Snapshot', ...
				'Parent', this.handles.uimenu.settings);
			this.handles.uimenu.videoSettings = uimenu(...
				'Callback', @this.videoSettingsCallback, ...
				'Label', 'Video', ...
				'Parent', this.handles.uimenu.settings);
		this.handles.uimenu.takeSnapshot = uimenu(...
			'Callback', @this.takeSnapshotCallback, ...
			'Label', 'Take Snapshot', ...
			'Parent', this.handles.uimenu.video, ...
			'Separator', 'on');
		this.handles.uimenu.exportVideo = uimenu(...
			'Callback', @this.exportVideoCallback, ...
			'Label', 'Export Video', ...
			'Parent', this.handles.uimenu.video);
	this.handles.uimenu.help = uimenu(...
		'Label', 'Help', ...
		'Parent', this.handles.fig);
		this.handles.uimenu.contents = uimenu(...
			'Callback', @this.contentsCallback, ...
			'Label', 'Contents...', ...
			'Parent', this.handles.uimenu.help);
		this.handles.uimenu.about = uimenu(...
			'Callback', @this.aboutCallback, ...
			'Label', 'About', ...
			'Parent', this.handles.uimenu.help, ...
			'Separator', 'on');

	% Create seek bar slider and text
	this.handles.seekBarSlider = uicontrol(...
		'Callback', {@(src, event) seekCallback(this, src, event)}, ...
		'Enable', 'off', ...
		'SliderStep', [0 0], ...
		'Max', 2, ...
		'Min', 0, ...
		'Parent', this.handles.fig, ...
		'Position', [10 30 this.options.size(3)-20 20], ...
		'Style', 'slider',...
		'Value', 0);
	this.handles.seekBarText = uicontrol(...
		'Parent', this.handles.fig, ...
		'Position', [10 10 this.options.size(3)-20 20],...
		'String', 'Current Time: ', ...
		'Style', 'text');

	% Set figure to visible now that everything has been created
	set(this.handles.fig, 'Visible', 'on');
end % initializeGui
