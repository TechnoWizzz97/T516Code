%FUNCTIONGENERATOR Function generation object.
%
% Copyright 2014 Mikhail S. Jones

classdef (Abstract = true) FunctionGenerator < handle

  % PROTECTED PROPERTIES ==================================================
  properties (SetAccess = protected)
    argsIn@cell vector = {}
    argsOut@cell vector = {}
    name@char vector = ''
    fid@double scalar = 1
  end % properties

  % ABSTRACT METHODS ======================================================
  methods (Abstract = true)
    writeHeader
    writeIndex
    writeExpression
    writeFooter
  end % methods

  % PUBLIC METHODS ========================================================
  methods
    function this = FunctionGenerator(argsIn, argsOut, name)
    %FUNCTIONGENERATOR Function generation object constructor.

      % TODO: Input argument checks

      % Set object properties
      this.argsIn = argsIn;
      this.argsOut = argsOut;
      this.name = name;
    end % FunctionGenerator
  end % methods
end % classdef
