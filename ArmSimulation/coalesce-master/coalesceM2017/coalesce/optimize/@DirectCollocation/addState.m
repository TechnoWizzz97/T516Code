function expression = addState(this, varargin)
%ADDSTATE Add state to trajectory optimization problem.
%
% Description:
%   Wrapper for addVariable method that also adds variable to state list.
%
% Copyright 2013-2014 Mikhail S. Jones

  % Add input variable to problem
  expression = this.addVariable(varargin{:});

  % Add variable to state variable reference array
  this.stateVariable(end+1,numel(this.timeVariable)) = expression;
end % addState
