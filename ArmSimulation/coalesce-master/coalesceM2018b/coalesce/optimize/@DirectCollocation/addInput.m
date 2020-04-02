function expression = addInput(this, varargin)
%ADDINPUT Add input to trajectory optimization problem.
%
% Description:
%   Wrapper for addVariable method that also adds variable to input list.
%
% Copyright 2013-2014 Mikhail S. Jones

  % Add input variable to problem
  expression = this.addVariable(varargin{:});

  % Add variable to input variable reference array
  this.inputVariable(end+1,numel(this.timeVariable)) = expression;
end % addInput
