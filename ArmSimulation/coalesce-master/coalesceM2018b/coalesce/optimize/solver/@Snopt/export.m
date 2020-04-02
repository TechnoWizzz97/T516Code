function export(this)
%EXPORT Export optimization functions for SNOPT.
%
% Copyright 2013-2014 Mikhail S. Jones

	% User feedback
	fprintf('Exporting SNOPT functions... \n');

	% Number of variables, constraints, and objectives
	nVars = this.nlp.numberOfVariables;
	nCons = this.nlp.numberOfConstraints;
	nObjs = this.nlp.numberOfObjectives;

	% Compute combined Jacobian of objective and constraints
	f = vertcat(this.nlp.objective.expression, this.nlp.constraint.expression);
	[i, j, s] = f.jacobian;

	% Initialize sparse Jacobian indexes
	iA = {}; jA = {}; A = ConstantNode.empty;
	% iG = {}; jG = {}; G = ConstantNode.empty;
	iG = i; jG = j; G = s;

	% % Separate linear and nonlinear terms
	% for k = 1:numel(s)
	% 	isLinear = true;
	% 	[~,~,tmp] = s(k).jacobian;
	% 	for kk = 1:numel(tmp)
	% 		isLinear = isLinear && isempty(symvar(tmp(kk)));
	% 	end % for
	%
	% 	if isLinear % TODO: Build into ExpressionNode
	% 		iA{k} = i{k}; jA{k} = j{k}; A(k) = s(k);
	% 	else
	% 		iG{k} = i{k}; jG{k} = j{k}; G(k) = s(k);
	% 	end % if
	% end % for

	% Write user function
	gen = MatlabFunctionGenerator({}, {'A', 'iAfun', 'jAvar', 'iGfun', 'jGvar'}, 'snoptUser');
	gen.writeHeader;
	gen.writeExpression(A, 'A');
	gen.writeIndex(iA, 'iAfun');
	gen.writeIndex(jA, 'jAvar');
	gen.writeIndex(iG, 'iGfun');
	gen.writeIndex(jG, 'jGvar');
	gen.writeFooter;

	% Write objective and constraint function
	gen = MatlabFunctionGenerator({'var'}, {'f', 'G'}, 'snoptUserFun');
	gen.writeHeader;
	gen.writeExpression(f, 'f');
	gen.writeExpression(G, 'G');
	gen.writeFooter;
end % export
