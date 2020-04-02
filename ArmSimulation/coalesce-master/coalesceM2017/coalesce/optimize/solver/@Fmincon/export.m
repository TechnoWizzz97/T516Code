function export(this)
%EXPORT Export optimization functions for MATLAB FMINCON.
%
% Copyright 2013-2014 Mikhail S. Jones

	% User feedback
	fprintf('Exporting FMINCON functions... \n');

	% Number of variables, constraints, and objectives
	nVars = this.nlp.numberOfVariables;
	nCons = this.nlp.numberOfConstraints;
	nObjs = this.nlp.numberOfObjectives;

	% Split equality and inequality constraints
	equality = [];
	for k = 1:numel(this.nlp.constraint)
		equality(k) = all(this.nlp.constraint(k).lowerBound == this.nlp.constraint(k).upperBound);
	end % for
	equality = logical(equality);

	% Write user function
	gen = MatlabFunctionGenerator({}, {'A', 'b', 'Aeq', 'beq'}, 'fminconUser');
	gen.writeHeader;
	gen.writeExpression(ConstantNode.empty, 'A');
	gen.writeIndex({}, 'b');
	gen.writeExpression(ConstantNode.empty, 'Aeq');
	gen.writeIndex({}, 'beq');
	gen.writeFooter;

	% Write objective function
	gen = MatlabFunctionGenerator({'var'}, {'f', 'g'}, 'fminconObj');
	gen.writeHeader;
	f = vertcat(this.nlp.objective.expression);
	gen.writeExpression(f, 'f');
	[ig, jg, sg] = f.jacobian;
	gen.writeIndex(ig, 'ig');
	gen.writeIndex(jg, 'jg');
	gen.writeExpression(sg, 'sg');
	fprintf(gen.fid, '\tg = sparse(ig, jg, sg, %d, %d);\n\n', nObjs, nVars);
	gen.writeFooter;

	% Write nonlinear constraint function
	gen = MatlabFunctionGenerator({'var'}, {'c', 'ceq', 'G', 'Geq'}, 'fminconNonlcon');
	gen.writeHeader;
	if any(~equality)
		c = vertcat(this.nlp.constraint(~equality).expression);
	else
		c = ConstantNode.empty;
	end % if
	gen.writeExpression(c, 'c');
	if any(equality)
		ceq = vertcat(this.nlp.constraint(equality).expression);
	else
		ceq = ConstantNode.empty;
	end % if
	gen.writeExpression(ceq, 'ceq');
	fprintf(gen.fid, '\tif nargout > 2\n');
	[jG, iG, sG] = c.jacobian;
	gen.writeIndex(iG, 'iG');
	gen.writeIndex(jG, 'jG');
	gen.writeExpression(sG, 'sG');
	fprintf(gen.fid, '\tG = sparse(iG, jG, sG, %d, %d);\n\n', nVars, sum([c.length]));
	[jGeq, iGeq, sGeq] = ceq.jacobian;
	gen.writeIndex(iGeq, 'iGeq');
	gen.writeIndex(jGeq, 'jGeq');
	gen.writeExpression(sGeq, 'sGeq');
	fprintf(gen.fid, '\tGeq = sparse(iGeq, jGeq, sGeq, %d, %d);\n\n', nVars, sum([ceq.length]));
	fprintf(gen.fid, '\tend %% if\n');
	gen.writeFooter;
end % export
