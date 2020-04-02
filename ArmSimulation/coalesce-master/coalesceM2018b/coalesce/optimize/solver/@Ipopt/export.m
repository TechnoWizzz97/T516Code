function export(this)
%EXPORT Export optimization functions for IPOPT.
%
% Copyright 2013-2014 Mikhail S. Jones

	% User feedback
	fprintf('Exporting IPOPT functions... \n');

	% Number of variables, constraints, and objectives
	nVars = this.nlp.numberOfVariables;
	nCons = this.nlp.numberOfConstraints;
	nObjs = this.nlp.numberOfObjectives;

	% Write objective function
	gen = MatlabFunctionGenerator({'var'}, {'f'}, 'ipoptObjective');
	gen.writeHeader;
	f = vertcat(this.nlp.objective.expression);
	gen.writeExpression(f, 'f');
	gen.writeFooter;

	% Write objective gradient function
	gen = MatlabFunctionGenerator({'var'}, {'g'}, 'ipoptGradient');
	gen.writeHeader;
	[ig, jg, sg] = f.jacobian;
	gen.writeIndex(ig, 'ig');
	gen.writeIndex(jg, 'jg');
	gen.writeExpression(sg, 'sg');
	fprintf(gen.fid, '\tg = sparse(ig, jg, sg, %d, %d);\n\n', nObjs, nVars);
	gen.writeFooter;

	% Write constraint function
	gen = MatlabFunctionGenerator({'var'}, {'c'}, 'ipoptConstraints');
	gen.writeHeader;
	if ~isempty(this.nlp.constraint)
		c = vertcat(this.nlp.constraint.expression);
	else
		c = ConstantNode.empty;
	end % if
	gen.writeExpression(c, 'c');
	gen.writeFooter;

	% Write constraint jacobian function
	gen = MatlabFunctionGenerator({'var'}, {'J'}, 'ipoptJacobian');
	gen.writeHeader;
	if ~isempty(this.nlp.constraint)
		[iJ, jJ, sJ] = c.jacobian;
	else
		iJ = {}; jJ = {}; sJ = ConstantNode.empty;
	end % if
	gen.writeIndex(iJ, 'iJ');
	gen.writeIndex(jJ, 'jJ');
	gen.writeExpression(sJ, 'sJ');
	fprintf(gen.fid, '\tJ = sparse(iJ, jJ, sJ, %d, %d);\n\n', nCons, nVars);
	gen.writeFooter;

	% Write constraint jacobian structure function
	gen = MatlabFunctionGenerator({'var'}, {'J'}, 'ipoptJacobianStructure');
	gen.writeHeader;
	gen.writeIndex(iJ, 'iJ');
	gen.writeIndex(jJ, 'jJ');
	fprintf(gen.fid, '\tsJ = 1 + zeros(1,%d);\n', sum([sJ.length]));
	fprintf(gen.fid, '\tJ = sparse(iJ, jJ, sJ, %d, %d);\n\n', nCons, nVars);
	gen.writeFooter;
end % export
