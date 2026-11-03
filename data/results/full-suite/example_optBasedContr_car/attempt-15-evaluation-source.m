function [cost,c] = jointOptEvaluation(x,Opts,feasibilityTolerance)
% JOINTOPTEVALUATION - shared reachable-set objective and inequalities.
% [cost,c] = jointOptEvaluation(x,Opts) computes the same reach sequence as
% costFun/conFun, retaining both outputs to avoid duplicate reach calls.
% Opts.evaluationKey uniquely identifies one initialized controller problem.
% The caller audits local probes and independently verifies the final point.
%
%------------------------------------------------------------------
% This file is part of <a href="matlab:docsearch aroc">AROC</a>, a Toolbox for Automatic Reachset-
% Optimal Controller Synthesis developed at the Chair of Robotics, 
% Artificial Intelligence and Embedded Systems, 
% Technische Universitaet Muenchen. 
%
% For updates and further information please visit <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
%
% More Toolbox Info by searching <a href="matlab:docsearch aroc">AROC</a> in the Matlab Documentation
%
%------------------------------------------------------------------
% Authors:      Ivan Hernandez, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

% Local reproduction optimization: reuse the same reach computation for
% the objective and constraints. A unique call key prevents cross-model reuse.
if nargin<3, feasibilityTolerance=[]; end
persistent previousKey previousWeights previousCost previousConstraints previousTolerance
if ~isempty(previousKey) && strcmp(previousKey,Opts.evaluationKey) && isequal(previousWeights,x) && isequal(previousTolerance,feasibilityTolerance)
    cost=previousCost;c=previousConstraints;return;
end
cost=Inf;
    c = [];

    % get options for reachability analysis
    options = Opts.ReachOpts;
    params = Opts.ReachParams;
    
    params.R0 = cartProd(Opts.R0,zonotope(Opts.x0));
    
    % loop over all time steps
    for i = 1:Opts.N
        for j = 1:Opts.Ninter

            k = (i-1)*Opts.Ninter + j;
        
            % get reference input
            u_ref = Opts.uc(:,k);
                
            % compute closed loop system
            if Opts.isLin
                
                % get feedback matrix
                K = compFeedbackMatrix(x(:,i),Opts.linModel.A, ...
                                       Opts.linModel.B);
                
                % construct linear system
                sys = closedLoopSystemLin(Opts.linModel.A,Opts.linModel.B, ...
                                          Opts.linModel.D,Opts.linModel.c, ...
                                          K,u_ref,Opts.V);
                                      
                % compute the reachable set
                R = reach(sys,params,options);
                
            else
                
                % get feedback matrix
                K = compFeedbackMatrix(x(:,i),Opts.linModel.A{k}, ...
                                       Opts.linModel.B{k});
                
                p = [u_ref;reshape(K,[Opts.nx*Opts.nu,1])];
                params.paramInt = p;
                
                % compute the reachable set
                try
                    R = reachNonlinear(Opts.sys,params,options);
                catch
                    if isempty(Opts.stateCon)
                        M = size(Opts.inputCon.A,1);
                    else
                        M = size(Opts.inputCon.A,1)+size(Opts.stateCon.A,1);
                    end
                    c = inf * ones(M*Opts.N*Opts.Ninter*Opts.reachSteps,1);
                    return;
                end
            end
            
            % loop over all reachability time steps
            cTemp = checkConstraints(R.timeInterval.set,K,u_ref,Opts);
            c = [c;cTemp];
            % Concatenation cannot remove a violated prefix constraint.
            % Barrier-only calls can reject here; ordinary calls remain full.
            if ~isempty(feasibilityTolerance) && ...
                    (any(~isfinite(cTemp(:))) || any(cTemp(:)>feasibilityTolerance))
                return;
            end

            
            % update initial set
            params.R0 = R.timePoint.set{end};
        end
    end
    
    % assign output values

int=interval(project(params.R0,1:Opts.nx));
r=rad(int);cFinal=center(int);
cost=sum(r.^2./rad(interval(Opts.R0)).^2)+sum(abs(Opts.xf-cFinal));
previousKey=Opts.evaluationKey;previousWeights=x;
previousCost=cost;previousConstraints=c;previousTolerance=feasibilityTolerance;
end
