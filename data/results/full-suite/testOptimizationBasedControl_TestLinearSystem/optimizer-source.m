function [objContr,res,solverInfo] = optimizationBasedControl(benchmark,Param,varargin)
% OPTIMIZATIONBASEDCONTROL - Implementation of a optimazation based control
%                            algorithm
%
% Syntax:
%       [objContr,res] = OPTIMIZATIONBASEDCONTROL(benchmark,Param)
%       [objContr,res] = OPTIMIZATIONBASEDCONTROL(benchmark,Param,Opts)
%       [objContr,res] = OPTIMIZATIONBASEDCONTROL(benchmark,Param,Opts,Post)
%
% Description:
%       Offline-phase computations for a control algorithm that is based on
%       the optimization over reachable sets
%
% Input Arguments:
%
%       -benchmark:    name of the considered benchmark model (see
%                      "aroc/benchmarks/...")
%       -Param:             a structure containing the benchmark parameters
%
%           -.R0:           initial set of states (class: interval)
%           -.xf:           goal state
%           -.tFinal:       final time after which the goal state should be
%                           reached
%           -.U:            set of admissible control inputs (class:
%                           interval)
%           -.W:            set of uncertain disturbances (class: interval 
%                           or zonotope)
%           -.V:            set of measurement errors (class: interval or
%                           zonotope)
%           -.X:            set of state constraints (class: polytope)
%
%       -Opts:              a structure containing the algorithm settings
%
%           -.N:                number of time steps
%                               [{5} / positive integer]
%           -.Ninter:           number of intermediate time steps
%                               [{1} / positive integer]                            
%           -.reachSteps:       number of reachability steps during one
%                               time step (optimization)
%                               [{10} / positive integer]
%           -.reachStepsFin:    number of reachability steps during one
%                               time step (final reachable set computation)
%                               [{100} / positive integer]
%           -.maxIter:          maximum number of iterations for
%                               optimization with fmincon 
%                               [{15} / positive integer]
%           -.bound:            scaling factor between upper and lower
%                               bound of the weigthing matrices
%                               [{1000} / positive scalar]
%
%           -.refTraj.Q:    state weighting matrix for the cost function of
%                           optimal control problem (dimension:[nx,nx])
%           -.refTraj.R:    input weighting matrix for the cost function of
%                           optimal control problem (dimension:[nu,nu])
%           -.refTraj.x:    user provided reference trajectory
%                           (dimension: [nx,Opts.N + 1])
%           -.refTraj.u     inputs for the user provided reference
%                           trajectory (dimension: [nu,Opts.N])
%
%           -.cora.alg:                 reachability algorithm (nonlinear)
%                                       [{'lin'} / 'poly']
%           -.cora.linAlg:              reachability algorithm (linear)
%                                       [{'standard'} / 'fromStart' / 
%                                        'wrapping-free' / 'adap']
%           -.cora.tensorOrder:         taylor order for the abstraction of
%                                       the nonlinear function
%                                       [{2} / 3]
%           -.cora.taylorTerms:         taylor order for computing e^At
%                                       [{10} / positive integer]
%           -.cora.zonotopeOrder:       upper bound for the zonotope order
%                                       [{50} / positive integer]
%           -.cora.errorOrder:          upper bound for the zonotope order
%                                       before comp. the abstraction error
%                                       [{5} / positive integer]
%           -.cora.intermediateOrder:   upper bound for the zonotope order
%                                       during internal computations
%                                       [{30} / positive integer]
%           -.cora.error:               uppper-bound for Hausdorff distance
%                                       (for cora.linAlg = 'adap' only)
%
%       -Post:      function handle to the postprocessing function that is
%                   used to compute the occupancy set
%
% Output Arguments:
%
%       -objContr:  resulting controller storing the data computed during
%                   the offline phase (class: objOptBasedContr)
%       -res:       results object storing the computed reachable set and
%                   the center trajectory
%
% See Also:
%       objOptBasedContr
%
% References:
%       * *[1] Schuermann et al. (2017)*, Optimal Control of Sets of 
%              Solutions to Formally Guarantee Constraints of Disturbed 
%              Linear Systems
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


%% Initialization

Opts = [];
if nargin > 2
   Opts = varargin{1}; 
end

Opts = initialization(benchmark,Param,Opts);   
     
     
%% Center Trajectory

sys = Opts.funHandle;

[xCenter,uCenter] = centerTrajectory(sys,Opts.CentOpts);
    
Opts.xc = xCenter;
Opts.uc = uCenter;


%% Algorithm

% initial value for optimiaztion variables
xInit = ones((Opts.nx+Opts.nu-1),Opts.N);
warmStartFile = getenv('AROC_OPTIM_WARMSTART');
if ~isempty(warmStartFile)
    initial = load(warmStartFile,'xInit');
    assert(isfield(initial,'xInit') && isequal(size(initial.xInit),size(xInit)) ...
        && all(isfinite(initial.xInit(:))) && all(initial.xInit(:)>0),'Invalid saved initial weights');
    % Project the initial guess onto the original box, including roundoff at
    % exp(log(bound)). This does not change either optimization bound.
    xInit = min(Opts.bound,max(1/Opts.bound,initial.xInit));
end

% linearize the system
if ~Opts.isLin 
    Opts = linearizeSystem(Opts);
    initializeReach(Opts);
end

% The same reachable-set objective and constraints are retained. Direct
% search avoids finite-difference gradients of nonsmooth zonotope enclosures.
shape=size(xInit);
fun=@(z)costFun(reshape(exp(z),shape),Opts);
con=@(z)conFun(reshape(exp(z),shape),Opts);
lb=ones(shape)/Opts.bound;ub=ones(shape)*Opts.bound;
options=optimoptions('patternsearch','Algorithm','classic', ...
    'PollMethod','GPSPositiveBasis2N','UseParallel',true,'UseCompletePoll',true, ...
    'InitialMeshSize',1,'InitialPenalty',1e3,'ScaleMesh',false,'MaxIterations',Opts.maxIter, ...
    'MaxFunctionEvaluations',100000,'Display','iter', ...
    'ConstraintTolerance',1e-6,'MeshTolerance',1e-6, ...
    'FunctionTolerance',1e-6,'StepTolerance',1e-6, ...
    'OutputFcn',@(values,opts,state)checkpointPattern(values,opts,state,shape));
[zOpt,fval,exitflag,output]=patternsearch(fun,log(xInit(:)),[],[],[],[],log(lb(:)),log(ub(:)),con,options);
xOpt=min(ub,max(lb,reshape(exp(zOpt),shape)));
cOpt=conFun(xOpt,Opts);
actualViolation=max([0;cOpt(:);lb(:)-xOpt(:);xOpt(:)-ub(:)]);
solverInfo=struct('exitflag',exitflag,'objective',fval,'verifiedObjective',costFun(xOpt,Opts), ...
    'iterations',output.iterations,'functionEvaluations',output.funccount, ...
    'actualSearchPerformed',output.funccount>numel(xInit), ...
    'meshSize',output.meshsize,'constraintViolation',actualViolation, ...
    'reportedConstraintViolation',output.maxconstraint,'message',output.message, ...
    'maxIterations',Opts.maxIter,'parameterization','logarithmic positive LQR weights', ...
    'algorithm','patternsearch classic GPSPositiveBasis2N','savedInitialWeights',~isempty(warmStartFile), ...
    'weightLowerBound',1/Opts.bound,'weightUpperBound',Opts.bound, ...
    'minimumFinalWeight',min(xOpt(:)),'maximumFinalWeight',max(xOpt(:)), ...
    'constraintTolerance',options.ConstraintTolerance,'meshTolerance',options.MeshTolerance, ...
    'stepTolerance',options.StepTolerance,'functionTolerance',options.FunctionTolerance, ...
    'initialMeshSize',options.InitialMeshSize,'initialPenalty',options.InitialPenalty,'scaleMesh',options.ScaleMesh);
traceDir=getenv('AROC_OPTIM_TRACE_DIR');
if ~isempty(traceDir)
    save(fullfile(traceDir,'solver-diagnostics.mat'),'solverInfo','xOpt','output');
    fid=fopen(fullfile(traceDir,'solver-diagnostics.json'),'w');fprintf(fid,'%s',jsonencode(solverInfo,PrettyPrint=true));fclose(fid);
end
if exitflag<=0
    warning('AROC:OptimizationNotConverged','Local optimizer did not terminate successfully: exitflag %d.',exitflag);
end

assert(solverInfo.actualSearchPerformed,'AROC:NoSearchPerformed','Solver terminated without sufficient actual polling.');

% compute the reachable set for the optimized feedback matrix
[res,Rcont,R,K] = computeFinalReachSet(xOpt,Opts);

% check if constraints are satisfied
if ~res
   error('Failed to find a feasible solution!'); 
end


%% Assign output values

% compute the occupancy set
occSet = [];

if nargin > 3
   post = varargin{2};
   occSet = compOccupancySet(Rcont,post);
end

% bring parameters to the correct structure
if Opts.isLin
    dyn = Opts.linModel;
else
    dyn = Opts.funHandle;
end

contrLaw.K = K;
contrLaw.u_ref = uCenter;
contrLaw.N = Opts.N;
contrLaw.Ninter = Opts.Ninter;

% construct controller object
objContr = objOptBasedContr(dyn,R{end},contrLaw,Param,occSet);  

% construct results object
res = results(Rcont,R,Opts.xc);

end


% Auxiliary Functions -----------------------------------------------------

function [stop,options,optchanged]=checkpointPattern(values,options,state,shape)
stop=false;optchanged=false;traceDir=getenv('AROC_OPTIM_TRACE_DIR');
if ~isempty(traceDir)
    z=values.x;x=reshape(exp(z),shape);
    save(fullfile(traceDir,'optimizer-checkpoint.mat'),'z','x','values','state');
end
end

function Opts = initialization(benchmark,Param,Opts)

    % initialize general settings
    Opts = initOpts(mfilename,benchmark,Opts,Param);
    
    % initialize algorithm dependent settings
    Opts.uMin = infimum(Param.U);
    Opts.uMax = supremum(Param.U);
    Opts.x0 = center(Opts.R0);
    
    if isfield(Opts,'linModel')
       Opts.isLin = 1;
    else
       Opts.isLin = 0;
    end

    Ntot = Opts.N * Opts.Ninter;
    Opts.ReachParams.tFinal = Opts.tFinal/Ntot;
    if ~Opts.isLin || ~strcmp(Opts.ReachOpts.linAlg,'adap')
        Opts.ReachOpts.timeStep = (Opts.tFinal/Ntot)/ Opts.reachSteps; 
    end
    if ~Opts.isLin
        Opts.ReachOpts.intermediateTerms = 4; 
    end
    
    Opts.CentOpts.N = Ntot;               % number ref. traj. time steps
    Opts.CentOpts.dt = Opts.tFinal/Ntot;   % size center traj. time step

    % store matrices for inequality constraints for input and state con
    if isfield(Param,'X')
       if ~isa(Param.X,'polytope')
           Param.X = polytope(Param.X);
       end
       Opts.stateCon.A = Param.X.A;
       Opts.stateCon.b = Param.X.b;
    else
       Opts.stateCon = []; 
    end
    
    Opts.inputCon.A = [eye(Opts.nu);-eye(Opts.nu)];
    Opts.inputCon.b = [Opts.uMax;-Opts.uMin];
    
    % extend the disturbance set by the set of measurement errors
    if ~representsa(Opts.V,'emptySet')
        Opts.ReachParams.U = cartProd(Opts.ReachParams.U,Opts.V);
    end
    
    % create nonlinear parameter system object for the closed loop
    if ~Opts.isLin
        
        name = ['AROCoptBased',benchmark,Opts.ReachOpts.alg, ...
                num2str(Opts.ReachOpts.tensorOrder)];
            
        if representsa(Opts.V,'emptySet')
            funHan = @(x,u,p) closedLoopSystemNonlin(x,u,p,...
                                  Opts.funHandle,Opts.nx,Opts.nu,Opts.nw);
        
            Opts.sys = nonlinParamSys(name,@(x,u,p) funHan(x,u,p), ...
                                  2*Opts.nx,Opts.nw,(Opts.nx+1)*Opts.nu);
        else
            funHan = @(x,u,p) closedLoopSystemNonlinMeasErr(x,u,p,...
                                  Opts.funHandle,Opts.nx,Opts.nu,Opts.nw);
        
            Opts.sys = nonlinParamSys(name,@(x,u,p) funHan(x,u,p), ...
                            2*Opts.nx,Opts.nw+Opts.nx,(Opts.nx+1)*Opts.nu);
        end
    end
end

function Opts = linearizeSystem(Opts)
% compute linearize models for each time step (required to obtain initial
% feedback matrices with LQR control)

    % calculate the jacobian matrix symbolically
    x = sym('x',[Opts.nx,1]);
    u = sym('u',[Opts.nu,1]);
    w = sym('w',[Opts.nw,1]);

    model = Opts.funHandle(x,u,w);

    Jx = jacobian(model,x);
    Ju = jacobian(model,u);

    Afun = matlabFunction(Jx,'Vars',{x,u,w});
    Bfun = matlabFunction(Ju,'Vars',{x,u,w});
    
    Opts.linModel.A = cell(Opts.N*Opts.Ninter,1);
    Opts.linModel.B = cell(Opts.N*Opts.Ninter,1);

    % loop over all center trajectory time steps
    for i = 1:Opts.N
        for j = 1:Opts.Ninter

            k = (i-1)*Opts.Ninter + j;

            % linearize the system in the middle of the time step
            x_ = 0.5*(Opts.xc(:,k) + Opts.xc(:,k+1));
            u_ = Opts.uc(:,k);
            w_ = zeros(Opts.nw,1);
    
            Opts.linModel.A{k} = Afun(x_,u_,w_);
            Opts.linModel.B{k} = Bfun(x_,u_,w_);
        end
    end
end

function initializeReach(Opts)
% exectue reachability analysis once to generate the files for tensors in
% CORA

    options = Opts.ReachOpts;
    params = Opts.ReachParams;
    params.R0 = cartProd(Opts.R0,zonotope(Opts.x0));
    params.paramInt = zeros((Opts.nx+1)*Opts.nu,1);
    
    R = reach(Opts.sys,params,options);
end
