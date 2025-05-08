% This script demonstrates the Reachset Model Predictive Control algorithm
% for the stirred tank reactor benchmark

% load benchmark parameter
Param = param_stirredTankReactor();
Param.x0 = [-0.3;-30];

% define algorithm options
Opts = [];

Opts.N = 5;                     % number of time steps
Opts.reachSteps = 1;            % number of reachability steps
Opts.tOpt = 9;                  % final time for optimization horizon
Opts.Q = diag([100,1]);         % state weighting matrix  
Opts.R = 0.9;                   % input weighting matrix
Opts.Qlqr = diag([1;1]);        % state weighting matrix (feedback control)
Opts.Rlqr = 100;                % input weighting matrix (feedback control)
Opts.scale = 0.9556;            % scaling factor for tightend input set
Opts.tComp = 0.54;              % allocated computation time
Opts.alpha = 0.1;               % contraction rate
Opts.maxIter = 50;              % max number of iterations for opt. control

% terminal region (see Sec. IV in Schuermann et al. (2018): "Reachset Model 
% Predictive Control for Disturbed Nonlinear Systems", CDC)
A = [-1.0000 0;1.0000 0;30.0000 -1.0000;66.6526 -4.8603;-66.6526 4.8603];
b = [0.3000;0.0620;11.8400;65.0000;15.0000];

Opts.termReg = polytope(A,b);
% execute model predictive control algorithm
res = reachsetMPC('stirredTankReactor',Param,Opts);

% visualization
figure; hold on; box on
plot(Opts.termReg,[1,2],'FaceColor',[100 182 100]./255,...
     'EdgeColor','none','FaceAlpha',0.8);
plotReach(res,[1,2],[.7 .7 .7]);
plotReachTimePoint(res,[1,2],'b');
plotSimulation(res,[1,2],'r');
xlabel('C_A'); ylabel('T');
xlim([-0.35,0.05]); ylim([-35,5]);
savefig('./MPC.fig');
% saveas(gcf, 'figure_name.svg'); % 保存为 SVG 格式
