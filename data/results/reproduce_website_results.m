% Reproduce the two numerical website figures (MATLAB script).
% First add AROC 2026, CORA v2026.0.1, YALMIP R20250626, and a conic
% solver to the MATLAB path. See README-results.txt in this directory.
% Original run: MATLAB R2024b Update 9, SeDuMi 1.3 for terminal regions,
% MATLAB linprog for linear MPC. Seed 20261019. Real-time checking disabled.
% AROC hard-codes MOSEK in terminalRegion/linSys/src/setOptsInternal.m.
% To match this run, change 'solver','mosek' to 'solver','sedumi' there.
% This changes the solver backend only. Keep a copy of the original file.
% With MOSEK installed, the original source can be used instead, but the
% numerical result will not be an exact reproduction of this solver setup.
set(groot,'defaultFigureVisible','off');
set(groot,'defaultAxesFontName','Arial','defaultAxesFontSize',12);
rng(20261019,'twister');
out = fullfile(pwd,'aroc-website-results');
if ~isfolder(out), mkdir(out); end
checkAROCdependencies;
report = struct('matlab',version,'seed',20261019,'terminalSolver','SeDuMi 1.3', ...
    'mpcSolver','MATLAB linprog','realTime',false);

% Linear MPC: the terminal polytope and benchmark match testLinSysMPC.m.
try
    Param = rmfield(param_doubleIntegrator(),'R0');
    Param.xf = zeros(2,1); Param.x0 = [3;-3];
    Opts = struct;
    Opts.termReg = polytope([1 0;0 1;-1 0;0 -1;1 1;-1 -1],[1;1;1;1;1.2;1.2]);
    Opts.tOpt = 8; Opts.N = 40; Opts.realTime = false;
    started = tic;
    result = linSysMPC('doubleIntegrator',Param,Opts);
    report.mpcSeconds = toc(started);
    report.mpcInputsPass = logical(checkSimInputs(result,Param.U));
    report.mpcTerminalPass = logical(checkFinalSimPointsInSet(result,Opts.termReg));
    assert(report.mpcInputsPass && report.mpcTerminalPass,'MPC checks failed');
    save(fullfile(out,'linear-mpc.mat'),'result','Param','Opts');
    f=figure('Position',[100 100 1000 680],'Color','white'); hold on; box on;
    plot(Opts.termReg,[1 2],'FaceColor',[.72 .86 .70],'EdgeColor',[.20 .45 .20]);
    plotReach(result,[1 2],[.74 .84 .96]);
    plotReachTimePoint(result,[1 2],[.10 .34 .72]);
    plotSimulation(result,[1 2],[.80 .22 .10]);
    plot(3,-3,'ko','MarkerFaceColor','k');
    xlabel('Position x_1'); ylabel('Velocity x_2');
    title('Linear MPC: double integrator'); grid on;
    subtitle('Blue: reachable sets | Red: simulation | Green: terminal region');
    exportgraphics(f,fullfile(out,'linear-mpc.png'),'Resolution',180);
    print(f,fullfile(out,'linear-mpc.svg'),'-dsvg'); close(f);
    report.mpcStatus = 'passed';
catch err
    report.mpcStatus = 'failed'; report.mpcError = getReport(err,'extended');
    disp(report.mpcError);
end
write_report(out,report);

% Terminal region: fixed zonotope, 10 steps of 0.1 s, bounded disturbances.
try
    yalmip('clear');
    Param = param_doubleIntegrator();
    Param.X = diag([10 4])*interval(-ones(2,1),ones(2,1));
    Opts = struct('N',10,'timeStep',.1,'terminalRegionType','fixed', ...
        'terminalRegionSet','zonotope','genMethod','spherical','nGenerators',10);
    Opts.Tdomain = Param.X;
    started=tic;
    T = computeTerminalRegion('doubleIntegrator','linSys',Param,Opts);
    report.terminalSeconds=toc(started);
    sims=T.simulateRandom(10,'extreme');
    T.verifyTrajectory(sims,1e-2);
    report.terminalTrajectories = length(sims);
    report.terminalChecksPass = true;
    for k=1:length(sims)
        assert(all(contains(Param.X,sims{k}.x,'exact',1e-2),'all'),'State constraint check failed');
        assert(all(contains(Param.U,sims{k}.u,'exact',1e-2),'all'),'Input constraint check failed');
        assert(contains(T.set,sims{k}.y(:,1),'exact',1e-2),'Initial set check failed');
        assert(contains(T.termSet,sims{k}.y(:,end),'exact',1e-2),'Terminal set check failed');
    end
    save(fullfile(out,'terminal-region.mat'),'T','sims','Param','Opts');
    f=figure('Position',[100 100 1000 680],'Color','white'); hold on; box on;
    plot(T.set,[1 2],'FaceColor',[.72 .86 .70],'EdgeColor',[.20 .45 .20],'FaceAlpha',.6);
    plot(T.termSet,[1 2],'FaceColor',[.35 .65 .40],'EdgeColor',[.20 .45 .20],'FaceAlpha',.5);
    for k=1:length(sims)
        plot(sims{k}.x(1,:),sims{k}.x(2,:),'Color',[.1 .34 .72],'LineWidth',1.1);
        plot(sims{k}.x(1,1),sims{k}.x(2,1),'o','Color',[.1 .34 .72],'MarkerSize',4);
    end
    axis tight; xlim([-1.1 1.1]); ylim([-1.2 1.2]);
    xlabel('Position x_1'); ylabel('Velocity x_2');
    title('Safe terminal region: double integrator'); grid on;
    subtitle('Light green: terminal region | Dark green: target set | Blue: sampled trajectories');
    exportgraphics(f,fullfile(out,'terminal-region.png'),'Resolution',180);
    print(f,fullfile(out,'terminal-region.svg'),'-dsvg'); close(f);
    report.terminalStatus='passed';
catch err
    report.terminalStatus='failed'; report.terminalError=getReport(err,'extended');
    disp(report.terminalError);
end
write_report(out,report);

function write_report(out,report)
    fid=fopen(fullfile(out,'run-report.json'),'w');
    fprintf(fid,'%s',jsonencode(report,PrettyPrint=true)); fclose(fid);
end
