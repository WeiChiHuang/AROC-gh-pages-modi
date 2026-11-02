% Reproduce conformant synthesis using AROC 2026 and CORA v2026.0.1.
% Add AROC and CORA to the MATLAB path first; MATLAB Optimization Toolbox
% is required. Measurements are the bundled measurements_platoon.mat.
% This checks fitting-data containment, not held-out generalization.
set(groot,'defaultFigureVisible','off');
set(groot,'defaultAxesFontName','Arial','defaultAxesFontSize',12);
rng(20261019,'twister');
out=fullfile(pwd,'aroc-website-results');
if ~isfolder(out), mkdir(out); end
if isfile(fullfile(out,'run-report.json'))
    report=jsondecode(fileread(fullfile(out,'run-report.json')));
else
    report=struct('matlab',version,'seed',20261019);
end
report.conformanceSolver='MATLAB quadratic programming';
try
    data=load('measurements_platoon.mat'); M=data.M;
    Opts=struct('group',6,'measErr',true,'mu',.1,'set','interval');
    started=tic;
    [Wint,Vint]=conformantSynthesis('platoon4',M,Opts);
    report.conformanceSeconds=toc(started);
    R=reachSetConformance('platoon4',M{1},Wint,Vint);
    times=cell2mat(R.timePoint.time); mt=M{1}.t-M{1}.t(1);
    good=[];
    for k=1:length(mt)
        [delta,j]=min(abs(times-mt(k)));
        if delta<1e-7
            good(end+1)=contains(R.timePoint.set{j},M{1}.x(:,k),'exact',1e-6);
        end
    end
    report.conformanceCheckedPoints=length(good);
    report.conformanceContainedPoints=sum(good);
    report.conformanceTrajectoriesUsed=length(M);
    assert(all(good),'Supplied trajectory containment check failed');
    report.conformanceStatus='passed-on-supplied-trajectory';
    save(fullfile(out,'conformant-synthesis.mat'),'Wint','Vint','R','Opts','M');
    f=figure('Position',[100 100 1000 680],'Color','white'); hold on; box on;
    plot(R,[1 2],'FaceColor',[.74 .84 .96],'EdgeColor','none');
    plot(M{1}.x(1,:),M{1}.x(2,:),'Color',[.8 .22 .1],'LineWidth',1.3);
    xlabel('Lead-vehicle position x_1'); ylabel('Lead-vehicle velocity x_2');
    title('Conformant synthesis: four-vehicle platoon'); grid on;
    subtitle('Blue: model reachable sets | Red: supplied measurement trajectory');
    exportgraphics(f,fullfile(out,'conformant-synthesis.png'),'Resolution',180);
    print(f,fullfile(out,'conformant-synthesis.svg'),'-dsvg'); close(f);
catch err
    report.conformanceStatus='failed'; report.conformanceError=getReport(err,'basic'); disp(report.conformanceError);
end
fid=fopen(fullfile(out,'run-report.json'),'w'); fprintf(fid,'%s',jsonencode(report,PrettyPrint=true)); fclose(fid);

