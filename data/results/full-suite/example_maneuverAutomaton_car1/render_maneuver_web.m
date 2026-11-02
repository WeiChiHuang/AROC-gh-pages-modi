function render_maneuver_web(workspacePath,outputDirectory)
% Re-render recorded planning geometry; does not recompute the controller.
% Load the matching AROC/CORA versions before calling this function.
s=load(workspacePath,'MA','ind','x0','dynObs','lanelets');
f=figure('Visible','off','Color','w','Position',[100,100,1100,850]);
layout=tiledlayout(f,4,1,'TileSpacing','compact','Padding','compact');
for k=1:4
    ax=nexttile(layout); hold(ax,'on'); box(ax,'on');
    for j=1:numel(s.lanelets)
        plot(s.lanelets{j},[1,2],'FaceColor',[.6 .6 .6],'EdgeColor','k');
    end
    plotPlannedTrajectory(s.MA,s.ind,s.x0,[],[0 .7 0],'EdgeColor','k');
    plotPlannedTrajectory(s.MA,s.ind,s.x0,interval(k-1),'b');
    plotObstacles([],s.dynObs,interval(k-1));
    xlim(ax,[0,70]);
    ylabel(ax,'y [m]');
    title(ax,sprintf('t = %d s',k-1),'HorizontalAlignment','left');
    set(ax,'FontName','Arial','FontSize',12);
    if k==1
        a=plot(ax,nan,nan,'s','MarkerFaceColor',[0 .7 0],'MarkerEdgeColor','k');
        b=plot(ax,nan,nan,'s','MarkerFaceColor','b','MarkerEdgeColor','k');
        c=plot(ax,nan,nan,'s','MarkerFaceColor',[1 .5 .5],'MarkerEdgeColor','none');
        legend(ax,[a,b,c],{'Planned occupancy','Occupancy at shown time','Dynamic obstacles'},'Location','northoutside','Orientation','horizontal');
    end
end
xlabel(layout,'x [m]');
exportgraphics(f,fullfile(outputDirectory,'website-figure.png'),'Resolution',180);
print(f,fullfile(outputDirectory,'website-figure.svg'),'-dsvg');
close(f);
end
