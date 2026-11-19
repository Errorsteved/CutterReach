function exportFigures(s)
%EXPORTFIGURES Report-ready raster plots; exact data are available in study.mat.
c = s.config; b = s.baseline; o = s.optimized;
teal = [0.04 0.46 0.45]; amber = [0.85 0.43 0.12]; slate = [0.35 0.43 0.48];
r = {s.baselineTargets,s.optimizedTargets}; w = {s.baselineWorkspace,s.optimizedWorkspace};
labels = ["Baseline","Optimized"];
cuts = {s.baselineCutters,s.optimizedCutters};
row = find(cuts{2}.nFree>0 & cuts{1}.nFree==0,1);
if isempty(row), row = find(cuts{2}.nFree>0,1); end
if isempty(row), i = 1; else, i = cuts{2}.WitnessTarget(row); end
meshes = cr.bodyMeshes(cr.buildRobot(o,c.env));
[f,tl] = canvas('Installed arm',1,2);
cl = onCleanup(@()close(f));
ax = nexttile(tl); cr.drawScene(ax,o,r{2}.q(i,:),c,s.goals,i,"chamber",meshes); axis(ax,'off');
title(ax,'Chamber, storage cabin and cutter stations');
ax = nexttile(tl); cr.drawScene(ax,o,r{2}.q(i,:),c,s.goals,i,"installed",meshes); axis(ax,'off');
title(ax,sprintf('Cutter %02d   |   %.2f m radius, head rotation %g°',s.goals.cutterID(i),s.goals.radius(i),s.goals.headAngle(i)));
title(tl,'Optimized geometry in the registered chamber');
saveplot(f,c,'model'); clear cl

% Fixed caption positions stay clear of the changing projected 3D bounds.
f = figure('Visible','off','Color','w','Position',[50 50 1250 410]);
cl = onCleanup(@()close(f));
q = r{2}.q(i,:); assemblyGoals=s.goals;
if any(~isfinite(q))
    [~,~,q]=cr.structure(o,c);
    assemblyGoals.pose(:,:,i)=cr.fk(o,q);
end
ax=axes(f,'Position',[.035 .06 .49 .72]);
cr.drawScene(ax,o,q,c,assemblyGoals,i,"assembly",meshes);
ax=axes(f,'Position',[.57 .06 .39 .72]);
cr.drawScene(ax,o,q,c,assemblyGoals,i,"cutter",meshes);
annotation(f,'textbox',[.035 .86 .50 .10],'String','Manipulator assembly', ...
    'EdgeColor','none','FontSize',16,'FontWeight','bold','FitBoxToText','on');
annotation(f,'textbox',[.57 .86 .40 .10],'String','Axle gripper and disc cutter', ...
    'EdgeColor','none','FontSize',16,'FontWeight','bold','FitBoxToText','on');
saveplot(f,c,'assembly'); clear cl

[f,tl] = canvas('Workspace',1,2); cl = onCleanup(@()close(f));
xyz = [w{1}.xyz;w{2}.xyz]; low = floor(min(xyz)); high = ceil(max(xyz));
for j = 1:2
    ax = nexttile(tl); hold(ax,'on'); W = w{j}; blocked = W.xyz(~W.free,:); good = W.xyz(W.free,:);
    scatter3(ax,blocked(:,1),blocked(:,2),blocked(:,3),3,[0.72 0.75 0.77],'filled','MarkerFaceAlpha',0.12);
    scatter3(ax,good(:,1),good(:,2),good(:,3),7,W.eta(W.free),'filled');
    clim(ax,[0 0.4]); colormap(ax,parula(128)); cb = colorbar(ax); cb.Label.String = '\sigma_4 / \sigma_1';
    axis(ax,'equal'); xlim(ax,[low(1) high(1)]); ylim(ax,[low(2) high(2)]); zlim(ax,[low(3) high(3)]);
    view(ax,[-36 21]); grid(ax,'on'); xlabel(ax,'X_0 (m)'); ylabel(ax,'Y_0 (m)'); zlabel(ax,'Z_0 (m)');
    title(ax,sprintf('%s   |   %d / %d clear',labels(j),nnz(W.free),numel(W.free)));
end
title(tl,'Joint-limit workspace and collision-free samples');
subtitle(tl,'Grey points intersect the modeled envelope. Colour indicates local attainable-twist dexterity.');
saveplot(f,c,'workspace'); clear cl

[f,tl] = canvas('Accessibility',1,2); cl = onCleanup(@()close(f));
for j = 1:2
    ax = nexttile(tl); hold(ax,'on'); t = r{j};
    plotTarget(ax,t,t.nPose==0,'x',slate,'No bounded pose');
    plotTarget(ax,t,t.nPose>0 & t.nFree==0,'s',amber,'Blocked');
    plotTarget(ax,t,t.nFree>0,'o',teal,'Reached');
    cut=cuts{j}; success=cut.FeasibleStops>0;
    plot(ax,cut.CutterID(success),cut.BestHead_deg(success),'ko','LineWidth',1.4, ...
        'MarkerSize',9,'DisplayName','Selected stop');
    xticks(ax,sort(cut.CutterID)); xlim(ax,[min(cut.CutterID)-.5 max(cut.CutterID)+.5]);
    xlabel(ax,'Cutter ID'); ylabel(ax,'Head rotation from home (deg)');
    yticks(ax,0:45:360); ylim(ax,[-6 366]); grid(ax,'on'); box(ax,'on');
    title(ax,sprintf('%s   |   %d / %d cutters',labels(j),nnz(success),height(cut)));
    legend(ax,'Location','southoutside','NumColumns',2,'FontSize',9);
end
title(tl,'Which cutters have a feasible service pose after indexing?');
saveplot(f,c,'accessibility'); clear cl

[f,tl] = canvas('Design tradeoffs',1,2); cl = onCleanup(@()close(f));
ax = nexttile(tl); g = s.grid; ok = isfinite(g.Rate); hold(ax,'on');
scatter(ax,g.Stroke_m(ok),g.Arm_m(ok),280,100*g.Rate(ok),'filled','Marker','s');
scatter(ax,g.Stroke_m(~ok),g.Arm_m(~ok),70,slate,'x','LineWidth',1.5);
scatter(ax,diff(b.limits(5,:)),b.arm,410,slate,'o','LineWidth',1.5);
scatter(ax,diff(o.limits(5,:)),o.arm,420,amber,'d','LineWidth',2);
xlabel(ax,'Telescope stroke (m)'); ylabel(ax,'Upper-arm length (m)');
cb = colorbar(ax); cb.Label.String = 'Accessible cutters (%)'; colormap(ax,parula(128));
title(ax,'Design grid'); subtitle(ax,'Circle = baseline   /   diamond = selected   /   × = excluded');
axis(ax,'padded'); grid(ax,'on');
ax = nexttile(tl); t = s.summary;
sets = ["Selection" "Refined" "Holdout"]; A = zeros(3,2);
for j = 1:3
    A(j,:) = 100*t.Rate(t.Set==sets(j)).';
end
hb = bar(ax,A); hb(1).FaceColor = slate; hb(2).FaceColor = teal; ylim(ax,[0 100]); grid(ax,'on');
xticks(ax,1:3); xticklabels(ax,sets); ylabel(ax,'Accessible cutters (%)');
legend(ax,labels,'Location','southoutside','Orientation','horizontal');
title(ax,'Paired access rates');
title(tl,'Structural design search and verification');
saveplot(f,c,'design_comparison'); clear cl

[f,tl] = canvas('Dexterity',1,1); cl = onCleanup(@()close(f));
ax = nexttile(tl); hold(ax,'on'); colors=[slate;teal];
for j=1:2
    t=cuts{j};
    plot(ax,t.CutterID,t.eta,'o-','Color',colors(j,:),'LineWidth',1.6,'DisplayName',labels(j));
end
xticks(ax,sort(cuts{1}.CutterID)); grid(ax,'on'); box(ax,'on');
xlabel(ax,'Cutter ID'); ylabel(ax,'Best feasible \sigma_4 / \sigma_1');
legend(ax,'Location','southoutside','Orientation','horizontal');
title(tl,'Dexterity at the selected stop for each cutter');
subtitle(tl,'Gaps indicate cutters with no feasible sampled stop.');
saveplot(f,c,'dexterity'); clear cl
end

function plotTarget(ax,t,mask,symbol,color,label)
plot(ax,t.CutterID(mask),t.Head_deg(mask),symbol,'Color',color,'MarkerFaceColor',color, ...
    'MarkerSize',3,'LineWidth',.7,'DisplayName',label);
end
function [f,t] = canvas(name,n,m)
f = figure('Visible','off','Color','w','Position',[50 50 1230 525],'Name',name);
t = tiledlayout(f,n,m,'Padding','compact','TileSpacing','compact');
end
function saveplot(f,c,name)
set(findall(f,'Type','axes'),'FontSize',11);
exportgraphics(f,fullfile(c.out,[name '.png']),'Resolution',165,'BackgroundColor','white');
end
