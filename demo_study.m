function fig = demo_study(study)
%DEMO_STUDY Fixed-base whole-arm view with a separately rotatable detail view.
if nargin==0
    c=cr.config; file=fullfile(c.out,'study.mat');
    if isfile(file), stored=load(file,'study'); end
    if exist('stored','var') && isfield(stored.study,'baselineCutters'), study=stored.study;
    else, study=run_study(c); end
end
c=study.config; g=study.goals;
data={study.baselineTargets,study.optimizedTargets};
arms={study.baseline,study.optimized};
cutters={study.baselineCutters,study.optimizedCutters};
catalogue=g.catalogue; stops=unique(g.headAngle);
meshes=cellfun(@(b)cr.bodyMeshes(cr.buildRobot(b,c.env)),arms,'UniformOutput',false);
modes=["cutter","installed","chamber","assembly"];
i=find(cutters{1}.nFree==0 & cutters{2}.nFree>0,1);
if isempty(i), i=1; end
% Frame the union of both arms' reportable poses, parked poses and target frames.
points=[0 0 0;reshape(g.pose(1:3,4,:),3,[]).'];
park=zeros(2,6);
for d=1:2
    [ok,reason,park(d,:)]=cr.structure(arms{d},c);
    assert(ok,'Cannot display the verified parked pose for design %d (%s).',d,reason);
    Q=[park(d,:);data{d}.q(data{d}.nFree>0,:)];
    P=cr.geometry(arms{d},Q);
    points=[points;reshape(permute(P,[1 3 2]),[],3)]; %#ok<AGROW>
end
rail=[min(arms{1}.limits(1,1),arms{2}.limits(1,1)),max(arms{1}.limits(1,2),arms{2}.limits(1,2))];
points=[points;zeros(2,2) rail.'];
padding=max([c.env.linkRadius .18])+.08;
fig=figure('Name','CutterReach | Fixed base and cutter positions','NumberTitle','off','MenuBar','none', ...
    'ToolBar','none','Color','white','Position',[90 75 1210 690]);
main=axes(fig,'Position',[.025 .24 .535 .61],'Tag','BaseView');
detail=axes(fig,'Position',[.605 .245 .365 .59],'Tag','DetailView');
hold(main,'on'); axis(main,'off'); disableDefaultInteractivity(main); main.Toolbar.Visible='off';
cr.drawGuide(main,rail);
markerColor=[.11 .40 .66];
plot3(main,0,0,0,'d','MarkerSize',9,'MarkerFaceColor',markerColor, ...
    'MarkerEdgeColor','white','LineWidth',1.3,'Tag','BaseOrigin');
text(main,0,0,0,'base_link  ','Interpreter','none','HorizontalAlignment','right', ...
    'VerticalAlignment','top','Color',markerColor,'FontWeight','bold','Clipping','off');
goalDot=plot3(main,NaN,NaN,NaN,'o','Color',[.67 .12 .48],'LineWidth',2, ...
    'MarkerSize',13,'Tag','SelectedTarget');
for d=1:2
    links{d}=cr.armGraphics(main,meshes{d},1:7,true); %#ok<AGROW>
    set(links{d},'Visible','off');
end
cr.baseCamera(main,points,padding);
key=camlight(main,-24,30); key.Color=[.90 .92 .94];
fill=camlight(main,40,-10); fill.Color=[.36 .38 .40];
uicontrol(fig,'Style','text','Units','normalized','Position',[.025 .92 .535 .045], ...
    'String','Full arm with fixed base','BackgroundColor','white','FontWeight','bold','FontSize',14);
headline=uicontrol(fig,'Style','text','Units','normalized','Position',[.595 .92 .39 .045], ...
    'BackgroundColor','white','FontWeight','bold','FontSize',14);
poseLabel=uicontrol(fig,'Style','text','Units','normalized','Position',[.025 .855 .535 .047], ...
    'BackgroundColor','white','FontSize',11,'Tag','PoseState');
options=compose('Cutter %02d   /   r %.2f m',catalogue.CutterID,catalogue.Radius_m);
selector=menu([.025 .095 .155 .055],{'Baseline','Optimized'},2,'Design');
station=menu([.195 .095 .235 .055],cellstr(options),i,'Station');
stopMenu=menu([.445 .095 .25 .055],cellstr(["Automatic stop selection"; compose("Stop at %g°",stops)]),1,'HeadStop');
viewMenu=menu([.71 .095 .265 .055], ...
    {'Cutter close-up','Installed arm detail','Chamber overview','Arm assembly'},1,'DetailMode');
status=uicontrol(fig,'Style','text','Units','normalized','Position',[.025 .018 .95 .06], ...
    'BackgroundColor','white','FontSize',11,'Tag','StationMetrics');
fig.SizeChangedFcn=@(~,~)cr.baseCamera(main,points,padding);
refresh();
rotator=rotate3d(fig); setAllowAxesRotate(rotator,main,false);
setAllowAxesRotate(rotator,detail,true); rotator.Enable='on';

    function ctl=menu(pos,labels,value,tag)
        ctl=uicontrol(fig,'Style','popupmenu','Units','normalized','Position',pos, ...
            'String',labels,'Value',value,'Tag',tag,'Callback',@refresh);
    end

    function refresh(~,~)
        d=selector.Value; k=station.Value; t=data{d}; mode=modes(viewMenu.Value);
        cut=cutters{d}(k,:); rows=find(t.CutterID==cut.CutterID);
        automatic=stopMenu.Value==1;
        if automatic && cut.FeasibleStops>0
            j=cut.WitnessTarget;
        elseif automatic
            [~,nearest]=min(min(t.Head_deg(rows),360-t.Head_deg(rows)));
            j=rows(nearest);
        else
            j=rows(t.Head_deg(rows)==stops(stopMenu.Value-1));
        end
        label=sprintf('%s · Cutter %02d · Stop %g°',selector.String{d},cut.CutterID,t.Head_deg(j));
        reached=t.nFree(j)>0;
        if reached
            q=t.q(j,:);
            poseLabel.String=sprintf('%s · Reachable (%d/%d stops feasible)',label,cut.FeasibleStops,cut.StopsTested);
            status.String=sprintf('TCP error %.3g mm    |    Rotation error %.3g deg    |    Clearance above allowance %.1f mm    |    Dexterity %.3f', ...
                1000*t.posErr(j),rad2deg(t.rotErr(j)),1000*t.gap(j),t.eta(j));
        else
            q=park(d,:);
            if automatic
                poseLabel.String=sprintf('Cutter %02d · No solution found at any of %d stops. Showing stowed pose.',cut.CutterID,cut.StopsTested);
            else
                poseLabel.String=sprintf('%s · No solution found. Showing stowed pose.',label);
            end
            status.String=sprintf('The purple ring marks the cutter at %g°. This stop has %d pose candidates within joint limits. This cutter is reachable at %d/%d stops.', ...
                t.Head_deg(j),t.nPose(j),cut.FeasibleStops,cut.StopsTested);
        end
        [~,F]=cr.fk(arms{d},q);
        set(links{3-d},'Visible','off');
        for k=1:7
            links{d}(k).Matrix=F(:,:,k); links{d}(k).Visible='on';
        end
        target=g.pose(1:3,4,j);
        set(goalDot,'XData',target(1),'YData',target(2),'ZData',target(3));
        cr.drawScene(detail,arms{d},q,c,g,j,mode,meshes{d});
        if mode=="chamber"
            e=c.env; corner=e.cabinXY-e.cabinSize(1:2)/2;
            text(detail,corner(1),corner(2),e.rearZ,'  Storage cabin','FontWeight','bold', ...
                'BackgroundColor','white','Margin',1,'Clipping','off');
        end
        headline.String=viewMenu.String{viewMenu.Value};
        drawnow
    end
end
