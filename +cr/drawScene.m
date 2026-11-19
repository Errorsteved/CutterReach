function drawScene(ax,b,q,c,goals,target,mode,meshes)
%DRAWSCENE The robot uses the exact visual meshes stored in its rigidBodyTree.
% Modes: chamber cutaway, installed detail, isolated assembly, or cutter close-up.
if islogical(mode) || isnumeric(mode)
    if mode, mode="installed"; else, mode="chamber"; end
end
mode=string(mode);
if nargin<8 && all(isfinite(q)), meshes=cr.bodyMeshes(cr.buildRobot(b,c.env)); end
cla(ax,'reset'); hold(ax,'on'); ax.Color=[.98 .985 .985];
installed=any(mode==["chamber","installed"]);
if installed, cr.drawChamber(ax,b,c.env,goals,target); end
G=goals.pose(:,:,target); pos=G(1:3,4).';
hasPose=all(isfinite(q)); pts=pos;
if hasPose
    [~,F]=cr.fk(b,q);
    parts=1:7;
    if mode=="cutter", parts=6:7; end
    visibleVertices=cell(1,7);
    bodies=cr.armGraphics(ax,meshes,parts,~installed);
    for j=1:numel(parts)
        k=parts(j); M=F(:,:,k); bodies(j).Matrix=M;
        visibleVertices{k}=meshes{k}.V*M(1:3,1:3).'+M(1:3,4).';
    end
    pts=reshape(F(1:3,4,:),3,[]).';
end
if installed || ~hasPose
    % TCP is a standoff in front of the selected mounting pocket.
    pocket=[pos(1:2) c.env.headZ];
    plot3(ax,[pos(1) pocket(1)],[pos(2) pocket(2)],[pos(3) pocket(3)], ...
        ':','Color',[.84 .34 .08],'LineWidth',1.2);
    cols=[.81 .26 .16;.17 .52 .36;.20 .40 .68];
    for j=1:3
        v=.18*G(1:3,j);
        quiver3(ax,pos(1),pos(2),pos(3),v(1),v(2),v(3),0, ...
            'Color',cols(j,:),'LineWidth',1.2,'MaxHeadSize',.5);
    end
end
axis(ax,'equal'); ax.Projection='orthographic'; box(ax,'off');
xlabel(ax,'X_0 (m)'); ylabel(ax,'Y_0 (m)'); zlabel(ax,'Z_0 (m)');
er=[cosd(goals.azimuth(target)) sind(goals.azimuth(target)) 0]; et=[-er(2) er(1) 0];
direction=-et-.36*er+[0 0 .70]; up=[0 0 1];
switch mode
    case "chamber"
        lo=[c.env.center-c.env.radius-.15 c.env.rearZ-.10];
        hi=[c.env.center+c.env.radius+.15 c.env.headZ+.34];
        direction=[-.80 -.52 -1.12]; up=[-1 0 0];
    case "cutter"
        if hasPose
            % A parked arm can be far from the selected goal.
            pos=F(1:3,4,7).';
            wrist=F(1:3,4,6).';
            et=F(1:3,3,7).'; er=[et(2) -et(1) 0];
        else
            wrist=pos-b.tool*G(1:3,1).';
        end
        lo=min(pos,wrist)-.12; hi=max(pos,wrist)+.12;
        direction=-.57*et+.63*er+[0 0 .71];
        % Keep the spindle axis tilted towards the viewer and the tool horizontal.
        up=er;
    otherwise
        if ~hasPose, pts=[pos;[c.env.center c.env.gateZ]]; end
        if installed, pts=[pts;pos]; end
        lo=min(pts)-.15; hi=max(pts)+.15;
        if mode=="installed", direction=-et+.34*er+[0 0 -.46]; end
end
xlim(ax,[lo(1) hi(1)]); ylim(ax,[lo(2) hi(2)]); zlim(ax,[lo(3) hi(3)]);
direction=direction/norm(direction); center=(lo+hi)/2; distance=4*norm(hi-lo);
if hasPose && ~installed
    % Fit the projected solids, including both jaws, independently of world-axis bounds.
    right=cross(up,direction); right=right/norm(right); up=cross(direction,right);
    vertices=vertcat(visibleVertices{parts}); basis=[right;up;direction];
    projected=vertices*basis.'; low=min(projected); high=max(projected);
    center=((high+low)/2)*basis;
    viewport=getpixelposition(ax,true);
    aspect=viewport(3)/max(viewport(4),1);
    height=1.14*max(high(2)-low(2),(high(1)-low(1))/aspect);
    camva(ax,2*atand(height/(2*distance)));
end
camtarget(ax,center); campos(ax,center+distance*direction); camup(ax,up);
if installed, grid(ax,'on'); ax.GridAlpha=.09; else, axis(ax,'off'); end
key=camlight(ax,-24,30); key.Color=[.90 .92 .94];
fill=camlight(ax,40,-10); fill.Color=[.36 .38 .40];
end
