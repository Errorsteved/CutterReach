function drawChamber(ax,b,e,g,idx)
%DRAWCHAMBER Service-side cutaway, with the indexed spoke facing the selected station.
steel=[.38 .48 .51]; rim=[.25 .36 .40]; pale=[.65 .73 .74]; selected=[.88 .55 .17];
origin=e.center; z=e.headZ; rad=e.radius;
head=hgtransform('Parent',ax,'Tag','Cutterhead'); parent=head;
if isfield(g,'headAngle')
    stop=g.headAngle(idx);
    head.Matrix=cr.headTransform(e,stop);
    ang=deg2rad(e.spokeZero_deg);
    radii=g.catalogue.Radius_m;
    holderAngles=deg2rad(g.catalogue.HomeAzimuth_deg);
    chosen=g.catalogue.CutterID==g.cutterID(idx);
else
    ang=deg2rad(g.azimuth(idx)); radii=unique(g.radius(g.azimuth==g.azimuth(idx)));
    holderAngles=repmat(ang,size(radii)); chosen=abs(radii-g.radius(idx))<1e-9;
end
% Ring and eight solid spokes are on the material side of the checked head plane.
ring(.915*rad,rad,z,z+.16,rim,0,2*pi,96);
ring(.50*rad,.525*rad,z+.025,z+.115,steel,0,2*pi,96);
ring(.045*rad,.145*rad,z,z+.17,steel,0,2*pi,64);
for j=0:7
    t=ang+j*pi/4; er=[cos(t) sin(t) 0];
    deck=[origin z+.099];
    % Taper the spoke from its load-bearing root to the outer clamp.
    tip=deck+.94*rad*er; root=deck+.12*rad*er;
    beam(root,tip,.46,.19,steel,.30);
    beam(root+[0 0 .105],tip+[0 0 .105],.53,.035,pale,.36);
end
for j=1:numel(radii)
    er=[cos(holderAngles(j)) sin(holderAngles(j)) 0]; et=[-er(2) er(1) 0];
    p=[origin z+.028]+radii(j)*er;
    color=pale;
    if chosen(j), color=selected; end
    for side=[-1 1]
        a=p+side*.083*et;
        beam(a-.12*er,a+.12*er,.040,.055,color);
    end
end
% Cutaway strip and two ring frames show the cylinder depth and partition.
parent=ax;
for zz=[e.gateZ e.gateZ+e.depths(1)]
    ring(rad,rad+.055,zz,zz+.04,pale,0,2*pi,96);
end
ring(rad,rad+.045,e.gateZ+.04,z,[.71 .78 .78],-.24*pi,.50*pi,42,.18);
% The cabin aperture and rails use the installed coordinate system.
corners=e.cabinXY+[1 1;-1 1;-1 -1;1 -1].*e.cabinSize(1:2)/2;
for j=1:4
    k=mod(j,4)+1;
    beam([corners(j,:) e.rearZ],[corners(j,:) e.gateZ],.040,.040,steel);
    for zz=[e.rearZ e.gateZ]
        beam([corners(j,:) zz],[corners(k,:) zz],.048,.048,rim);
    end
end
cr.drawGuide(ax,b.limits(1,:));
% Each I-section and its mounting pedestals fit within the checked capsules.
parent=head;
for j=1:size(e.strutA,1)
    a=e.strutA(j,:); d=e.strutB(j,:); r=e.strutRadius;
    [~,~,normal]=basis(a,d);
    beam(a,d,.26*r,1.13*r,rim);
    for side=[-1 1]
        offset=side*.57*r*normal;
        beam(a+offset,d+offset,1.25*r,.15*r,steel);
    end
    if isfield(e,'postRadius')
        for foot=[a;d].'
            p=foot.'; top=[p(1:2) z]; rr=e.postRadius;
            beam(p,top,.94*rr,1.1*rr,rim);
            beam(top-[0 0 .035],top,1.36*rr,1.36*rr,steel);
            for dx=[-1 1]
                for dy=[-1 1]
                    bolt=top+[dx*.48*rr dy*.48*rr -.047];
                    beam(bolt,bolt+[0 0 .012],.016,.016,pale);
                end
            end
        end
    end
end

    function beam(a,d,w,h,color,tipWidth)
        if nargin<6, tipWidth=w; end
        [~,v,n]=basis(a,d);
        profile=[-1 -1;1 -1;1 1;-1 1];
        offsets=profile(:,1).*v*w/2+profile(:,2).*n*h/2;
        endOffsets=profile(:,1).*v*tipWidth/2+profile(:,2).*n*h/2;
        V=[a+offsets;d+endOffsets];
        F=[1 4 3 2;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
        patch('Parent',parent,'Vertices',V,'Faces',F,'FaceColor',color,'FaceLighting','flat','EdgeColor','none');
    end
    function ring(ri,ro,z1,z2,color,t1,t2,N,alpha)
        if nargin<9, alpha=1; end
        t=linspace(t1,t2,N+1).'; dirs=[cos(t) sin(t)];
        V=[origin+ri*dirs,z1*ones(N+1,1);origin+ro*dirs,z1*ones(N+1,1); ...
           origin+ri*dirs,z2*ones(N+1,1);origin+ro*dirs,z2*ones(N+1,1)];
        a=(1:N).'; stride=N+1;
        F=[a,a+1,a+1+stride,a+stride; a+2*stride,a+3*stride,a+3*stride+1,a+2*stride+1; ...
           a,a+2*stride,a+2*stride+1,a+1; a+stride,a+stride+1,a+3*stride+1,a+3*stride];
        patch('Parent',parent,'Vertices',V,'Faces',F,'FaceColor',color,'FaceLighting','flat','EdgeColor','none','FaceAlpha',alpha);
    end
end

function [u,v,n]=basis(a,b)
u=(b-a)/norm(b-a);
v=cross([0 0 1],u);
if norm(v)<1e-8, v=[1 0 0]; else, v=v/norm(v); end
n=cross(u,v);
end
