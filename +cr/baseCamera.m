function baseCamera(ax,points,padding)
%BASECAMERA Fit all study poses once, pinning frame 0 in the main viewport.
% The orthographic camera is independent of target, design and context view.
direction=[-1 -.30 -.24]; direction=direction/norm(direction);
% Place the swept disk across the wide viewport, with axial rail travel upward.
right=cross([0 0 1],direction); right=right/norm(right); up=cross(direction,right);
basis=[right;up;direction]; projected=points*basis.';
low=min(projected)-padding; high=max(projected)+padding;
viewport=getpixelposition(ax,true); aspect=viewport(3)/max(1,viewport(4));
anchor=[.50 .23]; % leave room on both sides for a complete head revolution
height=1.06*max([-low(1)/(aspect*anchor(1)),high(1)/(aspect*(1-anchor(1))), ...
    -low(2)/anchor(2),high(2)/(1-anchor(2))]);
center=[(.5-anchor(1))*height*aspect,(.5-anchor(2))*height,mean([low(3) high(3)])]*basis;
distance=5*max([height*aspect height norm(high-low)]);
ax.Projection='orthographic'; daspect(ax,[1 1 1]);
limits=[min(points)-padding;max(points)+padding];
xlim(ax,limits(:,1)); ylim(ax,limits(:,2)); zlim(ax,limits(:,3));
camtarget(ax,center); campos(ax,center+distance*direction); camup(ax,up);
camva(ax,2*atand(height/(2*distance)));
end
