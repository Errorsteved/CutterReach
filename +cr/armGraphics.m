function bodies = armGraphics(ax,meshes,parts,outline)
%ARMGRAPHICS Keep each link's vertices local; its graphics transform is T_0i.
bodies = gobjects(1,numel(parts));
for j=1:numel(parts)
    k=parts(j); m=meshes{k};
    bodies(j)=hgtransform('Parent',ax,'Tag',sprintf('frame%d',k));
    patch('Parent',bodies(j),'Faces',m.F,'Vertices',m.V,'FaceVertexCData',m.C, ...
        'FaceColor','flat','EdgeColor','none','AmbientStrength',.46, ...
        'FaceLighting','gouraud','DiffuseStrength',.68,'SpecularStrength',.16,'SpecularExponent',32);
    if outline && k~=7
        sides=m.E;
        keep=vecnorm(m.V(sides(:,2),:)-m.V(sides(:,1),:),2,2)>.003;
        sides=sides(keep,:);
        lines=nan(3*size(sides,1),3);
        lines(1:3:end,:)=m.V(sides(:,1),:);
        lines(2:3:end,:)=m.V(sides(:,2),:);
        line('Parent',bodies(j),'XData',lines(:,1),'YData',lines(:,2),'ZData',lines(:,3), ...
            'Color',[.20 .24 .24],'LineWidth',.35);
    end
end
end
