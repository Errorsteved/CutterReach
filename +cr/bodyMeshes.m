function meshes = bodyMeshes(robot)
%BODYMESHES Combine native visual triangles per link, retaining material colours.
% Vertex coordinates are in the owning MDH frame, ready for FK or envelope QA.
meshes=cell(1,robot.NumBodies);
for i=1:robot.NumBodies
    parts=getVisual(robot.Bodies{i});
    vertices=cell(1,numel(parts)); faces=vertices; colors=vertices; offset=0;
    for k=1:numel(parts)
        part=parts(k); T=part.Tform;
        v=part.Triangulation.Points.*part.Scale;
        vertices{k}=v*T(1:3,1:3).'+T(1:3,4).';
        f=part.Triangulation.ConnectivityList;
        faces{k}=f+offset;
        colors{k}=repmat(part.Color(1:3),size(f,1),1);
        offset=offset+size(v,1);
    end
    v=vertcat(vertices{:}); f=vertcat(faces{:});
    % Mark sharp folds rather than drawing the triangulation on machined faces.
    edges=featureEdges(triangulation(f,v),deg2rad(38));
    meshes{i}=struct('V',v,'F',f,'C',vertcat(colors{:}),'E',edges);
end
end
