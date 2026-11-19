function verifyWitnesses(b,t,g,c)
%VERIFYWITNESSES Independently recompute every reported success with the toolbox.
tree = cr.buildRobot(b);
for i = reshape(find(t.nFree>0),1,[])
    q = t.q(i,:); T = getTransform(tree,q,'frame7');
    [p,a] = cr.poseError(T,g.pose(:,:,i));
    P = zeros(1,3,6); ids = [1 2 3 4 5 7];
    for j = 1:numel(ids)
        F = getTransform(tree,q,char("frame"+ids(j)));
        P(1,:,j) = F(1:3,4).';
    end
    e = c.env;
    if isfield(g,'headAngle'), e = cr.indexedEnvironment(e,g.headAngle(i)); end
    [free,gap] = cr.collision(P,e);
    assert(cr.inLimits(b,q) && free && abs(gap-t.gap(i))<1e-9 ...
        && p<=c.positionTol && a<=c.rotationTol,'Invalid witness for target %d.',i);
end
end
