function v = validateBaseline(b,c,w)
%VALIDATEBASELINE Independent published Cartesian checkpoints before design search.
robot = cr.buildRobot(b);
qt = [0.807 pi -pi/2 -pi/2 1.310 -pi/2; ...
      1.0677 deg2rad([176.6 -81 -66.9]) 1.924 deg2rad(-103.5)];
reported = [0.460 0 3.309; -0.820 0.082 3.857];
p = zeros(2,3);
for k = 1:2
    T = cr.fk(b,qt(k,:)); p(k,:) = T(1:3,4).';
end
v.checkpoints = table(["A";"B"],reported,p,1000*vecnorm(p-reported,2,2), ...
    cr.inLimits(b,qt),'VariableNames',{'Point','Paper_m','Computed_m','Error_mm','InTable1Limits'});
assert(all(v.checkpoints.Error_mm < [0.01;2]),'Paper Cartesian checkpoints failed.');
paper = readtable(fullfile(c.root,'data','paper_table2.csv'));
Q = paper{:,2:end}.*[0.001 pi/180 pi/180 pi/180 0.001 pi/180];
assert(all(cr.inLimits(b,Q)),'Published Table 2 violates the selected limits.');
for k = height(paper):-1:1
    G = cr.fk(b,Q(k,:));
    toolXYZ(k,:) = G(1:3,4).';
    sol = cr.solvePose(b,G,c,c.fineGrid);
    poseCount(k,1) = sol.nPose;
end
v.table2 = addvars(paper,toolXYZ,poseCount,'NewVariableNames',{'ReconstructedTCP_m','BoundedIKCount'});
assert(all(poseCount>0),'A Table 2 pose did not close under bounded IK.');
% Cross-check poses and the Jacobian over the joint range using independent APIs.
samples = [Q;w.q(1:min(height(w.q),32),:)];
err = zeros(height(samples),4);
for k = 1:height(samples)
    q = samples(k,:);
    [T,F] = cr.fk(b,q); [P,R] = cr.geometry(b,q);
    treeT = getTransform(robot,q,'frame7');
    JT = geometricJacobian(robot,q,'frame7');
    sv = svd(diag([1 1 1 1/c.Lref 1/c.Lref 1/c.Lref])*JT*diag([c.Lref 1 1 1 c.Lref 1]));
    ids = [1 2 3 4 5 7];
    dp = squeeze(F(1:3,4,ids))-reshape(P,3,6);
    err(k,:) = [norm(T-treeT,'fro'),norm(cr.jacobian(b,q)-JT,'fro'), ...
        max(norm(dp,'fro'),norm(R-T(1:3,1:3),'fro')),abs(cr.dexterity(b,q,c.Lref)-sv(4)/sv(1))];
    assert(sv(5)<1e-10 && sv(4)>1e-8,'Unexpected kinematic rank.');
end
v.crosscheck = array2table(max(err,[],1),'VariableNames', ...
    {'MaxTreeFK','MaxTreeJacobian','MaxBatchGeometry','MaxDexterityDifference'});
assert(all(v.crosscheck{1,:}<1e-9),'Independent kinematic implementations disagree.');
% Explicit admissible configurations isolate the Fig. 10 axial discrepancy
% from Monte Carlo coverage or seed choice (unconstrained paper workspace).
stress = [0 0 0 0 b.limits(5,2) 0; 3 0 -pi/2 -pi/2 b.limits(5,2) -pi/2];
xyz = zeros(2,3);
for k = 1:2
    T = getTransform(robot,stress(k,:),'frame7'); xyz(k,:) = T(1:3,4).';
end
v.axialExamples = table(stress,xyz,cr.inLimits(b,stress),'VariableNames',{'q','TCP_m','WithinLimits'});
v.extents = table(["X";"Y";"Z"],[-2.9;-3.3;-0.2],[3.6;3.3;4.6], ...
    min(w.xyz,[],1).',max(w.xyz,[],1).','VariableNames', ...
    {'Axis','PaperMin_m','PaperMax_m','SampleMin_m','SampleMax_m'});
end
