function w = workspace(b,e,U,L)
%WORKSPACE Uniform joint-space MC. One normalized sample is shared by both arms.
Q = b.limits(:,1).'+U.*diff(b.limits,1,2).';
P = cr.geometry(b,Q);
[w.free,w.margin,w.reason] = cr.collision(P,e);
w.xyz = P(:,:,6); w.q = Q; w.eta = cr.dexterity(b,Q,L);
end
