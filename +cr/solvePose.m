function out = solvePose(b,G,c,grid)
%SOLVEPOSE Semi-analytic IK for the paper's MDH family. SI input/output.
% Enumerate two redundant coordinates, invert the radial wrist equation on
% both sine branches, then independently check each surviving pose and limit.
out = struct('q',nan(1,6),'eta',NaN,'gap',NaN,'posErr',NaN,'rotErr',NaN, ...
    'nPose',0,'nFree',0,'nWall',0,'nBrace',0,'nSelf',0,'status',"no bounded pose");
R = G(1:3,1:3);
yaw = atan2(-R(1,3),R(2,3)); u = [cos(yaw);sin(yaw);0];
yaw = inTurn(yaw,b.limits(2,1));
s = atan2(R(3,1),-u.'*R(:,1));
[T3,D] = ndgrid(linspace(b.limits(3,1),b.limits(3,2),grid(1)), ...
                 linspace(b.limits(5,1),b.limits(5,2),grid(2)));
t3 = T3(:); d = D(:);
target = G(1:3,4)-[b.baseOffset;0;0];
rw = u.'*target+b.tool*cos(s);
sine = (b.shoulderOffset+b.arm*cos(t3)-rw)./d;
ok = abs(sine)<=1+1e-12;
if ~any(ok), return; end
h = asin(min(1,max(-1,sine(ok))));
h = [h;pi-h]; t3 = repmat(t3(ok),2,1); d = repmat(d(ok),2,1);
t4 = inTurn(h-t3,b.limits(4,1));
t6 = inTurn(s-h,b.limits(6,1));
d1 = target(3)-b.tool*sin(s)+b.arm*sin(t3)+d.*cos(h);
Q = [d1,repmat(yaw,numel(h),1),t3,t4,d,t6];
Q = Q(cr.inLimits(b,Q),:);
if isempty(Q), return; end
[P,ori] = cr.geometry(b,Q);
posErr = vecnorm(P(:,:,6)-G(1:3,4).',2,2);
% trace(R_goal' * R_actual) = Frobenius inner product.
cs = (reshape(sum(ori.*R,[1 2]),[],1)-1)/2;
rotErr = acos(min(1,max(-1,cs)));
ok = posErr<=c.positionTol & rotErr<=c.rotationTol;
out.nPose = nnz(ok);
if ~any(ok)
    out.status = "pose tolerance";
    return
end
Q = Q(ok,:); P = P(ok,:,:);
[free,gap,reason] = cr.collision(P,c.env);
out.nFree = nnz(free);
out.nWall = nnz(reason==1); out.nBrace = nnz(reason==2); out.nSelf = nnz(reason==3);
if ~any(free)
    out.status = "collision";
    return
end
Q = Q(free,:); gap = gap(free);
[out.eta,i] = max(cr.dexterity(b,Q,c.Lref));
out.q = Q(i,:); out.gap = gap(i); out.status = "reached";
[out.posErr,out.rotErr] = cr.poseError(cr.fk(b,out.q),G);
end

function v = inTurn(v,lo)
% Pick the first equivalent angle in [lo,lo+2*pi); retain the lower endpoint.
v = v+2*pi*ceil((lo-v-1e-12)/(2*pi));
end
