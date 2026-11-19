function [ok,reason,stow] = structure(b,c)
%STRUCTURE Absolute extension, stroke/retracted ratio and a folded cabin witness.
e = c.env;
stow = [e.gateZ-0.13 pi/2 deg2rad(74) deg2rad(-74) b.limits(5,1) -pi/2];
P = cr.geometry(b,stow);
ceiling = squeeze(max(max(P(:,3,1:5),P(:,3,2:6)) ...
    +reshape(e.linkRadius,1,1,[])+e.clearance));
reason = "admitted";
if b.limits(5,2)>c.maxExtended+1e-12
    reason = "extended length";
elseif diff(b.limits(5,:))>c.maxStrokeRatio*b.limits(5,1)
    reason = "stroke ratio";
elseif ~cr.inLimits(b,stow) || any(ceiling>=e.gateZ) || ~cr.collision(P,e)
    reason = "stow envelope";
end
ok = reason=="admitted";
end
