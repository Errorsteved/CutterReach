function ok = inLimits(b,Q)
%INLIMITS Floating-point allowance only, applied in metres or radians.
ok = all(isfinite(Q) & Q >= b.limits(:,1).'-1e-10 & Q <= b.limits(:,2).'+1e-10,2);
end
