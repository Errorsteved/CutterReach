function rows = mdh(b)
%MDH Craig modified-DH rows [a_(i-1), alpha_(i-1), d_i, theta_i].
% Variable entries are zero here. FK substitutes absolute joint coordinates.
rows = [0 0 0 0; b.baseOffset 0 0 0; b.shoulderOffset -pi/2 0 0; ...
        b.arm 0 0 0; 0 -pi/2 0 pi; 0 -pi/2 0 0; b.tool 0 0 0];
end
