function [a,b,r] = braceSegments(e)
%BRACESEGMENTS Inclined support and two pedestals fastening it to the head.
a=e.strutA; b=e.strutB; r=repmat(e.strutRadius,size(a,1),1);
if isfield(e,'postRadius')
    foot=[a;b];
    deck=foot; deck(:,3)=e.headZ;
    a=[a;foot]; b=[b;deck]; r=[r;repmat(e.postRadius,size(foot,1),1)];
end
end
