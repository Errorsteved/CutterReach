function e = indexedEnvironment(e,angle)
%INDEXEDENVIRONMENT Rotate only components bolted to the cutterhead.
T = cr.headTransform(e,angle);
e.strutA = e.strutA*T(1:3,1:3).' + T(1:3,4).';
e.strutB = e.strutB*T(1:3,1:3).' + T(1:3,4).';
end
