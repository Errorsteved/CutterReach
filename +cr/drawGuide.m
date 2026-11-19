function drawGuide(ax,rail)
%DRAWGUIDE The twin guide rails are fixed in frame 0 and the carriage slides on them.
f=[1 4 3 2;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
for side=[-1 1]
    xy=[-.022 side*.018]+[-1 -1;1 -1;1 1;-1 1].*[.015 .021]/2;
    vertices=[xy repmat(rail(1)-.06,4,1); xy repmat(rail(2)+.06,4,1)];
    patch(ax,'Vertices',vertices,'Faces',f,'FaceColor',[.38 .48 .51], ...
        'FaceLighting','flat','EdgeColor','none');
end
end
