function [rowrange,colrange] = rangegetter(medianrow,mediancol,range)
%% returns range for row and col for cropping depth image
row1 = medianrow - range; row2 = medianrow + range;
col1 = mediancol - range; col2 = mediancol + range;
if col1 < 1
    col1 = 1;
elseif col2 > 640
    col2 = 640;
end
if row1 < 1
    row1 = 1;
elseif row2 > 480
    row2 = 480;
end
rowrange = (row1:row2)';
colrange = (col1:col2)';
end