function [x, y, z] = DepthAprox(depth_pic)
%% returns location of quad as x, y, and z, given cropped depth picture
new_pic = depth_pic;
new_pic(depth_pic == 0 | depth_pic > 2000) = 2000;
minD = min(min(new_pic));
[row,col] = find(new_pic<minD+100);
index = sub2ind(size(new_pic),row,col);
x = round(median(col));
y = round(median(row));
z = round(mean(new_pic(index)));
end