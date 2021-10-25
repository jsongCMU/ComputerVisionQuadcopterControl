function [CurrentState] = initialpositiongetter
%% Get initial position of quadcopter
% Called at start of Python script
% Assumes no initial velocity
Rthresh = 100;
Gthresh = 50;
Bthresh = 150;
CurrentState = zeros(5,3);
xadjust = -10;
yadjust = -10;
radius = 20;
vid1 = imaqfind('name','RGB_640x480-kinect-1');
vid2 = imaqfind('name','Depth_640x480-kinect-2');
vid1 = vid1{1}; vid2 = vid2{1};
pic = flipud(getsnapshot(vid1));                                                
depth_pic = flipud(getsnapshot(vid2));  
[~,row,col] = redfilterv2(pic,Rthresh,Gthresh,Bthresh);
mediancol = round(median(col)); medianrow = round(median(row));             % Calculates median to approximate center of mass; I used median rather than mean since it's not heavily influenced by outliers
approxx = mediancol+xadjust; approxy = medianrow+yadjust;
[rowrange,colrange] = rangegetter(approxy,approxx,radius);                  % Returns valid range of rows and colums given a point; last parameter is half range size
[x,y,z] = DepthAprox(depth_pic(rowrange,colrange));                         % DepthAprox accepts a cropped image, then finds the approximate center of mass
x = colrange(1)-1+x; y = rowrange(1)-1+y;                                   % x and y is with respect to the cropped image (eg 1,1 would be top left of the cropped image, not the original pic); this changes to with respect to actual image
xtemp = x; ytemp = y; ztemp = z;
x = double(ztemp); y = double(xtemp); z = double(ytemp);
x = x/1000; y = (y-320)*x/570.3; z = (z-240)*x/570.3; % in m
CurrentState(5,:) = [x,y,z];
CurrentState(2,:) = CurrentState(5,:);
end