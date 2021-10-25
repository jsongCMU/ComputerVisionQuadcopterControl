function stopvid
% Terminates video streams
vid1 = imaqfind('name','RGB_640x480-kinect-1');
vid2 = imaqfind('name','Depth_640x480-kinect-2');
stop(vid1{1})
stop(vid2{1})
delete(vid1{1})
delete(vid2{1})
clear vid1 vid2
end