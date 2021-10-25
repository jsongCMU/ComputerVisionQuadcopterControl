function startvid
% Starts video aquisition
% Similar to setup.m, but no preview
vid1 = videoinput('kinect',1);
triggerconfig(vid1,'manual');
vid2 = videoinput('kinect',2);
triggerconfig(vid2,'manual');
start(vid1);
start(vid2);
end