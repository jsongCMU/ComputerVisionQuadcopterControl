% Sets up video stream and shows preview
startvid
vid1 = imaqfind('name','RGB_640x480-kinect-1');
vid1 = vid1{1};
vid2 = imaqfind('name','Depth_640x480-kinect-2');
vid2 = vid2{1};
preview(vid1);
preview(vid2);