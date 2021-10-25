function [controls,NewState,OUTOFFRAME] = amalgamfunctionforcrazyflie20(ControlSettings,CurrentState,dt,setpoint,yaw_degrees,vbat,time,startingpoint)
%% Locates quadcopter, updates state, and computes new control parameters
%% Input
% ControlSettings: [thrust,roll,pitch,yawrate]
% CurrentState: most recent information on the following: [ex,ey,ez;
%                                                         x,y,z;
%                                                         vx,vy,vz;
%                                                         cam,yaw,bat;
%                                                         rawx,rawy,rawz]
% dt: change in time since last use of this function
% setpoint: desired location of quadcopter [x,y,z,yaw] (yaw = 0)
%   Unused; setpoint/trajectory computed here, not Python script
% yaw_degrees: the current yaw of the quadcopter, with respect to when it was first turned on
% vbat: the current voltage level of the battery
% time: time since the control loop was initiated
% startingpoint: location of setpoint: [x,y,z]
%% Output
% controls: quadcopter control parameters: [thrust, roll, pitch, yawrate]
% NewState: updated quadcopter state: [ex,ey,ez;
%                                      x,y,z;
%                                      vx,vy,vz;
%                                      cam,yaw,bat;
%                                      rawx,rawy,rawz]
% OUTOFFRAME: whether quadcopter has left frame of Kinect camera
%% Set Point Parameters
timer = 30;
delay = 5;
T = (timer-delay)/2;
w = 2*pi/T;
R = 0.3;
setpoint = zeros(1,4);
setpoint(1:3) = startingpoint;
%% Hover
if time < delay
    setpoint(3) = startingpoint(3) + 0.5 * time/delay;
else
    setpoint(3) = startingpoint(3) + 0.5;
end
%% Escalate
% setpoint(3) = startingpoint(3)+time/(timer+10);
%% Oscillate in y
% if time < delay
%     setpoint(3) = startingpoint(3)+0.5*time/delay;
% else
%     setpoint(2) = startingpoint(2)+0.4*sin(w*time);
%     setpoint(3) = startingpoint(3)+0.5;
% end
%% Circle
% if time < delay
%     setpoint(3) = startingpoint(3) + 0.2*(time/delay);
% else
%     setpoint(2) = startingpoint(2) + R*sin(w*(time-delay));
%     setpoint(3) = startingpoint(3) + 0.5 - R*cos(w*(time-delay));
% end
%% Acquire and update data
% Frame offset between RGB image and depth image
xadjust = -10;
yadjust = -10;
% Size of cropped image
radius = 125;
% Low pass filter parameter
wn = 2*pi*9;
% Detecting quadcopter
Rthresh = 100;
Gthresh = 50;
Bthresh = 150;
% Initialize NewState matrix
NewState = zeros(5,3);
% Get picture from video streams
vid1 = imaqfind('name','RGB_640x480-kinect-1');
vid2 = imaqfind('name','Depth_640x480-kinect-2');
pic = getsnapshot(vid1{1});
depth_pic = getsnapshot(vid2{1});
% Detect red, make image binary
[row, col] = find(pic(:,:,2)<Gthresh & pic(:,:,3) < Bthresh & pic(:,:,1) > Rthresh);
if ~(isempty(row) || isempty(col))
    mediancol = mode(col); medianrow = mode(row); % mode gives best performance
    approxx = mediancol+xadjust; approxy = medianrow+yadjust;
    [rowrange,colrange] = rangegetter(approxy,approxx,radius); % compute range containing quadcopter
    [x,y,z] = DepthAprox(depth_pic(rowrange,colrange)); % locates quadcopter in cropped image
    x = colrange(1)-1+x; y = 481-(rowrange(1)-1+y); % locates quadcopter in whole image
    xtemp = x; ytemp = y; ztemp = z;
    x = double(ztemp); y = double(xtemp); z = double(ytemp);
    x = x/1000; y = (y-320)*x/570.3; z = (z-240)*x/570.3; % Convert location to metric; values derived from calibration
    NewState(5,1) = x; NewState(5,2) = y; NewState(5,3) = z;
    % Lowpass filter to remove noise:
    NewState(3,:) = (CurrentState(3,:) + dt*wn^2*(NewState(5,:)-CurrentState(2,:)))/(1+ dt*2*wn+dt^2*wn^2);
    NewState(2,:) = CurrentState(2,:) + NewState(3,:)*dt;
    NewState(1,:) = CurrentState(1,:) + dt*(NewState(2,:)-setpoint(1:3));
end
%% Compute control parameters
% Values acquired through tuning
KPz = 8e3;
KIz = 4e3;
KDz = 8e3;
KPxy = 5e3;
KIxy(1) = 0.75e3;
KIxy(2) = 0.75e3;
KDxy = 0;

CurrentState = NewState;
mass_of_quad = 40.5; % in grams
weight_of_quad = mass_of_quad*9.81;
% liftoffthrust gives the thrust needed to suspend quad in the air
% vbat is in volts
% Constants acquired through calibration
liftoffthrust = (weight_of_quad/(cosd(ControlSettings(2))*cosd(ControlSettings(3)))+913.8795-201.9946*vbat)/0.0132;
yaw = yaw_degrees*pi/180;
% Thrust PID controller
thrust = (liftoffthrust -KIz*CurrentState(1,3)...
    - KPz*(CurrentState(2,3)-setpoint(3))...
    - KDz*CurrentState(3,3));
if thrust>60000
    thrust = 60000;
end
% Add trim to stabalize quadcopter during takeoff
timeconstr = 10;
timeconstp = 10;
pitch_trim = 4*exp(-time/timeconstp);
roll_trim = 4*exp(-time/timeconstr);
% Roll and pitch PID controller
xycorr = KIxy(1:2).*CurrentState(1,1:2)...
    + KPxy*(CurrentState(2,1:2)-setpoint(1:2))...
    + KDxy*CurrentState(3,1:2);
roll = (xycorr(2)*cos(yaw)-xycorr(1)*sin(yaw))*180/pi/12000;
pitch = -(xycorr(1)*cos(yaw)+xycorr(2)*sin(yaw))*180/pi/12000;
pitch = pitch + pitch_trim;
roll = roll + roll_trim;
% Limit pitch and roll
if (pitch>30) pitch = 30; end
if (pitch<-30) pitch = -30; end
if (roll>30) roll = 30; end
if (roll<-30) roll = -30; end
% Yaw P controller
feedbackgainyaw = 1;
yawrate = feedbackgainyaw*sin(yaw-setpoint(4)*pi/180)*180/pi;
% Land at end of run
if (time>timer)
    thrust = thrust*exp(-(time-timer)/8);
end
OUTOFFRAME = 0;
% Failsafe: Land drone when out of frame
if  all(all(CurrentState == 0)) ||...
        x <= 0.8 || x >= 1.75 ||...
        y <= (0-320)*x/570.3 || y >= (640-320)*x/570.3 ||...
        z <= (0-240)*x/570.3 || z >= (480-240)*x/570.3
    thrust = ControlSettings(1)-dt*20000;
    if thrust < 0
        thrust = 0;
    end
    pitch = pitch_trim;
    roll = roll_trim;
    OUTOFFRAME = 1;
end
% Return control parameters to Python script
controls = [thrust,roll,pitch,yawrate];
end