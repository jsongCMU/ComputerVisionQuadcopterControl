function [new_picture,row,col] = redfilterv2(pic,redthresh,greenthresh,bluethresh)
% Turns rgb picture into black and white, where red is white and everything
% else is black
% Best if following is true: redthresh > bluethresh > greenthresh
new_picture = zeros(480,640);
[row, col] = find(pic(:,:,2)<greenthresh & pic(:,:,3) < bluethresh & pic(:,:,1) > redthresh);
new_picture(480*(col-1)+row) = 255;
end