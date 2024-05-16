clear
theta_deg = 0;
theta = theta_deg*pi/180;

gamma_deg = 0;
gamma = gamma_deg*pi/180;

alpha_deg = 170;
alpha = alpha_deg*pi/180;
z = 45;

%radius = 35/2
%640*480

cameraMatrix = [
    635, 0, 346; %Found from camera calibration
    0, 636, 226;
    0, 0, 1];

translationMatrix = [ %not used, visualization purpose
    1,0,0,0;
    0,1,0,0;
    0,0,1,45; %cm
    0,0,0,1]; 

transformMatrix = [
    cos(gamma)*cos(alpha), sin(gamma)*sin(theta)*cos(alpha)-sin(alpha)*cos(theta), cos(alpha)*sin(gamma)*cos(theta)+sin(alpha)*sin(theta), 0;
    sin(alpha)*cos(gamma), sin(gamma)*sin(theta)*sin(alpha)+cos(alpha)*cos(theta), sin(alpha)*sin(gamma)*cos(theta)-sin(theta)*cos(alpha), 0;
    -sin(gamma), cos(gamma)*sin(theta), cos(gamma)*cos(theta), -45;
    0, 0, 0, 1];

z_matrix = [
    z, 0, 0;
    0, z, 0;
    0, 0, z;    
    0, 0, 1];

pixelPoint = [344 292 1]'

realWorldPos = transformMatrix*z_matrix*inv(cameraMatrix)*pixelPoint