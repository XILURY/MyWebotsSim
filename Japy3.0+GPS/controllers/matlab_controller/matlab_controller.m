% MATLAB controller for Webots
% File:          matlab_controller.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
desktop;
keyboard;

TIME_STEP = 5;

% get and enable devices, e.g.:
%  camera = wb_robot_get_device('camera');
%  wb_camera_enable(camera, TIME_STEP);
%  motor = wb_robot_get_device('motor');
RF1_motor = wb_robot_get_device('RF1_motor');
RF2_motor = wb_robot_get_device('RF2_motor');
RF3_motor = wb_robot_get_device('RF3_motor');

RH1_motor = wb_robot_get_device('RH1_motor');
RH2_motor = wb_robot_get_device('RH2_motor');
RH3_motor = wb_robot_get_device('RH3_motor');

LF1_motor = wb_robot_get_device('LF1_motor');
LF2_motor = wb_robot_get_device('LF2_motor');
LF3_motor = wb_robot_get_device('LF3_motor');

LH1_motor = wb_robot_get_device('LH1_motor');
LH2_motor = wb_robot_get_device('LH2_motor');
LH3_motor = wb_robot_get_device('LH3_motor');

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
while wb_robot_step(TIME_STEP) ~= -1
    [t,x,y] = sim('matlab_controller',[],[],[]);
     wb_motor_set_position(RF1_motor,try123);
  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);

  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
  %  wb_motor_set_postion(motor, 10.0);

  % if your code plots some graphics, it needs to flushed like this:
  % drawnow;

end

% cleanup code goes here: write data to files, etc.
