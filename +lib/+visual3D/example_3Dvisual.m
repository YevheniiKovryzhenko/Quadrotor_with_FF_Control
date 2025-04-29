clear all; close all; clc
fig = figure; % your figure with whatever format and static details you would like to plot over

%%%%%%%%%%%%%%%%%%%%%%%%%
pause_dt = 0.02; %sec
write_gif = false;
scaling = 2;


%generate some fake data:
t = 0:pause_dt:10; %sec
n = length(t);
pos = repmat(linspace(0, 5, n),[3,1]); %[x y z]
q = repmat([1,0,0,0],[n,1])'; % [w x y z]

plot3(pos(1,:),pos(2,:),pos(3,:),'-b')


animation_3D = lib.visual3D.create_quadcopter_NED(fig,pos(:,1),scaling); %create an object handle


axis(gca(fig),'equal') %make sure you do this for the 3D shapes to look nice -> (axis equal)    

lib.visual3D.move_3D(t,pos,q, animation_3D, ...
    write_gif, './Saves/', fig, pause_dt)