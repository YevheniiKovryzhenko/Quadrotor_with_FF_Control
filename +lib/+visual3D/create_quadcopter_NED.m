% /****************************************************************************
%  *
%  *    Copyright (C) 2024  Yevhenii Kovryzhenko. All rights reserved.
%  *
%  *    This program is free software: you can redistribute it and/or modify
%  *    it under the terms of the GNU Affero General Public License as published by
%  *    the Free Software Foundation, either version 3 of the License, or
%  *    (at your option) any later version.
%  *
%  *    This program is distributed in the hope that it will be useful,
%  *    but WITHOUT ANY WARRANTY; without even the implied warranty of
%  *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  *    GNU Affero General Public License Version 3 for more details.
%  *
%  *    You should have received a copy of the
%  *    GNU Affero General Public License Version 3
%  *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
%  *
%  *    1. Redistributions of source code must retain the above copyright
%  *       notice, this list of conditions, and the following disclaimer.
%  *    2. Redistributions in binary form must reproduce the above copyright
%  *       notice, this list of conditions, and the following disclaimer in
%  *       the documentation and/or other materials provided with the
%  *       distribution.
%  *    3. No ownership or credit shall be claimed by anyone not mentioned in
%  *       the above copyright statement.
%  *    4. Any redistribution or public use of this software, in whole or in part,
%  *       whether standalone or as part of a different project, must remain
%  *       under the terms of the GNU Affero General Public License Version 3,
%  *       and all distributions in binary form must be accompanied by a copy of
%  *       the source code, as stated in the GNU Affero General Public License.
%  *
%  ****************************************************************************/

function animation_3D = create_quadcopter_NED(fig, p, scaling)
p = p(:);
x = p(1,1);
y = p(2,1);
z = p(3,1);

animation_3D.init.x = x;
animation_3D.init.y = y;
animation_3D.init.z = z;
animation_3D.scaling = scaling;
animation_3D.fig = fig;

ax = gca(fig);
% hold(ax,"on")
h = hgtransform('Parent',ax); 

% axis(ax,"equal")


% This Animation code is for QuadCopter. Written by Jitendra Singh 
%% Define design parameters
D2R = pi/180;
R2D = 180/pi;
b   = 0.6*scaling;   % the length of total square cover by whole body of quadcopter in meter
a   = b/3;   % the legth of small square base of quadcopter(b/4)
H   = 0.06*scaling;  % hight of drone in Z direction (4cm)
H_m = H+H/2; % hight of motor in z direction (5 cm)
r_p = b/4;   % radius of propeller

%% Conversions
ro = 45*D2R;                   % angle by which rotate the base of quadcopter
Ri = [cos(ro) -sin(ro) 0;
      sin(ro) cos(ro)  0;
       0       0       1];     % rotation matrix to rotate the coordinates of base 
base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base 
           -a/2 -a/2 a/2 a/2;
             0    0   0   0];
base = Ri*base_co;             % rotate base Coordinates by 45 degree 
to = linspace(0, 2*pi);
xp = r_p*cos(to);
yp = r_p*sin(to);
zp = zeros(1,length(to));
 
%% Design Different parts
% design the base square
 drone(1) = patch(ax,[base(1,:)+x],[base(2,:)+y],[base(3,:)+z],'r');
 drone(2) = patch(ax,[base(1,:)+x],[base(2,:)+y],[base(3,:)+H+z],'r');
 alpha(drone(1:2),0.7);
% design 2 parpendiculer legs of quadcopter 
 [xcylinder ycylinder zcylinder] = cylinder([H/2 H/2]);
 drone(3) =  surface(ax,b*zcylinder-b/2+x,ycylinder+y,xcylinder+H/2+z,'facecolor','b');
 drone(4) =  surface(ax,ycylinder+x,b*zcylinder-b/2+y,xcylinder+H/2+z,'facecolor','b') ; 
 alpha(drone(3:4),0.6);
% design 4 cylindrical motors 
 drone(5) = surface(ax,xcylinder+b/2+x,ycylinder+y,H_m*zcylinder+H/2+z,'facecolor','r');
 drone(6) = surface(ax,xcylinder-b/2+x,ycylinder+y,H_m*zcylinder+H/2+z,'facecolor','r');
 drone(7) = surface(ax,xcylinder+x,ycylinder+b/2+y,H_m*zcylinder+H/2+z,'facecolor','r');
 drone(8) = surface(ax,xcylinder+x,ycylinder-b/2+y,H_m*zcylinder+H/2+z,'facecolor','r');
 alpha(drone(5:8),0.7);
% design 4 propellers
 drone(9)  = patch(ax,xp+b/2+x,yp+y,zp+(H_m+H/2)+z,'c','LineWidth',0.5);
 drone(10) = patch(ax,xp-b/2+x,yp+y,zp+(H_m+H/2)+z,'c','LineWidth',0.5);
 drone(11) = patch(ax,xp+x,yp+b/2+y,zp+(H_m+H/2)+z,'p','LineWidth',0.5);
 drone(12) = patch(ax,xp+x,yp-b/2+y,zp+(H_m+H/2)+z,'p','LineWidth',0.5);
 alpha(drone(9:12),0.3);
%% create a group object and parent surface
set(drone,'parent',h)
% view(ax,-45,20)
% set (ax, 'ydir', 'reverse')
% hold(ax,"off")

animation_3D.axes = ax;
animation_3D.object = h;
end