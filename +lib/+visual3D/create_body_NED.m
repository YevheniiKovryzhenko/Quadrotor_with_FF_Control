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

function animation_3D = create_body_NED(fig, p, scaling)
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
h = hgtransform('Parent',ax); 
hold(ax,"on")
axis equal
lib.visual3D.plot_box([x(1),y(1),z(1)],ones(1,3)*scaling,h)
plot3([x(1),x(1)+2*scaling],[y(1),y(1)],[z(1),z(1)],'LineWidth',7,'Color',[1,0,0],'Parent',h)
plot3([x(1),x(1)],[y(1),y(1)],[z(1),z(1)-1.5*scaling],'LineWidth',2,'Color',[0,0,0],'Parent',h)
plot3([x(1),x(1)],[y(1),y(1)+1.5*scaling],[z(1),z(1)],'LineWidth',2,'Color',[0,0,0],'Parent',h)
% view(-45,20)
set (ax, 'ydir', 'reverse')
hold(ax,"off")

animation_3D.axes = ax;
animation_3D.object = h;
end