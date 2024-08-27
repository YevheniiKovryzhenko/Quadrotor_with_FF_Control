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

function q_sp = bodyzToAttitude(body_z, yaw_sp)
body_z_ = zeros(3,1);
body_z_(:) = body_z(1:3);
q_sp = zeros(1,4);
% // zero vector, no direction, set safe level value
if (norm(body_z_)^2 < 1.192093E-7)
	body_z_(3) = 1.0;
end

body_z_ = body_z_ / norm(body_z_);

% // vector of desired yaw direction in XY plane, rotated by PI/2
y_C = [-sin(yaw_sp); cos(yaw_sp); 0.0];

% // desired body_x axis, orthogonal to body_z
body_x = cross(y_C,body_z_);

% // keep nose to front while inverted upside down
if (body_z_(3) < 0.0)
	body_x = -body_x;
end

if (abs(body_z_(3)) < 0.000001)
	% // desired thrust is in XY plane, set X downside to construct correct matrix,
	% // but yaw component will not be used actually
	body_x([1,2]) = 0;
	body_x(3) = 1.0;
end

body_x = body_x / norm(body_x);

% // desired body_y axis
body_y = cross(body_z_, body_x);

R_sp = zeros(3,3);

% // fill rotation matrix
for i = 1:3
	R_sp(i, 1) = body_x(i);
	R_sp(i, 2) = body_y(i);
	R_sp(i, 3) = body_z_(i);
end

q_sp(:) = lib.math.dcm2quat(R_sp);
end