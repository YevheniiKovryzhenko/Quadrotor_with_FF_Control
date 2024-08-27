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

function rpy_rate_fb   = manual_att_control(q, att_ref)
roll_ref = att_ref(1);
pitch_ref = att_ref(2);
yaw_ref = att_ref(3);
att = quat2eul(q,"ZYX");
q_rp_fb = quatmultiply(quatconj(q), eul2quat([att(1), pitch_ref, roll_ref], "ZYX")); %in inertial XYZ
q_y_fb = quatmultiply(quatconj(q), eul2quat([yaw_ref, att(2), att(3)], "ZYX")); %in inertial XYZ


if q_rp_fb(1) < 0
    q_rp_fb = quatconj(q_rp_fb);
end
if q_y_fb(1) < 0
    q_y_fb = quatconj(q_y_fb);
end

axis_err_ = [q_rp_fb([2,3])'; q_y_fb(4)];

tmp_sign = sign(axis_err_);

axis_err_ = tmp_sign.*sqrt(abs(axis_err_));

rpy_rate_fb = axis_err_(:);
end