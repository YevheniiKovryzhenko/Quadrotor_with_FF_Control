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

function rate_ref = att_quat_control(q, q_ref, yaw_rate_ref, proportional_gain, rate_limit)

rate_ref = zeros(3,1);
qd = q_ref(:)';

% // calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
e_z = lib.math.q2dcm_z(q);
e_z_d = lib.math.q2dcm_z(qd);
qd_red = lib.math.quat_error_from_vec(e_z, e_z_d);

if (abs(qd_red(2)) > (1 - 1e-5) || abs(qd_red(3)) > (1 - 1e-5))
	% // In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
	% // full attitude control anyways generates no yaw input and directly takes the combination of
	% // roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
	qd_red = qd;

else
	% // transform rotation from current to desired thrust vector into a world frame reduced desired attitude
    qd_red = quatmultiply(qd_red, q);
end

% // mix full and reduced desired attitude
q_mix = quatmultiply(quatinv(qd_red),qd);
q_mix = lib.math.get_cannonical_quat(q_mix);
% // catch numerical problems with the domain of acosf and asinf
q_mix(1) = lib.math.constrain(q_mix(1), -1, 1);
q_mix(4) = lib.math.constrain(q_mix(4), -1, 1);
qd = quatmultiply(qd_red, [cos(acos(q_mix(1))), 0, 0, sin(asin(q_mix(4)))]);

% // quaternion attitude control law, qe is rotation from q to qd
qe = quatmultiply(quatinv(q), qd);

% // using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
% // also taking care of the antipodal unit quaternion ambiguity
tmp = 2 * lib.math.get_cannonical_quat(qe);
eq = tmp(2:4);
% // calculate angular rates setpoint
rate_ref(:) = eq(:).*proportional_gain(:);

% // Feed forward the yaw setpoint rate.
% // yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
% // but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
% // Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
% // and multiply it by the yaw setpoint rate (yawspeed_setpoint).
% // This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
% // such that it can be added to the rates setpoint.
% if (is_finite(_yawspeed_setpoint))
	rate_ref = rate_ref + lib.math.q2dcm_z(quatinv(q)) * yaw_rate_ref;
% end

% // limit rates
for i = 1:3
	rate_ref(i) = lib.math.constrain(rate_ref(i), -rate_limit(i), rate_limit(i));
end
end