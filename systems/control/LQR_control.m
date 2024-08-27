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

function [Tau_des, T_des, p_fb_out] = LQR_control(...
    pi, vi, q, omega, K,...
    p_ref, v_ref, att_ref, omega_ref,...
    T_ref, Tau_ref, p_fb_int)%#codegen

%% inputs %%
pi_ = pi(1:3,1);            %[m] estimated position (in inertial frame)
vi_ = vi(1:3,1);            %[m/s] estimated velocity (in inertial frame)
q_ = q(1,1:4);              %[qw qx qy qz] estimated quaternian attitude
omega_ = omega(1:3,1);      %[rad/s] estimated body rotation rates
K_ = K(1:4,1:12);           %gain matrix
p_ref_ = p_ref(1:3,1);      %[m] reference position (in inertial frame)
v_ref_ = v_ref(1:3,1);      %[m/s] reference velocity (in inertial frame)
att_ref_ = att_ref(1:3,:);  % reference attitude
omega_ref_ = ...
    omega_ref(1:3,1);       %[rad/s] reference body rates
p_fb_int_ = p_fb_int(1:6,1); %integral of the position feedback term 

%% outputs %%
T_des = 0;                  %[N] desired thrust
Tau_des = zeros(3,1);       %[N-m] desired torque
p_fb_out = zeros(6,1);  %we will integrate this

%% calculations
p_fb_ = (p_ref_ - pi_);
p_fb_out(1:3) = p_fb_(:);
p_fb_ = p_fb_ + p_fb_int_(1:3);

v_fb_ = (v_ref_ - vi_);

T_r_ = T_ref(1);
Tau_r_ = [Tau_ref(1); Tau_ref(2); Tau_ref(3)];
U_r_ = [T_r_;Tau_r_];

[eul_] = lib.math.quat2eul(q_,"ZYX");
roll_ = eul_(3);
pitch_ = eul_(2);
yaw_ = eul_(1);

q_yaw_ = eul2quat([yaw_, 0, 0],"ZYX");
p_fb_ = lib.math.quatrotate(q_yaw_,p_fb_);
v_fb_ = lib.math.quatrotate(q_yaw_,v_fb_);

att_fb_ = [...
    lib.math.get_angle_err(att_ref_(1), roll_)
    lib.math.get_angle_err(att_ref_(2), pitch_)
    lib.math.get_angle_err(att_ref_(3), yaw_)];

p_fb_out(4:6) = att_fb_(:);
att_fb_ = att_fb_ +  p_fb_int_(4:6);

X_err_ = [...
    -p_fb_;
    -v_fb_;
    -att_fb_;
    (omega_(:) - omega_ref_(:))];

U_ = U_r_-K_*X_err_; %control
T_des(1) = U_(1);
Tau_des(1:3) = U_(2:4);
end

function out = sqrt_ctr_(in)
    out = sign(in) .*sqrt(abs(in));
end