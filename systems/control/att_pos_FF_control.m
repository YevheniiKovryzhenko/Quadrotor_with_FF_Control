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

function [omega_dot_des, T_des, p_fb_out] = att_pos_FF_control(...
    p_ref, v_ref, a_ref, psi_ref, ...
    q_ref, omega_ref, ...
    K_pos, K_pos_int, K_vel, K_acc,...
    K_att, K_omega,...
    pi, vi, ai, q, omega,...
    p_fb_int)%#codegen

%% inputs %%
p_ref_ = p_ref(1:3,1);      %[m] reference position (in inertial frame)
v_ref_ = v_ref(1:3,1);      %[m/s] reference velocity (in inertial frame)
a_ref_ = a_ref(1:3,1);      %[m/s2] reference acceleration (in inertial frame)
% psi_ref_ = psi_ref(1,1);    %[rad] reference heading

% J_ = J(1:3,1:3);            % moment of inertia matrix
% D_ = diag(D(1,1:3));        % drag coefficient
% g_ = g(1,1);                %[m/s2] acceleration due to gravity
K_pos_ = ...
    K_pos(1:3,1:3);     % position controller gains
K_pos_int_ = ...
    K_pos_int(1:3,1:3); % position controller integral gains
K_vel_ = ...
    K_vel(1:3,1:3);     % velocity controller gains
K_acc_ = ...
    K_acc(1:3,1:3);     % acceleration controller gains
K_att_ = ...
    K_att(1:3,1:3);     % attitude controller gains
K_omega_ = ...
    K_omega(1:3,1:3);   % body rate controller gains
% A_ = A(1:3,1:3);            % velocity-dependant drag coefficients
% B_ = B(1:3,1:3);            % omega-dependant drag coefficients
% k_h_ = k_h(1,1);            % constant for a quadratic velocity-dependent input disturbance

q_ref_ = q_ref(1,1:4);      % reference attitude quaternian
omega_ref_ = ...
    omega_ref(1:3,1);       %[rad/s] reference body rates
% omega_dot_ref_ = ...
%     omega_dot_ref(1:3,1);   %[rad/s2] reference body rotational acceleration
% Tau_ref_ = Tau_ref(1:3,1);  %[N-m] reference torque
% T_ref_ = T_ref(1,1);        %[N] reference collective thrust

pi_ = pi(1:3,1);            %[m] estimated position (in inertial frame)
vi_ = vi(1:3,1);            %[m/s] estimated velocity (in inertial frame)
ai_ = ai(1:3,1);            %[m/s2] estimated acceleration (in inertial frame)
q_ = q(1,1:4);              %[qw qx qy qz] estimated quaternian attitude
omega_ = omega(1:3,1);      %[rad/s] estimated body rotation rates
% tau_gyro_ = ...
    % tau_gyro(1:3,1);        %[N-m] estimated gyroscopic torque due to motor spin

p_fb_int_ = p_fb_int(1:3,1); %integral of the position feedback term  

%% outputs %%
% T_des = 0;                  %[N] desired thrust
% Tau_des = zeros(3,1);       %[N-m] desired torque
% p_fb = zeros(3,1);
% v_fb = zeros(3,1);
% a_fb_ = zeros(3,1);

% att_fb = zeros(3,1);
% omega_fb = zeros(3,1);
omega_dot_des = zeros(3,1);
T_des = 0;
p_fb_out = zeros(3,1); %we will integrate this

%% Positon Control %%
MAX_XYZ_POS_ERR = 10.0;
MAX_XYZ_VEL_ERR = 5.0;
MAX_XYZ_ACC_ERR = 3;


%feedback control:
p_fb_ = (p_ref_ - pi_);
p_fb_out(:) = p_fb_(:);
p_fb_ = K_pos_ * (saturate_err_vec_(p_fb_, -MAX_XYZ_POS_ERR, MAX_XYZ_POS_ERR)) + K_pos_int_*p_fb_int_;

v_fb_ = (v_ref_ - vi_);
v_fb_ = K_vel_ * (saturate_err_vec_(v_fb_, -MAX_XYZ_VEL_ERR, MAX_XYZ_VEL_ERR));

a_fb_ = (a_ref_ - ai_);
a_fb_ = K_acc_ * (saturate_err_vec_(a_fb_, -MAX_XYZ_ACC_ERR, MAX_XYZ_ACC_ERR));

%feedback component:
att_dir_correction_from_pos_ = a_fb_ + v_fb_ + p_fb_; %from position controller

%thrust feedback:
T_des(1) = [att_dir_correction_from_pos_(1),att_dir_correction_from_pos_(2),att_dir_correction_from_pos_(3)]*lib.math.quatrotate(q_, [0;0;-1]);

%% Attitude Control %%
[eul_] = lib.math.quat2eul(q_,"ZYX");
roll_ = eul_(3);
pitch_ = eul_(2);
yaw_ = eul_(1);

%extract x and y errors (in inertial frame)
roll_correction_ = [-att_dir_correction_from_pos_(1),att_dir_correction_from_pos_(2),0]*lib.math.quatrotate(q_, [0;1;0]);
pitch_correction_ =  [-att_dir_correction_from_pos_(1),att_dir_correction_from_pos_(2),0]*lib.math.quatrotate(q_, [1;0;0]);

% get reference roll pitch and yaw:
[eul_ref_] = lib.math.quat2eul(q_ref_,"ZYX");
roll_ref_ = eul_ref_(3);
pitch_ref_ = eul_ref_(2);
yaw_ref_ = eul_ref_(1);

%get attitude feeback:
eul_rpy_fb_   = manual_att_control(q_, [roll_ref_ + roll_correction_, pitch_ref_ + pitch_correction_, yaw_ref_]);

% add the rate control:
omega_fb_ = sqrt_ctr_(omega_ref_ - omega_);
% omega_fb_ = nthrt_ctr_(omega_ref_ - omega_);
% omega_fb_ = 6*(omega_ref_ - omega_);
omega_dot_des(:) = K_omega_ * omega_fb_(:)  + K_att_ * eul_rpy_fb_(:);
end

function out = saturate_err_vec_(in, out_min, out_max)
    n = length(in);
    if ~isequal(length(out_min),n)
        out_min_ = out_min(1)*ones(1,n);
    else
        out_min_ = out_min(:)';
    end
    if ~isequal(length(out_max),n)
        out_max_ = out_max(1)*ones(1,n);
    else
        out_max_ = out_max(:)';
    end
    out = zeros(n,1);
    for i = 1:n
        out(i) = saturate_err__(in(i), out_min_(i), out_max_(i));
    end

    function out = saturate_err__(in, out_min, out_max)
        in_ = in(1);
        if in_ > out_max
            out = 1;
            return;
        elseif in_ < out_min
            out = -1;
            return;
        else
            out = in_ / (out_max - out_min);
            return;
        end
    end
end
function out = sqrt_ctr_(in)
    out = sign(in) .*sqrt(abs(in));
end
function out = nthrt_ctr_(in)
    out = sign(in) .*nthroot(abs(in),3);
end