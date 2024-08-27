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

function [q, omega, omega_dot, T, Tau] = ...
    get_flat_states(...
    v, a, jerk, snap, psi, psi_dot, psi_ddot,...
    tau_gyro,...
    g, D, A, B, k_h, J, ...
    q_old, reset_fl)%#codegen
%% inputs %%
v_ = v(1:3,1);              %[m/s]  velocity
a_ = a(1:3,1);              %[m/s2] acceleration
jerk_ = jerk(1:3,1);        %[m/s3] jerk
snap_ = snap(1:3,1);        %[m/s4] snap      
psi_ = psi(1,1);            %[rad] heading angle
psi_dot_ = psi_dot(1,1);    %[rad/s] heading angle rate
psi_ddot_ = psi_ddot(1,1);  %[rad/s2] heading angle acceleration
tau_gyro_ = ...
    tau_gyro(1:3,1);        %[N-m]  gyroscopic torque

J_ = J(1:3,1:3);            % moment of inertia matrix
D_ = D(1:3,1:3);            % drag coefficient
g_ = g(1,1);                %[m/s2] acceleration due to gravity
A_ = A(1:3,1:3);            % velocity-dependant drag coefficients
B_ = B(1:3,1:3);            % omega-dependant drag coefficients
k_h_ = k_h(1,1);            % constant for a quadratic velocity-dependent input disturbance

q_old_ = q_old(1,1:4);      %last valid orientation (quaternion)

%% outputs %%
% R_ = zeros(3);           % rotation matrix
q = zeros(1,4);
omega = ...
    zeros([3,1]);       %[rad/s] body rates
omega_dot = ...
    zeros([3,1]);       %[rad/s2] body rotational acceleration
T = 0;                  %[m/s2] mass-normalized thrust
Tau = zeros(3,1);       %[N-m] torque

if reset_fl
    q(:) = eul2quat([psi,0,0],"ZYX");
    T = g_;
    return;
end

%% FRD 2 FLU Frame %%
v_ = lib.macro.FRD2FLU_vec(v_);
a_ = lib.macro.FRD2FLU_vec(a_);
jerk_ = lib.macro.FRD2FLU_vec(jerk_);
snap_ = lib.macro.FRD2FLU_vec(snap_);
tau_gyro_ = lib.macro.FRD2FLU_vec(tau_gyro_);
psi_ = -psi_;
psi_dot_ = -psi_dot_;
psi_ddot_ = -psi_ddot_;
q_old_ = lib.macro.FRD2FLU_quat(q_old_(:)).';

%% hardcoded for now %%
% x_w_ = [1;0;0];
% y_w_ = [0;1;0];
z_w_ = [0;0;1];

%% calculations %%
d_x_ = D_(1);
d_y_ = D_(2);
d_z_ = D_(3);

R_old_ = quat2rotm(q_old_(:).');

% rotation matrix R
x_c_ = [cos(psi_); sin(psi_); 0];
y_c_ = [-sin(psi_); cos(psi_); 0];

alpha_ = a_ + g_*z_w_ + d_x_*v_;
beta_ = a_ + g_*z_w_ + d_y_*v_;

tmp_ = cross(y_c_,alpha_);
R_isbad_ = all(tmp_ == 0);
if R_isbad_
    disp("Bad norm, use last valid orientation");
    R_ = R_old_;
    x_b_ = R_(1:3,1);
    y_b_ = R_(1:3,2);
    z_b_ = R_(1:3,3);
else
    x_b_ = tmp_ / norm(tmp_);
    tmp_ = cross(beta_, x_b_);
    R_isbad_ = R_isbad_ | all(tmp_ == 0);
    if R_isbad_
        disp("Bad norm, use last valid orientation");
        R_ = R_old_;
        x_b_ = R_(1:3,1);
        y_b_ = R_(1:3,2);
        z_b_ = R_(1:3,3);
    else
        if (z_w_.'*alpha_ < 0)
            disp("Inverted flight, heading will be off");
        end
    
        y_b_ = tmp_ / norm(tmp_);
    
        z_b_ = cross(x_b_, y_b_);
        
        R_ = [x_b_, y_b_, z_b_];
    end    
end

% mass-normalized thrust command
c_ = z_b_.'*(a_ + g_*z_w_ + d_z_*v_);
T(1) = c_ - k_h_*(v.' * (x_b_ + y_b_))^2;


% body angular rates
B1_ = c_ - (d_z_ - d_x_)*(z_b_.' * v_);
C1_ = -(d_x_ - d_y_)*(y_b_.'*v_);
D1_ = x_b_.'*jerk_ + d_x_*x_b_.'*a_;
A2_ = c_ + (d_y_ - d_z_) * (x_b_.'*v_);
C2_ = (d_x_ - d_y_) * (x_b_.' * v_);
D2_ = -y_b_.'*jerk_ - d_y_*y_b_.'*a_;
B3_ = -y_c_.'*z_b_;
C3_ = norm(cross(y_c_, z_b_));
D3_ = psi_dot_ * x_c_.' * x_b_;

omega_x_ = (-B1_*C2_*D3_ + B1_*C3_*D2_ - B3_*C1_*D2_ + B3_*C2_*D1_)/...
    (A2_*(B1_*C3_) - B3_*C1_);
omega_y_ = (-C1_*D3_ + C3_*D1_)/...
    (B1_*C3_ - B3_*C1_);
omega_z_ = (B1_*D3_ - B3_*D1_)/...
    (B1_*C3_ - B3_*C1_);

omega(1:3) = [...
    omega_x_
    omega_y_
    omega_z_];

are_bad_ = (isnan(omega)) | (isinf(omega));
if(any(are_bad_))
    omega(are_bad_) = 0;
end

% body angular acceleration
omega_skew_ = [...
    0, -omega_z_, omega_y_
    omega_z_, 0, -omega_x_
    -omega_y_, omega_x_, 0];

c_dot_ = z_b_.'*jerk_ + omega_x_*(d_x_ - d_z_)*(y_b_.'*v_)...
    +omega_y_*(d_z_-d_x_)*(x_b_.'*v_) + d_z_*z_b_.'*a_;
eta_ = R_*(omega_skew_^2*D_ + D_*omega_skew_^2 + 2*omega_skew_*D_*omega_skew_.')*R_.'*v_...
    +2*R_*(omega_skew_*D_ + D_*omega_skew_.')*R_.'*a_...
    + R_*D_*R_.'*jerk_;

E1_ = x_b_.'*snap_ - 2*c_dot_*omega_y_ - c_*omega_x_*omega_z_ + x_b_.'*eta_;
E2_ = -y_b_.'*snap_ - 2*c_dot_*omega_x_ + c_*omega_y_*omega_z_ - y_b_.'*eta_;
E3_ = psi_ddot_*x_c_.'*x_b_ + 2*psi_dot_*omega_z_*x_c_.'*y_b_...
    - 2*psi_dot_*omega_y_*x_c_.'*z_b_ - omega_x_*omega_y_*y_c_.'*y_b_...
    -omega_x_*omega_z_*y_c_.'*z_b_;

omega_dot_x_ = (-B1_*C2_*E3_ + B1_*C3_*E2_ - B3_*C1_*E2_ + B3_*C2_*E1_)/...
    (A2_*(B1_*C3_) - B3_*C1_);
omega_dot_y_ = (-C1_*E3_ + C3_*E1_)/...
    (B1_*C3_ - B3_*C1_);
omega_dot_z_ = (B1_*E3_ - B3_*E1_)/...
    (B1_*C3_ - B3_*C1_);

omega_dot(1:3) = [...
    omega_dot_x_
    omega_dot_y_
    omega_dot_z_];

Tau(1:3) = J_*omega_dot...
    + cross(omega,J_*omega)...
    + tau_gyro_...
    + A_*R_.'*v_ - B_*omega;

%% FLU 2 FRD Frame %%
omega = lib.macro.FLU2FRD_vec(omega);
omega_dot = lib.macro.FLU2FRD_vec(omega_dot);
Tau = lib.macro.FLU2FRD_vec(Tau);

q_ = rotm2quat(R_);
q(:) = lib.macro.FLU2FRD_quat(q_(:));
end