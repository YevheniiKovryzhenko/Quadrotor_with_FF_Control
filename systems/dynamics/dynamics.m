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

function [p_dot, v_dot, q_dot, omega_dot] ...
    = dynamics(...
    v, q, omega, tau_gyro,...
    T, Tau,...
    g, D, A, B, k_h, J)


%% inputs %%
J_ = J(1:3,1:3);        % moment of inertia matrix
D_ = diag(D(1:3));      % drag coefficient
g_ = g(1);              %[m/s2] acceleration due to gravity
A_ = A(1:3,1:3);        % velocity-dependant drag coefficients
B_ = B(1:3,1:3);        % omega-dependant drag coefficients
k_h_ = k_h(1);          % constant for a quadratic velocity-dependent input disturbance

v_ = v(1:3);            %[m/s]  velocity
q_ = q(1:4);            %[qw qx qy qz]  quaternian attitude
omega_ = omega(1:3);    %[rad/s]  body rotation rates
tau_gyro_ = ...
    tau_gyro(1:3);      %[N-m]  gyroscopic torque

T_ = T(1);              %[N] thrust
Tau_ = Tau(1:3);        %[N-m] torque

%% outputs %%
p_dot = zeros(3,1);
v_dot = zeros(3,1);
q_dot = rotm2quat(eye(3));
omega_dot = zeros(3,1);

%% hardcoded for now %%
x_w_ = [1;0;0];
y_w_ = [0;1;0];
z_w_ = [0;0;1];

%% calculations %
R_ = quat2rotm(q_);
x_B_ = R_*x_w_;
y_B_ = R_*y_w_;
z_B_ = R_*z_w_;

p_dot(1:3) = v_(1:3);

c_ = T_ + k_h_*(v_.'*(x_B_ + y_B_))^2;
v_dot(1:3) = -g_*z_w_ + c_*z_B_ - R_*D_*R_.'*v_;

Lambda_ = 0.5 *[...
    0, -omega_(1), -omega_(2), -omega_(3)
    omega_(1), 0, omega_(3), -omega_(2)
    omega_(2), -omega_(3), 0, omega_(1)
    omega_(3), omega_(2), -omega_(1), 0];
q_dot(1:4) = Lambda_*q_.';

omega_dot(1:3) = J\(Tau_ - cross(omega_, J_*omega_) - tau_gyro_...
    - A_*R_.'*v_ - B_*omega_);



end