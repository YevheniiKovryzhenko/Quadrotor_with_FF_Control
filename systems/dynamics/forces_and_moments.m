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

function [F_b, F_e, M_b] ...
    = forces_and_moments(...
    v_b, q, omega, tau_gyro,...
    T, Tau,...
    g, D, A, B, k_h,...
    m)


%% inputs %%
% J_ = J(1:3,1:3);        % moment of inertia matrix
D_ = D(1:3,1:3);      % drag coefficient
g_ = g(1);              %[m/s2] acceleration due to gravity
A_ = A(1:3,1:3);        % velocity-dependant drag coefficients
B_ = B(1:3,1:3);        % omega-dependant drag coefficients
k_h_ = k_h(1);          % constant for a quadratic velocity-dependent input disturbance
m_ = m(1);              % [kg] mass of the vehicle 

v_b_ = v_b(1:3);          %[m/s]  velocity in the body frame
q_ = q(1:4);            %[qw qx qy qz]  quaternian attitude
omega_ = omega(1:3);    %[rad/s]  body rotation rates
tau_gyro_ = ...
    tau_gyro(1:3);      %[N-m]  gyroscopic torque

T_ = T(1);              %[N] thrust
Tau_ = Tau(1:3);        %[N-m] torque

%% outputs %%
F_b = zeros(3,1);
F_e = zeros(3,1);
M_b = zeros(3,1);

%% hardcoded for now %%
x_w_ = [1;0;0];
y_w_ = [0;1;0];
% z_w_ = [0;0;-1];

%% calculations %
R_ = quat2rotm(q_(:).');
x_B_ = R_*x_w_;
y_B_ = R_*y_w_;
% z_B_ = R_*z_w_;

c_ = T_ + k_h_*(v_b_.'*(x_B_ + y_B_))^2 * m_;
F_b(1:3) = [0;0; c_] + D_*v_b_*m_;
F_e(1:3) = [0;0;g_*m_];
M_b(1:3) = Tau_ + tau_gyro_ + A_*v_b_ - B_*omega_;

end