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

function res = reconstruct_all_flat_states(...
    sol, ...
    g, D, A, B, k_h, J)
n_pts = length(sol.t);
R_ref = zeros(3,3,n_pts);
omega_ref = zeros(3,n_pts);
omega_dot_ref = zeros(3,n_pts);
T_ref = zeros(1,n_pts);
Tau_ref = zeros(3,n_pts);

roll_ref = zeros(1,n_pts);
pitch_ref = zeros(1,n_pts);
yaw_ref = zeros(1,n_pts);
q_ref = zeros(4,n_pts);

q_old = dcm2quat(eye(3));
for i_pt = 1:n_pts
    [tmp_q, tmp_omega, tmp_omega_dot, tmp_T, tmp_Tau] = ...
    get_flat_states(...
        sol.v(:,i_pt), sol.a(:,i_pt), sol.jerk(:,i_pt), sol.snap(:,i_pt),...
        sol.psi(i_pt), sol.psi_dot(i_pt), sol.psi_ddot(i_pt),...
        zeros(3,1),...
        g, D, A, B, k_h, J, ...
        q_old, false);

    R_ref(1:3,1:3,i_pt) = quat2dcm(tmp_q);    
    omega_ref(:,i_pt) = tmp_omega(:);
    omega_dot_ref(:,i_pt) = tmp_omega_dot(:);
    T_ref(i_pt) = tmp_T;
    Tau_ref(:,i_pt) = tmp_Tau(:);   

    % tmp_q = dcm2quat(tmp_R);
    % tmp_q = lib.macro.FLU2FRD_quat(tmp_q(:));
    q_ref(:,i_pt) = tmp_q(:);
    [tmp] = quat2eul(tmp_q(:).');
    tmp_roll = tmp(3);
    tmp_pitch = tmp(2);
    tmp_yaw = tmp(1);
    roll_ref(i_pt) = tmp_roll;
    pitch_ref(i_pt) = tmp_pitch;
    yaw_ref(i_pt) = tmp_yaw;
    q_old = tmp_q;
end
res = sol;
res.R = R_ref;
res.omega = omega_ref;
res.omega_dot = omega_dot_ref;
res.Thrust = T_ref;
res.Torque = Tau_ref;
res.roll = roll_ref;
res.pitch = pitch_ref;
res.yaw = yaw_ref;
res.q = q_ref;
end