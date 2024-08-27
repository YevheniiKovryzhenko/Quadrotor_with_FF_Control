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

function [p, v, a, jerk, snap, psi, psi_dot, psi_ddot, finished_traj] = eval_traj(...
    t,T,coefs,n_dim,n_dim_src, n_dim_ids, n_int, waypoints_offset, DU_input_factor, TU_input_factor)

%hardcoded for now
% n_dim = 4;
% FFS_fl = false;
finished_traj = false;
%%%
% nontriv_dim_ind = 1:4; %index of non-trivial (non-zero) dimension

v           = zeros(3,1);
p           = zeros(3,1);
a           = zeros(3,1);
jerk        = zeros(3,1);
snap        = zeros(3,1);
psi         = 0;
psi_dot     = 0;
psi_ddot    = 0;
if t > T(n_int+1)
    finished_traj = true;
    % return
end

% [q,qd,qdd,qddd,qdddd] = interpSnapTraj(coefs,T,t,FFS_fl,nontriv_dim_ind,n_dim);

q = TrajectoryOptimizer.common.eval_pp(t, 0, coefs, T, ...
    n_dim, n_dim_src, n_dim_ids, n_int, waypoints_offset, DU_input_factor, TU_input_factor);
qd = TrajectoryOptimizer.common.eval_pp(t, 1, coefs, T, ...
    n_dim, n_dim_src, n_dim_ids, n_int, waypoints_offset, DU_input_factor, TU_input_factor);
qdd = TrajectoryOptimizer.common.eval_pp(t, 2, coefs, T, ...
    n_dim, n_dim_src, n_dim_ids, n_int, waypoints_offset, DU_input_factor, TU_input_factor);
qddd = TrajectoryOptimizer.common.eval_pp(t, 3, coefs, T, ...
    n_dim, n_dim_src, n_dim_ids, n_int, waypoints_offset, DU_input_factor, TU_input_factor);
qdddd = TrajectoryOptimizer.common.eval_pp(t, 4, coefs, T, ...
    n_dim, n_dim_src, n_dim_ids, n_int, waypoints_offset, DU_input_factor, TU_input_factor);

p(1:3,1)    = q(1:3,1);
v(1:3,1)    = qd(1:3,1);
a(1:3,1)    = qdd(1:3,1);
jerk(1:3,1) = qddd(1:3,1);
snap(1:3,1) = qdddd(1:3,1);
psi(1)      = q(4);
psi_dot(1)  = qd(4);
psi_ddot(1) = qdd(4);

% if n_dim > 3
%     p(1:3,1)    = q(1:3,1);
%     v(1:3,1)    = qd(1:3,1);
%     a(1:3,1)    = qdd(1:3,1);
%     jerk(1:3,1) = qddd(1:3,1);
%     snap(1:3,1) = qdddd(1:3,1);
%     psi(1)      = q(4);
%     psi_dot(1)  = qd(4);
%     psi_ddot(1) = qdd(4);
% else
%     p(1:n_dim,1)    = q(1:n_dim,1);
%     v(1:n_dim,1)    = qd(1:n_dim,1);
%     a(1:n_dim,1)    = qdd(1:n_dim,1);
%     jerk(1:n_dim,1)     = qddd(1:n_dim,1);
%     snap(1:n_dim,1)     = qdddd(1:n_dim,1);
% end
end