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

%select trajectory and solve
traj_opt = trajectory_ID.DEFAULT;
% traj_opt = trajectory_ID.line_x_1;
% traj_opt = trajectory_ID.line_y_1; 
% traj_opt = trajectory_ID.line_xy_1;
% traj_opt = trajectory_ID.line_xy_2;
% traj_opt = trajectory_ID.line_xyz_1;
% traj_opt = trajectory_ID.line_xyz_2;
% traj_opt = trajectory_ID.yaw_1;
% traj_opt = trajectory_ID.fig_8_1;
% traj_opt = trajectory_ID.fig_8_1_slow;
% traj_opt = trajectory_ID.fig_8_2_slow;
% traj_opt = trajectory_ID.fig_8_yaw_1_slow;
% traj_opt = trajectory_ID.fig_8_yaw_2_slow;
% traj_opt = trajectory_ID.fig_8_yaw_1;
% traj_opt = trajectory_ID.fig_8_yaw_2;
% traj_opt = trajectory_ID.rrt_test_1;
% traj_opt = trajectory_ID.square_1;
% traj_opt = trajectory_ID.square_1_slow;
traj_opt = trajectory_ID.square_2;
% traj_opt = trajectory_ID.circle_1;
% traj_opt = trajectory_ID.spiral_1;
% traj_opt = trajectory_ID.spiral_yaw_1;
% traj_opt = trajectory_ID.non_rest2rest;
% traj_opt = trajectory_ID.collision_avoidance_loiter_1;
% traj_opt = trajectory_ID.line_y_obst_1;

traj = trajectory_manager(traj_opt);
[~, obstacles] = trajectory_manager.get_traj(traj_opt);
res = traj.result;

switch traj.ID
    case trajectory_ID.rrt_test_1
        res.omap = load("1_Lab_true_3_blocks.mat"); 
end

% res = get_solve_traj(traj_opt);

%get all the extra states from the solution
res = reconstruct_all_flat_states(res, Parameters.g, Parameters.Vehicle.D, Parameters.Vehicle.A, Parameters.Vehicle.B, Parameters.Vehicle.k_h, Parameters.Vehicle.I);

p_0(:) = res.p(:,1);
v_0(:) = res.v(:,1);
a_0(:) = res.a(:,1);
q_0(:) = res.q(:,1);
att_0(:) = [res.roll(1); res.pitch(1); res.yaw(1)];
omega_0(:) = res.omega(:,1);

save_traj2file(res,save_dir, traj_opt);

function save_traj2file(res,save_dir, traj_opt)
    lib.file.check_create_folder(save_dir);
    subfolder = fullfile(save_dir,"trajectories/");
    lib.file.check_create_folder(subfolder);
    file_name = fullfile(subfolder,sprintf("%s.traj",traj_opt));
    
    traj = trajectory_writer(res.minsnap_set);
    traj.write(file_name);
end