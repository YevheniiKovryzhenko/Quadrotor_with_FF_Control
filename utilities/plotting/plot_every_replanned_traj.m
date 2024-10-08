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

function [n_replanning_events, is_replanning_event] = plot_every_replanned_traj(t_s, traj, pr_color, col_tr, save_dir, save_all, run_animation, obstacles)
n = length(traj.T);
[n_replanning_events, is_replanning_event] = find_replanning_events(traj);
if (n < 2)
    return;
end

replan_ind = find(is_replanning_event);


for ind = 1:n_replanning_events
    i = replan_ind(ind);
    n_replanning_events = 1 + n_replanning_events;
    is_replanning_event(i) = true;

    ref_T = traj.T{i};
    eval_time_ahead = linspace(0,ref_T(end),n-i);
    TU_input_factor = 10 / ref_T(end);
    p_ref = TrajectoryOptimizer.common.eval_pp(...
        eval_time_ahead,...
        0,...
        traj.pp{i}, ...
        ref_T, ...
        traj.n_dim(i),...
        traj.n_dim_src(i), ...
        traj.n_dim_ids{i}, ...
        traj.N_segments(i), ...
        traj.state_0(:,i), ...
        1, ...
        TU_input_factor);
    
    v_ref = TrajectoryOptimizer.common.eval_pp(...
        eval_time_ahead,...
        1,...
        traj.pp{i}, ...
        ref_T, ...
        traj.n_dim(i),...
        traj.n_dim_src(i), ...
        traj.n_dim_ids{i}, ...
        traj.N_segments(i), ...
        traj.state_0(:,i), ...
        1, ...
        TU_input_factor);
    
    a_ref = TrajectoryOptimizer.common.eval_pp(...
        eval_time_ahead,...
        2,...
        traj.pp{i}, ...
        ref_T, ...
        traj.n_dim(i),...
        traj.n_dim_src(i), ...
        traj.n_dim_ids{i}, ...
        traj.N_segments(i), ...
        traj.state_0(:,i), ...
        1, ...
        TU_input_factor);
    
    j_ref = TrajectoryOptimizer.common.eval_pp(...
        eval_time_ahead,...
        3,...
        traj.pp{i}, ...
        ref_T, ...
        traj.n_dim(i),...
        traj.n_dim_src(i), ...
        traj.n_dim_ids{i}, ...
        traj.N_segments(i), ...
        traj.state_0(:,i), ...
        1, ...
        TU_input_factor);
    
    s_ref = TrajectoryOptimizer.common.eval_pp(...
        eval_time_ahead,...
        4,...
        traj.pp{i}, ...
        ref_T, ...
        traj.n_dim(i),...
        traj.n_dim_src(i), ...
        traj.n_dim_ids{i}, ...
        traj.N_segments(i), ...
        traj.state_0(:,i), ...
        1, ...
        TU_input_factor);
    
    pos_cell = {p_ref, v_ref, a_ref, j_ref, s_ref};

    res.states_cell = pos_cell;
    res.minsnap_set.T = ref_T;
    res.t = eval_time_ahead;
    res.wpts = traj.wpts{i};
    figs = plot_start_min_snap_poly(res, pr_color, col_tr, save_dir, save_all, run_animation, obstacles);
    
    if i == 1 %show future replanning events on the first figure
        first_fig = figs{1};
    else
        fprintf("Replanning event %i: time = %4.2f\n", ind-1, t_s(i))
        hold(gca(first_fig),"on")
        plot3(gca(first_fig),p_ref(1,1), p_ref(2,1), -p_ref(3,1), 'o','MarkerSize', 15,'MarkerEdgeColor','green', 'LineWidth', 3)
        plot3(gca(first_fig),p_ref(1,1), p_ref(2,1), -p_ref(3,1), 'X','MarkerSize', 10,'MarkerEdgeColor','green', 'LineWidth', 2)
        hold(gca(first_fig),"off")
    end        
end
end