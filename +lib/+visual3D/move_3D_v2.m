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

function move_3D_v2(t,p,q, animation_3D, ...
    write_gif, write_frames, FolderName, fig, pause_dt, traj, Parameters)

n = length(t);
update_obs = ~isempty(traj) && ~isempty(Parameters) && isfield(traj,'obstacle');
update_traj = ~isempty(traj) && isfield(traj,'reference_object');
update_wpts = ~isempty(traj) && isfield(traj,'wpts_object') && isfield(traj,'wpts');

if update_obs
    obstcl = traj.obstacle;
end
state_obj = traj.state_object;
if update_traj
    ref_obj = traj.reference_object;
end
if update_wpts
    wpts = traj.wpts;
    wpts_obj = traj.wpts_object;
end

ax = gca(fig);
% axis(ax,'equal')
view_elevation_deg = 60;
view(ax,-45,view_elevation_deg)



if write_gif
    time           = clock;
    FigNames_gif   = sprintf('%s/%d_%d_%d_%d_%d_%d.gif',FolderName,time(1),time(2),time(3),time(4),time(5),round(time(6)));
    if ~exist(FolderName, 'dir')
       mkdir(FolderName)
    end
    dt_gif = max(0.02, pause_dt);
    % dt_gif = 0.1;
    
    lib.gif.start(fig,FigNames_gif,dt_gif)

end

if write_frames
    frames_dir   = fullfile(FolderName, 'Frames/');
    lib.save_fig('1', frames_dir, fig);
end

% obs = obstacle(1,[0, -1, -1],[0.5, 0.5, 3]);
if update_obs
    n_obstcls = length(obstcl);
    obs_plot = cell(1,n_obstcls);
    for i = 1:n_obstcls
        if ~obstcl{i}.ignore_fl
            obs_plot{i} = obstcl{i}.draw_flip_z(ax);
            obstcl{i}.hide_obj_from_plot(obs_plot{i});
        end
    end
end

if update_obs
    detection_range = detection(1,p(:,1),Parameters.Vehicle.Obstacle_Avoidance.Detection_Range);
    detection_range_plot = detection_range.draw(ax);
end

for i = 2:n
    animation_3D = lib.visual3D.update(animation_3D,q(:,i),p(:,i).*[1;1;-1]);
    state_obj.XData = p(1,1:i);
    state_obj.YData = p(2,1:i);
    state_obj.ZData = -p(3,1:i);

    if (update_traj)
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
        
        ref_obj.XData = p_ref(1,:);
        ref_obj.YData = p_ref(2,:);
        ref_obj.ZData = -p_ref(3,:);
    end
    if update_wpts
        wpts_obj.XData = wpts{i}(1,:);
        wpts_obj.YData = wpts{i}(2,:);
        wpts_obj.ZData = -wpts{i}(3,:);
    end    
    if update_obs
        detection_range.center(:) = p(:,i);
        detection_range.draw_move_flip_z(detection_range_plot);
        for ii = 1:n_obstcls
            if ~obstcl{ii}.ignore_fl
                if obstcl{ii}.check_if_obs_is_interior(p(1:3,i), Parameters.Vehicle.Obstacle_Avoidance.Detection_Range)        
                    obstcl{ii}.show_obj_on_plot(obs_plot{ii}, 0.3, 0.35);
                else
                    obstcl{ii}.show_obj_on_plot(obs_plot{ii}, 0.05, 0.06);
                end
            end
        end
    end
    
    
    
    view(ax,-45 + (90)*(i/n),view_elevation_deg + (15)* (i/n));
    drawnow

    if write_gif        
        lib.gif.append(fig,FigNames_gif,dt_gif);
    end
    if write_frames
        lib.save_fig(sprintf('%i',i), frames_dir, fig);
    end
    if pause_dt > 0 && ~write_gif
        pause(pause_dt)
    end
end
end