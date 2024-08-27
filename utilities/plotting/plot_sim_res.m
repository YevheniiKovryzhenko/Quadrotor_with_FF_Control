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

function [save_dir, figs, plt_settings] = plot_sim_res(res, pr_color, col_tr, save_dir, save_all, out, run_animation, obstcl, Parameters, out_other)
% pos_cell = res.states_cell;
wpts = res.wpts;

fnt_s               = 35; %font size small small
fnt                 = 40;
line_width_1        = 7; %width for all and for references
en_z_lim_offset     = true;
en_xy_lim_offset    = true;
z_lim_offset        = 0.1; %m
xy_lim_offset       = 0.1; %m
plot_2D             = false;
marker_size_1       = 10;
plot_extra_2D       = true;
grid_on_fl          = true;
box_on_fl           = true;
plot_errors_fl      = false;
plot_n_drones_fl    = false;
nticks_y            = 3;
nticks_x            = 5;

% set(0, 'DefaultAxesFontSize', fnt_s, 'DefaultAxesFontWeight','bold')
% set(0, 'DefaultTextFontSize', fnt_s, 'DefaultTextFontWeight','bold')
set(0, 'DefaultAxesFontSize', fnt_s)
set(0, 'DefaultTextFontSize', fnt_s)

def_colors = colororder;

pos                 = [50 50 1200 1200];
leg_str_pos     = {"$$x$$","$$y$$","$$z$$", "$$-z$$"};
leg_str_pos_units = {" ($$m$$)"," ($$m$$)"," ($$m$$)", " ($$m$$)"};

common_args_marker_1 = {'o','MarkerSize', marker_size_1,'MarkerEdgeColor','black','MarkerFaceColor','red'};

common_args_line_ref = {':', 'Color', col_tr{1},'LineWidth',line_width_1};
common_args_line = {'-', 'Color', pr_color,'LineWidth',line_width_1};
common_args_text_1 = {'FontSize',fnt,'interpreter','latex'};

plot_omap_fl = isfield(res, "omap") && ~isempty(res.omap);
if plot_omap_fl
    omap = res.omap.omap3D;
    en_z_lim_offset = false;
    en_xy_lim_offset = false;
end

n_dim = size(wpts,1);
if n_dim > 3
    n_dim = 3;
end

plot_i = 0;

if n_dim > 1
    plot_i = plot_i +1;
    f = figure; f.Position = pos;
    figs{plot_i} = f;
    if plot_omap_fl
        hold on
        show(omap)
        title("")
        plot_extra_2D = true;
        tmp = gca();
        colormap(tmp,"white")
    end
    hold on
    if grid_on_fl
        grid on
    end
    if box_on_fl
        box on
    end
    tmp_x_ref = out.p_e_ref(:,1);
    tmp_y_ref = out.p_e_ref(:,2);
    tmp_x = out.p_e(:,1);
    tmp_y = out.p_e(:,2);
    % tmp_wpts_x = wpts(1,:);
    % tmp_wpts_y = wpts(2,:);
    plot_2D_fl = n_dim < 3 || plot_2D;

    if plot_n_drones_fl && ~plot_2D_fl
        scaling = 0.4;

        crop_range = [0.1, 0.2, 0.25, 0.39, 0.44, 0.55];
        n_org = length(out.t_s);
        ids = floor(crop_range*n_org)+1;
    
        tmp_p = [tmp_x(:).'; tmp_y(:).'; -out.p_e(:,3).'];
        tmp_q = out.q';
        lib.visual3D.create_n_spaced_objs(ids,tmp_p,tmp_q, ...
        scaling, figs{plot_i});

        axis(gca(figs{plot_i}),"equal")
    end

    if plot_2D
        plot(tmp_x_ref,tmp_y_ref,common_args_line_ref{:})
        plot(tmp_x,tmp_y, common_args_line{:})
        % plot(tmp_wpts_x,tmp_wpts_y,common_args_marker_1{:})
        plot_extra_2D = false;
    else
        tmp_z_ref = -out.p_e_ref(:,3);
        tmp_z = -out.p_e(:,3);
        % tmp_wpts_z = -(wpts(3,:));

        plot3(tmp_x_ref,tmp_y_ref,tmp_z_ref,common_args_line_ref{:})
        plot3(tmp_x,tmp_y,tmp_z,common_args_line{:})
        % plot3(tmp_wpts_x,tmp_wpts_y,tmp_wpts_z,common_args_marker_1{:})
        
        zlabel(leg_str_pos{4}+leg_str_pos_units{4},common_args_text_1{:})
        if en_z_lim_offset
            zlim([min([tmp_z;tmp_z_ref])-z_lim_offset,max([tmp_z;tmp_z_ref])+z_lim_offset]);
        else
            zlim([-inf,inf])
        end
        view(3)
    end
    xlabel(leg_str_pos{1}+leg_str_pos_units{1},common_args_text_1{:})
    ylabel(leg_str_pos{2}+leg_str_pos_units{2},common_args_text_1{:})
    if en_xy_lim_offset
        xlim([min([tmp_x;tmp_x_ref])-xy_lim_offset,max([tmp_x;tmp_x_ref])+xy_lim_offset]);
        ylim([min([tmp_y;tmp_y_ref])-xy_lim_offset,max([tmp_y;tmp_y_ref])+xy_lim_offset]);
    else
        xlim([-inf,inf])
        ylim([-inf,inf])
    end

    if plot_extra_2D
        f = figure; f.Position = pos;
        plot_i = plot_i + 1;
        figs{plot_i} = f;
        copyobj(gca(figs{1}),figs{plot_i});
        view(2)
    end


end






n_figs = 7;
if plot_errors_fl
    n_subplots_per_fig = [6,6,6,6,6,3,1];
else
    n_subplots_per_fig = [3,3,3,3,3,3,1];
end
plt_settings = lib.plot.init(n_figs,n_subplots_per_fig);
plt_settings.y_nticks = nticks_y;
plt_settings.x_nticks = nticks_x;

if plot_errors_fl
    plt_settings.data_header_cell_y = {...
        {...
        "x_err",...
        "y_err",...
        "z_err",...
        "x",...
        "y",...
        "z"...
        },...
        {...
        "vx_err",...
        "vy_err",...
        "vz_err",...
        "vx",...
        "vy",...
        "vz"...
        },...
        {...
        "ax_err",...
        "ay_err",...
        "az_err",...
        "ax",...
        "ay",...
        "az"...
        },...
        {...
        "phi_err",...
        "theta_err",...
        "psi_err",...
        "phi",...
        "theta",...
        "psi"...
        },...
        {...
        "p_err",...
        "q_err",...
        "r_err",...
        "p",...
        "q",...
        "r"...
        },...
        {...
        "wind_v",...
        "wind_F",...
        "wind_omega"...
        },...
        {...
        "motor"
        }...
        };
else
    plt_settings.data_header_cell_y = {...
        {...
        "x",...
        "y",...
        "z"...
        },...
        {...
        "vx",...
        "vy",...
        "vz"...
        },...
        {...
        "ax",...
        "ay",...
        "az"...
        },...
        {...
        "phi",...
        "theta",...
        "psi"...
        },...
        {...
        "p",...
        "q",...
        "r"...
        },...
        {...
        "wind_v",...
        "wind_F",...
        "wind_omega"...
        },...
        {...
        "motor"
        }...
        };
end

for i = 1:n_figs
    plt_settings.data_header_cell_x{i}(:) = {"t"};
    if i < 6
        for ii = 1:3
            if plot_errors_fl
                plt_settings.extra_args_plot{i}{ii}{3} = col_tr{1};            
                plt_settings.extra_args_plot{i}{ii+3}{3} = col_tr{2};
            else
                plt_settings.extra_args_plot{i}{ii}{3} = pr_color;
            end
        end
    elseif i < 7
        % if plot_errors_fl
            for ii = 1:3
                plt_settings.extra_args_plot{i}{ii}{3} = col_tr{1};
            end
        % else
        %     for ii = 1:3
        %         plt_settings.extra_args_plot{i}{ii}{3} = col_tr{1};
        %     end
        % end
    end
end

if plot_errors_fl
    plt_settings.y_var_str            = {...
        {...
        "$${x_{\mathrm{ref}} - x}$$",...
        "$${y_{\mathrm{ref}} - y}$$",...
        "$${z_{\mathrm{ref}} - z}$$",...
        "$${x}$$",...
        "$${y}$$",...
        "$${z}$$"...
        },...
        {...
        "$${v_{x_{\mathrm{ref}}} - v_x}$$",...
        "$${v_{y_{\mathrm{ref}}} - v_y}$$",...
        "$${v_{z_{\mathrm{ref}}} - v_z}$$",...
        "$${v_x}$$",...
        "$${v_y}$$",...
        "$${v_z}$$"...
        },...
        {...
        "$${a_{x_{\mathrm{ref}}} - a_x}$$",...
        "$${a_{y_{\mathrm{ref}}} - a_y}$$",...
        "$${a_{z_{\mathrm{ref}}} - a_z}$$",...
        "$${a_x}$$",...
        "$${a_y}$$",...
        "$${a_z}$$"...
        },...
        {...
        "$${\phi_{\mathrm{ref}} - \phi}$$",...
        "$${\theta_{\mathrm{ref}} - \theta}$$",...
        "$${\psi_{\mathrm{ref}} - \psi}$$",...
        "$${\phi}$$",...
        "$${\theta}$$",...
        "$${\psi}$$"...
        },...
        {...
        "$${p_{\mathrm{ref}} - p}$$",...
        "$${q_{\mathrm{ref}} - q}$$",...
        "$${r_{\mathrm{ref}} - r}$$",...
        "$${p}$$",...
        "$${q}$$",...
        "$${r}$$"...
        },...
        {...
        "$$\mathbf{V_{\mathrm{wind}}}$$",...
        "$$\mathbf{F_{\mathrm{aero}}}$$",...
        "$$\mathbf{\omega_{\mathrm{wind}}}$$"...
        },...
        {...
        "Motor Throttle"...
        }...
        };
else
    plt_settings.y_var_str            = {...
        {...
        "$${x}$$",...
        "$${y}$$",...
        "$${z}$$"...
        },...
        {...
        "$${v_x}$$",...
        "$${v_y}$$",...
        "$${v_z}$$"...
        },...
        {...
        "$${a_x}$$",...
        "$${a_y}$$",...
        "$${a_z}$$"...
        },...
        {...
        "$${\phi}$$",...
        "$${\theta}$$",...
        "$${\psi}$$"...
        },...
        {...
        "$${p}$$",...
        "$${q}$$",...
        "$${r}$$"...
        },...
        {...
        "$$\mathbf{V_{\mathrm{wind}}}$$",...
        "$$\mathbf{F_{\mathrm{aero}}}$$",...
        "$$\mathbf{\omega_{\mathrm{wind}}}$$"...
        },...
        {...
        "Motor Throttle"...
        }...
        };
end

plt_settings.units_str_y{1}(:) = {lib.macro.unit_wrap("m")};
plt_settings.units_str_y{2}(:) = {lib.macro.unit_wrap("m/s")};
plt_settings.units_str_y{3}(:) = {lib.macro.unit_wrap("m/s^2")};
plt_settings.units_str_y{4}(:) = {lib.macro.unit_wrap("^\circ")};
plt_settings.units_str_y{5}(:) = {lib.macro.unit_wrap("^\circ/s")};
plt_settings.conv_factor_y{4}(:) = {180/pi};
plt_settings.conv_factor_y{5}(:) = {180/pi};

plt_settings.units_str_y{6}(1) = {lib.macro.unit_wrap("m/s")};
plt_settings.units_str_y{6}(3) = {lib.macro.unit_wrap("^\circ/s")};
plt_settings.units_str_y{6}(2) = {lib.macro.unit_wrap("N")};
plt_settings.conv_factor_y{6}(3) = {180/pi};

tmp_data = get_control_data_from_sim(out);


%% wind
tmp_data.wind_v = out.wind_vel(:,1);
tmp_data.wind_omega = out.wind_w(:,1);
tmp_data.wind_F = out.wind_F(:,1);

%%motors
tmp_data.motor = out.motors(:,1);
plt_settings.extra_args_plot{7}{1}{3} = def_colors(1,:);

plt_settings = lib.plot.start(plt_settings,tmp_data);
for i = 1:n_figs
    if i < 6
        for ii = 1:3
            if plot_errors_fl
                plt_settings.skip_data{i}{ii} = true;
                plt_settings.extra_args_plot{i}{ii+3}{1} = ':';
                plt_settings.extra_args_plot{i}{ii+3}{3} = col_tr{3};
            else
                plt_settings.extra_args_plot{i}{ii}{1} = ':';
                plt_settings.extra_args_plot{i}{ii}{3} = col_tr{1};
            end
        end
        if plot_errors_fl
            plt_settings.leg{i}{6} = [{"State", "Ref"},plt_settings.leg{1}{3}(:)'];
            plt_settings.add_leg{i}{6} = true;
        else
            plt_settings.leg{i}{3} = [{"State", "Ref"},plt_settings.leg{1}{2}(:)'];
            plt_settings.add_leg{i}{3} = true;
        end
    elseif i < 7
        % if plot_errors_fl
            for ii = 1:3
                plt_settings.extra_args_plot{i}{ii}{3} = col_tr{2};
            end
        % end
    end
end

tmp_data.x = tmp_data.x_ref;
tmp_data.y = tmp_data.y_ref;
tmp_data.z = tmp_data.z_ref;

tmp_data.vx = tmp_data.vx_ref;
tmp_data.vy = tmp_data.vy_ref;
tmp_data.vz = tmp_data.vz_ref;

tmp_data.ax = tmp_data.ax_ref;
tmp_data.ay = tmp_data.ay_ref;
tmp_data.az = tmp_data.az_ref;

tmp_data.phi = tmp_data.phi_ref;
tmp_data.theta = tmp_data.theta_ref;
tmp_data.psi = tmp_data.psi_ref;

tmp_data.p = tmp_data.p_ref;
tmp_data.q = tmp_data.q_ref;
tmp_data.r = tmp_data.r_ref;

tmp_data.wind_v = out.wind_vel(:,2);
tmp_data.wind_omega = out.wind_w(:,2);
tmp_data.wind_F = out.wind_F(:,2);

tmp_data.motor = out.motors(:,2);
plt_settings.extra_args_plot{7}{1}{3} = def_colors(2,:);

lib.plot.add(plt_settings, tmp_data);


for i = 1:n_figs
    if i < 6
        if plot_errors_fl
            for ii = 1:6
                plt_settings.skip_data{i}{ii} = true;
                plt_settings.add_leg{i}{ii} = false;
            end
        else
            for ii = 1:3
                plt_settings.skip_data{i}{ii} = true;
                plt_settings.add_leg{i}{ii} = false;
            end
        end
    elseif i < 7
        % if plot_errors_fl
            for ii = 1:3
                plt_settings.extra_args_plot{i}{ii}{3} = col_tr{3};            
            end
            plt_settings.leg{i}{3} = [{"$${x}$$", "$${y}$$", "$${z}$$"},plt_settings.leg{1}{2}(:)'];
            plt_settings.add_leg{i}{3} = true;
        % end
    end
end

tmp_data.wind_v = out.wind_vel(:,3);
tmp_data.wind_omega = out.wind_w(:,3);
tmp_data.wind_F = out.wind_F(:,3);

tmp_data.motor = out.motors(:,3);
plt_settings.extra_args_plot{7}{1}{3} = def_colors(3,:);

lib.plot.add(plt_settings, tmp_data);

for i = 1:3
    plt_settings.skip_data{6}{i} = true;
end
plt_settings.add_leg{6}{3} = false;
plt_settings.add_leg{7}{1} = true;
plt_settings.leg{7}{1} = [num2cell(arrayfun(@(x)sprintf("$${m}_{%i}$$",x),1:4)),plt_settings.leg{7}{1}(:)'];

tmp_data.motor = out.motors(:,4);
plt_settings.extra_args_plot{7}{1}{3} = def_colors(4,:);
lib.plot.add(plt_settings, tmp_data);

if (save_all)
    save_dir = lib.save_all_figs(save_dir,out);
end

if run_animation
    f = figure; f.Position = pos;
    plot_i = plot_i + 1;
    figs{plot_i} = f;
    copyobj(gca(figs{1}),figs{plot_i});

    update_obs = ~isempty(obstcl) && ~isempty(Parameters);
    if class(out) == "Simulink.SimulationOutput"
        update_traj = any(out.who == "traj");
    elseif class(out) == "struct"
        update_traj = isfield(out,'traj');
    else
        update_traj = false;
    end
    
    if update_obs
        axObjs__ = figs{plot_i}.Children;
        dataObjs__ = axObjs__.Children;
        wpts_obj = plot3(gca(figs{plot_i}), wpts(1,:),wpts(2,:),-wpts(3,:),common_args_marker_1{:});

        traj.obstacle = obstcl;
        traj.state_object = dataObjs__(1);
        if update_traj
            traj.reference_object = dataObjs__(2);        
            traj.wpts_object = wpts_obj;
        end

        % pause_dt = 0.01;
        % pause_dt = 0.05;
        % pause_dt = 0.1;
        write_gif = save_all;
        save_frames = false;
        map_scale = norm([max(wpts(1,:)) - min(wpts(1,:))
            max(wpts(2,:)) - min(wpts(2,:))
            max(wpts(3,:)) - min(wpts(3,:))]);
        scaling = max(min(map_scale/6,30), 0.3);

        desired_sampling_hz = 30;
        current_sapling_hz = 1/mean(diff(out.t_s));
        if current_sapling_hz < desired_sampling_hz %can't do much about this
            sampling = 1;
        else %we want to discard data we don't need
            % n_pts = length(out.t_s);
            sampling = max(round(current_sapling_hz / desired_sampling_hz), 1);
        end
        pause_dt = mean(diff(out.t_s(1:sampling:end)));
        
    
        tmp_p = [out.p_e(1:sampling:end,1).'; out.p_e(1:sampling:end,2).'; out.p_e(1:sampling:end,3).'];
        tmp_q = out.q(1:sampling:end,:)';
        
        if update_traj
            traj.time_since_start = out.traj.time_since_start.Data(1:sampling:end)';
            traj.T = out.traj.T.Data(1:sampling:end)';
            traj.pp = out.traj.pp.Data(1:sampling:end)';
            traj.n_dim = out.traj.n_dim.Data(1:sampling:end)';
            traj.n_dim_src = out.traj.n_dim_src.Data(1:sampling:end)';
            traj.n_dim_ids = out.traj.n_dim_ids.Data(1:sampling:end)';
            traj.N_segments = out.traj.N_segments.Data(1:sampling:end)';
            traj.state_0 = out.traj.state_0.Data(1:sampling:end,:)';
            traj.wpts = out.traj.wpts.Data(1:sampling:end)';
        end

    
        animation_3D = lib.visual3D.create_quadcopter_NED(figs{plot_i}, tmp_p(:,1), scaling);
        axis(gca(figs{plot_i}),'equal') 

        
        
        lib.visual3D.move_3D_v2(out.t_s(1:sampling:end)',tmp_p, tmp_q, animation_3D, ...
            write_gif, save_frames, save_dir, figs{plot_i}, pause_dt, traj, Parameters)

        if update_traj
            plot_every_replanned_traj(out.t_s(1:sampling:end)', traj, pr_color, col_tr, save_dir, save_all, run_animation, obstcl);
        end
        
    else
        pause_dt = 0.01;
        % pause_dt = 0.1;
        write_gif = save_all;
        scaling = 0.5;
        sampling = 8;
        % sampling = 1;
    
        tmp_p = [tmp_x(1:sampling:end).'; tmp_y(1:sampling:end).'; tmp_z(1:sampling:end).'];
        tmp_q = out.q(1:sampling:end,:)';
    
        animation_3D = lib.visual3D.create_quadcopter_NED(figs{plot_i}, tmp_p(:,1), scaling);
        axis(gca(figs{plot_i}),'equal')    
        
        lib.visual3D.move_3D(out.t_s(1:sampling:end)',tmp_p,tmp_q, animation_3D, ...
            write_gif, save_dir, figs{plot_i}, pause_dt)
    end
    
    
end
end