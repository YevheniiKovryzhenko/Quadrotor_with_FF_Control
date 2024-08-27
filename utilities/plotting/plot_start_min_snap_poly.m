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

function [figs, plt_settings, save_dir] = plot_start_min_snap_poly(res,pr_color,col_tr,save_dir,save_all,run_animation, obstacles)
pos_cell = res.states_cell;
timepoints = res.minsnap_set.T;
tsamples = res.t;
wpts = res.wpts;

fnt_s               = 23; %font size small small
fnt                 = 30;
line_width_1        = 2; %width for all and for references
en_extra_leg        = 0; %enable extra legends for the same figure  
en_z_lim_offset     = true;
en_xy_lim_offset    = true;
z_lim_offset        = 0.1; %m
xy_lim_offset       = 0.1; %m
plot_2D             = false;
marker_size_1       = 10;
marker_size_2       = 6;
plot_extra_2D       = true;
grid_on_fl          = true;

set(0, 'DefaultAxesFontSize', fnt_s, 'DefaultAxesFontWeight','bold')
set(0, 'DefaultTextFontSize', fnt_s, 'DefaultTextFontWeight','bold')

n_dt            = numel(pos_cell);
pos                 = [50 50 1200 1200];
leg_str_pos     = {"$$\bf{X}$$","$$\bf{Y}$$","$$\bf{Z}$$", "$$\bf{Alt}$$"};
leg_str_pos_units = {" ($$\bf{m}$$)"," ($$\bf{m}$$)"," ($$\bf{m}$$)", " ($$\bf{m}$$)"};
leg_str_att     = {"$$\bf{\psi}$$"};
leg_str_der_att     = {" ($$\bf{^\circ}$$)"," \textbf{Rate} ($$\bf{^\circ/s}$$)"," \textbf{Accel.} ($$\bf{^\circ/s^2}$$)"," \textbf{Jerk} ($$\bf{^\circ/s^3}$$)"," \textbf{Snap} ($$\bf{^\circ/s^3}$$)"};
leg_str_der_pos     = {"\textbf{Pos.} ($$\bf{m}$$)","\textbf{Vel.} ($$\bf{m/s}$$)","\textbf{Accel.} ($$\bf{m/s^2}$$)","\textbf{Jerk} ($$\bf{m/s^3}$$)","\textbf{Snap} ($$\bf{m/s^4}$$)"};

common_args_marker_1 = {'o','MarkerSize', marker_size_1,'MarkerEdgeColor','black','MarkerFaceColor','red'};
common_args_marker_2 = {'o','MarkerSize', marker_size_2,'MarkerEdgeColor','black'};

common_args_line_1 = {'LineWidth',line_width_1};
common_args_line_2 = {'Color',pr_color,common_args_line_1{:}};
common_args_text_1 = {'FontSize',fnt,'fontweight','bold','interpreter','latex'};

time_str = "\textbf{Time} $$(s)$$";

plot_omap_fl = isfield(res, "omap") && ~isempty(res.omap);
if plot_omap_fl
    omap = res.omap.omap3D;
    en_z_lim_offset = false;
    en_xy_lim_offset = false;
end

n_dim = size(wpts,1);
plot_att = false;
if n_dim > 3
    n_dim_att = n_dim - 3;
    n_dim = 3;
    plot_att = true;    
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
    tmp_x = pos_cell{1}(1,:);
    tmp_y = pos_cell{1}(2,:);

    if n_dim < 3 || plot_2D
        plot(tmp_x,tmp_y,'-',common_args_line_2{:})
        plot(wpts(1,:),wpts(2,:),common_args_marker_1{:})
        plot_extra_2D = false;
    else
        tmp_z = -pos_cell{1}(3,:);
        plot3(tmp_x,tmp_y,tmp_z,'-',common_args_line_2{:})
        plot3(wpts(1,:),wpts(2,:),-wpts(3,:),common_args_marker_1{:})
        
        zlabel(leg_str_pos{4}+leg_str_pos_units{4},common_args_text_1{:})
        if en_z_lim_offset
            zlim([min(tmp_z)-z_lim_offset,max(tmp_z)+z_lim_offset]);
        else
            zlim([-inf,inf])
        end
        view(3)

        n_obst = length(obstacles);
        for i = 1:n_obst
            obstacles{i}.draw_flip_z(gca);
        end
    end
    xlabel(leg_str_pos{1}+leg_str_pos_units{1},common_args_text_1{:})
    ylabel(leg_str_pos{2}+leg_str_pos_units{2},common_args_text_1{:})
    if en_xy_lim_offset
        xlim([min(tmp_x)-xy_lim_offset,max(tmp_x)+xy_lim_offset]);
        ylim([min(tmp_y)-xy_lim_offset,max(tmp_y)+xy_lim_offset]);
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




f = figure; f.Position = pos;
plot_i = plot_i + 1;
figs{plot_i} = f;
hold on

m = 3;
n = 2;
i = 0;

i = i+1;
plot_i = plot_i + 1;
figs{plot_i} = subplot(m,n,i);
hold on
if grid_on_fl
    grid on
end
tmp_p = cell(1,n_dim);
for i_dim = 1:n_dim
    tmp_p{i_dim} = plot(tsamples,pos_cell{1}(i_dim,:),'-','Color',col_tr{i_dim},common_args_line_1{:});
    plot(timepoints,wpts(i_dim,:),common_args_marker_2{:}, 'MarkerFaceColor', col_tr{i_dim})
end
xlabel(time_str,common_args_text_1{:})
ylabel(leg_str_der_pos{1},common_args_text_1{:})
if en_extra_leg && n_dim > 1
    legend([tmp_p{1:n_dim}],leg_str_pos{1:n_dim},common_args_text_1{:})
    legend('AutoUpdate','off')
end
xlim([-inf,inf])
ylim([-inf,inf])
% set(gca,'FontSize',fnt_s)
hold off

for i_dt = i+1:n_dt
    plot_i = plot_i + 1;
    figs{plot_i} = subplot(m,n,i_dt);
    hold on
    if grid_on_fl
        grid on
    end
    tmp_p = cell(1,n_dim);
    for i_dim = 1:n_dim
        tmp_p{i_dim} = plot(tsamples,pos_cell{i_dt}(i_dim,:),'-','Color',col_tr{i_dim},common_args_line_1{:});
    end
    xlabel(time_str,common_args_text_1{:})
    ylabel(leg_str_der_pos{i_dt},common_args_text_1{:})
    if n_dim > 1
        if en_extra_leg
            legend([tmp_p{1:n_dim}],leg_str_pos{1:n_dim},common_args_text_1{:})
            legend('AutoUpdate','off')
        else
            if isequal(i_dt,n_dt)
                legend([tmp_p{1:n_dim}],leg_str_pos{1:n_dim},common_args_text_1{:},'Position',[0.05, .15, 1, 0.15]);
            end
        end
    end
    
    xlim([-inf,inf])
    ylim([-inf,inf])
%     set(gca,'FontSize',fnt_s)
    hold off

end

plot_norm = true;

if plot_norm
    f = figure; f.Position = pos;
    plot_i = plot_i + 1;
    figs{plot_i} = f;
    hold on
    
    m = 2;
    n = 2;
    
    
    for i_dt = 2:n_dt
        plot_i = plot_i + 1;
        figs{plot_i} = subplot(m,n,i_dt-1);
        hold on
        if grid_on_fl
            grid on
        end
        
        plot(tsamples, vecnorm(pos_cell{i_dt}(:,:),1),'-','Color',col_tr{i_dim},common_args_line_1{:});
        
        xlabel(time_str,common_args_text_1{:})
        ylabel(leg_str_der_pos{i_dt},common_args_text_1{:})
        
        xlim([-inf,inf])
        ylim([-inf,inf])
        hold off
    
    end
end

if plot_att
    f = figure; f.Position = pos;
    plot_i = plot_i + 1;
    figs{plot_i} = f;
    hold on
    
    m = 3;
    n = 2;
    i = 0;
    
    i = i+1;
    plot_i = plot_i + 1;
    figs{plot_i} = subplot(m,n,i);
    hold on
    if grid_on_fl
        grid on
    end
    tmp_p = cell(1,n_dim_att);
    for i_dim = 1:n_dim_att
        tmp_p{i_dim} = plot(tsamples,pos_cell{1}(n_dim+i_dim,:)*180/pi,'-','Color',col_tr{i_dim},common_args_line_1{:});
        plot(timepoints,wpts(n_dim+i_dim,:)*180/pi,common_args_marker_2{:}, 'MarkerFaceColor', col_tr{i_dim})
    end
    xlabel(time_str,common_args_text_1{:})
    if isequal(n_dim_att,1)
        ylabel(leg_str_att{1} + leg_str_der_att{1},common_args_text_1{:})
    else
        ylabel(leg_str_der_att{1},common_args_text_1{1:n_dim_att})
        if en_extra_leg
            legend([tmp_p{1:n_dim_att}],leg_str_att{1:n_dim_att},common_args_text_1{:})
            legend('AutoUpdate','off')
        end
    end
    
    xlim([-inf,inf])
    ylim([-inf,inf])
%     set(gca,'FontSize',fnt_s)
    hold off
    
    for i_dt = i+1:n_dt
        plot_i = plot_i + 1;
        figs{plot_i} = subplot(m,n,i_dt);
        hold on
        if grid_on_fl
            grid on
        end
        tmp_p = cell(1,n_dim_att);
        for i_dim = 1:n_dim_att
            tmp_p{i_dim} = plot(tsamples,pos_cell{i_dt}(n_dim+i_dim,:)*180/pi,'-','Color',col_tr{i_dim},common_args_line_1{:});
        end
        xlabel(time_str,common_args_text_1{:})
        if isequal(n_dim_att,1)
            ylabel(leg_str_att{1} + leg_str_der_att{i_dt},common_args_text_1{:})
        else
            ylabel(leg_str_der_att{i_dt},common_args_text_1{1:n_dim_att})
            if en_extra_leg
                legend([tmp_p{1:n_dim_att}],leg_str_att{1:n_dim_att},common_args_text_1{:})
                legend('AutoUpdate','off')
            end
        end
        if en_extra_leg
            legend([tmp_p{1:n_dim_att}],leg_str_pos{1:n_dim},common_args_text_1{:})
            legend('AutoUpdate','off')
        else
            if isequal(i_dt,n_dt) && ~isequal(n_dim_att,1)
                legend([tmp_p{1:n_dim_att}],leg_str_att{1:n_dim_att},common_args_text_1{:},'Position',[0.05, .15, 1, 0.15]);
            end
        end
        
        xlim([-inf,inf])
        ylim([-inf,inf])
        hold off
    
    end
end

plot_reconstructed_states = isfield(res, 'roll') && isfield(res, 'pitch') && isfield(res, 'omega') && isfield(res, 'omega_dot') && isfield(res, 'Torque') && isfield(res, 'Thrust');
if plot_reconstructed_states
    plt_settings = lib.plot.init(1,6);
    plt_settings.data_header_cell_y = {...
        {...
        "roll",...
        "pitch",...
        "omega",...
        "omega_dot",...
        "Torque",...
        "Thrust"...
        }
        };
    
    plt_settings.data_header_cell_x{1}(:) = {"t"};
    
    plt_settings.y_var_str{1}            = {...
        "$$\mathbf{\phi}$$",...
        "$$\mathbf{\theta}$$",...
        "$$\mathbf{\omega}$$",...
        "$$\mathbf{\dot{\omega}}$$",...
        "$$\mathbf{\tau}$$",...
        "$$\mathbf{T}_{cmd}$$"};
    
    plt_settings.units_str_y{1}(1:2) = {lib.macro.unit_wrap("^\circ")};
    plt_settings.units_str_y{1}(3) = {lib.macro.unit_wrap("^\circ/s")};
    plt_settings.units_str_y{1}(4) = {lib.macro.unit_wrap("^\circ/s^2")};
    plt_settings.units_str_y{1}(5) = {lib.macro.unit_wrap("Nm")};
    plt_settings.units_str_y{1}(6) = {lib.macro.unit_wrap("m/s")};
    plt_settings.conv_factor_y{1}(1:4) = {180/pi};
    
    tmp_data.t = res.t;
    tmp_data.roll = res.roll;
    tmp_data.pitch = res.pitch;
    tmp_data.omega = res.omega(1,:);
    tmp_data.omega_dot = res.omega_dot(1,:);
    tmp_data.Torque = res.Torque(1,:);
    tmp_data.Thrust = res.Thrust;
    plt_settings.extra_args_plot{1}{1}{3} = col_tr{1};
    plt_settings.extra_args_plot{1}(:) = plt_settings.extra_args_plot{1}(1);
    plt_settings.extra_args_plot{1}{2}{3} = col_tr{2};
    plt_settings = lib.plot.start(plt_settings,tmp_data);
    
    
    plt_settings.skip_data{1}([1:2, 6]) = {true};
    plt_settings.extra_args_plot{1}{1}{3} = col_tr{2};
    plt_settings.extra_args_plot{1}(:) = plt_settings.extra_args_plot{1}(1);
    
    tmp_data.omega = res.omega(2,:);
    tmp_data.omega_dot = res.omega_dot(2,:);
    tmp_data.Torque = res.Torque(2,:);
    
    lib.plot.add(plt_settings, tmp_data);
    
    
    plt_settings.extra_args_plot{1}{1}{3} = col_tr{3};
    plt_settings.extra_args_plot{1}(:) = plt_settings.extra_args_plot{1}(1);
    plt_settings.leg{1}{3} = [{"$$\mathbf{\omega}_x$$", "$$\mathbf{\omega}_y$$", "$$\mathbf{\omega}_z$$"},plt_settings.leg{1}{3}(:)'];
    plt_settings.leg{1}{4} = [{"$$\mathbf{\dot{\omega}}_x$$", "$$\mathbf{\dot{\omega}}_y$$", "$$\mathbf{\dot{\omega}}_z$$"}, plt_settings.leg{1}{4}(:)'];
    plt_settings.leg{1}{5} = [{"$$\mathbf{\tau}_x$$", "$$\mathbf{\tau}_y$$", "$$\mathbf{\tau}_z$$"},plt_settings.leg{1}{5}(:)'];
    plt_settings.add_leg{1}(3) = {true};
    plt_settings.add_leg{1}(4) = {true};
    plt_settings.add_leg{1}(5) = {true};
    
    tmp_data.omega = res.omega(3,:);
    tmp_data.omega_dot = res.omega_dot(3,:);
    tmp_data.Torque = res.Torque(3,:);
    
    lib.plot.add(plt_settings, tmp_data);
end

if (save_all)
    save_dir = lib.save_all_figs(save_dir,res);
end

if run_animation && isfield(res, 'q')
    f = figure; f.Position = pos;
    plot_i = plot_i + 1;
    figs{plot_i} = f;
    copyobj(gca(figs{1}),figs{plot_i});
    
    pause_dt = 0.01;
    write_gif = save_all;
    scaling = 0.2;
    n_th_entry = 2;
    
    tmp_p = [res.p(1,1:n_th_entry:end); res.p(2,1:n_th_entry:end); -res.p(3,1:n_th_entry:end)];
    tmp_q = [res.q(1,1:n_th_entry:end); res.q(2,1:n_th_entry:end); res.q(3,1:n_th_entry:end); res.q(4,1:n_th_entry:end)];
    
    animation_3D = lib.visual3D.create_quadcopter_NED(figs{plot_i}, tmp_p(:,1), scaling);
    axis(gca(figs{plot_i}),'equal')
    
    lib.visual3D.move_3D(res.t(1:n_th_entry:end),tmp_p,tmp_q, animation_3D, ...
        write_gif, save_dir, figs{plot_i}, pause_dt)
end
end