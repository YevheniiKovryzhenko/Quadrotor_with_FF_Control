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

function save_dir = add_plot_sim_res(figs, plt_settings, res, pr_color, col_tr, save_dir, save_all, out, run_animation)
% pos_cell = res.states_cell;
wpts = res.wpts;
line_width_1        = 5; %width for all and for references
plot_2D_fl             = false;
plot_extra_2D       = true;
plot_errors_fl      = false;
plot_n_drones_fl    = false;

pos                 = [50 50 1200 1200];
common_args_line = {'-.', 'Color', pr_color,'LineWidth',line_width_1};

n_dim = size(wpts,1);
if n_dim > 3
    n_dim = 3;
end

plot_i = 0;

if n_dim > 1
    plot_i = plot_i +1;    
    tmp_x = out.p_e(:,1);
    tmp_y = out.p_e(:,2);

    plot_2D_fl = n_dim < 3 || plot_2D_fl;

    if plot_2D_fl
        plot(gca(figs{plot_i}),tmp_x,tmp_y, common_args_line{:})
        plot_extra_2D = false;
    else
        tmp_z = -out.p_e(:,3);
        plot3(gca(figs{plot_i}),tmp_x,tmp_y,tmp_z,common_args_line{:})
    end

    legend(gca(figs{plot_i}),"Ref", "LQR", "$$H_\infty$$",plt_settings.leg{1}{2}{:})
    if plot_n_drones_fl && ~plot_2D_fl
        scaling = 0.2;
        scaling = 0.5;

        crop_range = [0.1, 0.2, 0.25, 0.39, 0.44, 0.55];
        n_org = length(out.t_s);
        ids = floor(crop_range*n_org)+1;
    
        tmp_p = [tmp_x(:).'; tmp_y(:).'; tmp_z(:).'];
        tmp_q = out.q';
        lib.visual3D.create_n_spaced_objs(ids,tmp_p,tmp_q, ...
        scaling, figs{plot_i});

        axis(gca(figs{plot_i}),"equal")
    end
    
    if plot_extra_2D
        plot_i = plot_i + 1;
        clf(figs{plot_i})
        copyobj(gca(figs{1}),figs{plot_i});
        view(gca(figs{plot_i}),2)
    end
    

end

n_figs = 7;

for i = 1:n_figs
    if i < 6
        plt_settings.skip_data{i}(:) = {false};
        for ii = 1:3
            if plot_errors_fl
                plt_settings.extra_args_plot{i}{ii}{3} = col_tr{1};            
                plt_settings.extra_args_plot{i}{ii+3}{3} = col_tr{2};
                plt_settings.extra_args_plot{i}{ii+3}{1} = '-.';
            else
                plt_settings.extra_args_plot{i}{ii}{3} = pr_color;
                plt_settings.extra_args_plot{i}{ii}{1} = '-.';

                if ii < 3
                    % xlabel(plt_settings.fig{i}{ii},"");
                    plt_settings.units_str_x{i}{ii} = "";
                    plt_settings.x_var_str{i}{ii} = "";
                end
            end

            plt_settings.extra_args_plot{i}{ii}{5} = line_width_1;
        end
        if plot_errors_fl
            plt_settings.leg{i}{6} = [{"LQR", "Ref", "$$H_\infty$$"},plt_settings.leg{1}{3}(:)'];
            plt_settings.add_leg{i}{6} = true;
        else
            plt_settings.leg{i}{3} = [{"LQR", "Ref", "$$H_\infty$$"},plt_settings.leg{1}{2}(:)'];
            plt_settings.add_leg{i}{3} = true;
        end
    elseif i < 7
        plt_settings.skip_data{i}(:) = {true};
        for ii = 1:3
            plt_settings.extra_args_plot{i}{ii}{3} = col_tr{1};
        end
    else
        plt_settings.skip_data{7}(:) = {true};
    end
end

tmp_data = get_control_data_from_sim(out);


%% wind
% tmp_data.wind_v = out.wind_vel(:,1);
% tmp_data.wind_omega = out.wind_w(:,1);
% tmp_data.wind_F = out.wind_F(:,1);

%%motors
% tmp_data.motor = out.motors(:,1);
% plt_settings.extra_args_plot{7}{1}{3} = def_colors(1,:);

% for i = 1:5
%     plt_settings.skip_data{i}(:) = {false};
%     for ii = 1:3
%         if plot_errors_fl
%             plt_settings.extra_args_plot{i}{ii+3}{1} = '-';
%             plt_settings.extra_args_plot{i}{ii+3}{3} = col_tr{3};
%         else
%             plt_settings.extra_args_plot{i}{ii}{1} = '-';
%             plt_settings.extra_args_plot{i}{ii}{3} = col_tr{1};
%         end
%     end
% end
plt_settings.skip_data{6}(:) = {true};
plt_settings.skip_data{7}(:) = {true};

lib.plot.add(plt_settings,tmp_data);

if (save_all)
    save_dir = lib.save_all_figs(save_dir,out);
end

if run_animation
    f = figure; f.Position = pos;
    plot_i = plot_i + 1;
    figs{plot_i} = f;
    copyobj(gca(figs{1}),figs{plot_i});

    pause_dt = 0.01;
    write_gif = save_all;
    scaling = 0.5;
    sampling = 8;
    
    tmp_p = [tmp_x(1:sampling:end).'; tmp_y(1:sampling:end).'; tmp_z(1:sampling:end).'];
    tmp_q = out.q(1:sampling:end,:)';
    
    axis(gca(figs{plot_i}),"equal")
    lib.visual3D.move_3D(out.t_s(1:sampling:end)',tmp_p,tmp_q, ...
        scaling, write_gif, save_dir, figs{plot_i}, pause_dt)
end
end