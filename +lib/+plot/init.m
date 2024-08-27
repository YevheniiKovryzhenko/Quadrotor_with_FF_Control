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

function plt_settings = init(n_figs,n_plots_per_fig)
    plt_settings.y_var_str = cell(1,n_figs);
    for i = 1:n_figs
        plt_settings.y_var_str{i} = cell(1,n_plots_per_fig(i));
        plt_settings.y_var_str{i}(:) = {""};
    end
    plt_settings.data_header_cell_x = plt_settings.y_var_str;
    plt_settings.data_header_cell_y = plt_settings.y_var_str;

    plt_settings.fnt_s          = 35; %font size small small
    plt_settings.fnt            = 40; %font size small
    plt_settings.fnt_tit        = 45; %font size title
    plt_settings.line_width_1   = 7; %width for all
    plt_settings.marker_size    = 15;
    plt_settings.marker_size_s  = 2.5;
    plt_settings.box_on_fl      = true;
    plt_settings.grid_on_fl     = true;
    transpose_subfig_fl         = true;

    plt_settings.y_nticks       = [];
    plt_settings.x_nticks       = [];
    
    color_id = 1;
    %%%%
    plt_settings.def_colors = {...
        [0 0.4470 0.7410],...
        [0.8500 0.3250 0.0980],...
        [0.9290 0.6940 0.1250],...
        [0.4940 0.1840 0.5560],...
        [0.4660 0.6740 0.1880],...
        [0.3010 0.7450 0.9330],...
        [0.6350 0.0780 0.1840]};
    %%%%   
    
    plt_settings.n_figs = numel(plt_settings.y_var_str);
    plt_settings.fig = cell(1,plt_settings.n_figs);
    plt_settings.subfig_ratio = cell(1,plt_settings.n_figs);
    plt_settings.use_subfig = cell(1,plt_settings.n_figs);
    plt_settings.n_vars_plots = cell(1,plt_settings.n_figs);
    plt_settings.conv_factor_x = cell(1,plt_settings.n_figs);
    plt_settings.conv_factor_y = cell(1,plt_settings.n_figs);
    plt_settings.x_var_str = cell(1,plt_settings.n_figs);
    plt_settings.extra_args_plot = cell(1,plt_settings.n_figs);
    plt_settings.x_extra_args_text = cell(1,plt_settings.n_figs);
    plt_settings.y_extra_args_text = cell(1,plt_settings.n_figs);
    plt_settings.units_str = cell(1,plt_settings.n_figs);
    plt_settings.skip_data = cell(1,plt_settings.n_figs);
    plt_settings.add_leg = cell(1,plt_settings.n_figs);
    plt_settings.leg = cell(1,plt_settings.n_figs);

    for i = 1:plt_settings.n_figs
        plt_settings.n_vars_plots{i} = numel(plt_settings.y_var_str{i});
        plt_settings.use_subfig{i} = ~isequal(plt_settings.n_vars_plots{i},1);
        if ~plt_settings.use_subfig{i}
            plt_settings.subfig_ratio{i} = [1,1];
        else
            tmp = min(plt_settings.n_vars_plots{i},5);
            for ii = 2:tmp
                if (isequal(rem(plt_settings.n_vars_plots{i},ii),0) && plt_settings.n_vars_plots{i}/ii <= ii) % try to fit ina  square
                    if transpose_subfig_fl
                        plt_settings.subfig_ratio{i} = [ii, plt_settings.n_vars_plots{i}/ii];
                    else
                        plt_settings.subfig_ratio{i} = [plt_settings.n_vars_plots{i}/ii, ii];
                    end
                    break;
                elseif ii < tmp
                    continue
                end
                if transpose_subfig_fl
                    plt_settings.subfig_ratio{i} = [ii, ceil(plt_settings.n_vars_plots{i}/ii)];
                else
                    plt_settings.subfig_ratio{i} = [ceil(plt_settings.n_vars_plots{i}/ii), ii];
                end
            end
        end
        plt_settings.skip_data{1,i}(:) = cell(1,plt_settings.n_vars_plots{i});
        plt_settings.add_leg{1,i}(:) = cell(1,plt_settings.n_vars_plots{i});
        plt_settings.leg{1,i}(:) = cell(1,plt_settings.n_vars_plots{i});
        plt_settings.fig{1,i} = cell(1,plt_settings.n_vars_plots{i});
        plt_settings.x_var_str{1,i} = cell(1,plt_settings.n_vars_plots{i});
        plt_settings.extra_args_plot{1,i} = cell(1,plt_settings.n_vars_plots{i});
        plt_settings.x_extra_args_text{1,i} = cell(1,plt_settings.n_vars_plots{i});
        plt_settings.y_extra_args_text{1,i} = cell(1,plt_settings.n_vars_plots{i});
        plt_settings.conv_factor_x{1,i} = cell(1,plt_settings.n_vars_plots{i});
        plt_settings.conv_factor_y{1,i} = cell(1,plt_settings.n_vars_plots{i});
        plt_settings.units_str_x{1,i} = cell(1,plt_settings.n_vars_plots{i});
        plt_settings.units_str_y{1,i} = cell(1,plt_settings.n_vars_plots{i});
        
        plt_settings.skip_data{1,i}(:) = {false};
        plt_settings.add_leg{1,i}(:) = {false};
        plt_settings.leg{1,i}(:) = {{'FontSize', plt_settings.fnt, 'interpreter','latex',...
            'Orientation', 'horizontal', 'Location', 'best'}};
        plt_settings.x_var_str{1,i}(:) = {"Time"};
        plt_settings.extra_args_plot{1,i}(:) = {{...
            '-',...
            'Color',plt_settings.def_colors{uint8(color_id)},...
            'LineWidth',plt_settings.line_width_1,...
            'MarkerSize',plt_settings.marker_size_s,...
            "MarkerEdgeColor","black",...
            "MarkerFaceColor","black"}};
        plt_settings.x_extra_args_text{1,i}(:) = {{'FontSize', plt_settings.fnt, 'interpreter','latex'}};
        plt_settings.y_extra_args_text{1,i}(:) = {{'FontSize', plt_settings.fnt, 'interpreter','latex'}};
        plt_settings.units_str_x{1,i}(:) = {lib.macro.unit_wrap("s")};
        plt_settings.units_str_y{1,i}(:) = {""};
        plt_settings.conv_factor_x{1,i}(:) = {1};
        plt_settings.conv_factor_y{1,i}(:) = {1};
    end
end