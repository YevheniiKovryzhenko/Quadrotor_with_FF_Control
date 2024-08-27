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

classdef configure_1 < handle    
    properties (Constant)
        def_colors = {...
        [0 0.4470 0.7410],...
        [0.8500 0.3250 0.0980],...
        [0.9290 0.6940 0.1250],...
        [0.4940 0.1840 0.5560],...
        [0.4660 0.6740 0.1880],...
        [0.3010 0.7450 0.9330],...
        [0.6350 0.0780 0.1840]}
    end

    properties
        fig (1,1) figure
        plot_type (1,1) lib.plot.plot_type = lib.plot.plot_type.plot
        figure_type (1,1) lib.plot.figure_type = lib.plot.figure_type.figure

        pos (1,4) double = [50 50 1200 1200];

        x_var_str   = ""
        y_var_str   = ""
        z_var_str   = ""
        
        conv_factor_x (1,1) double = 1 
        conv_factor_y (1,1) double = 1
        conv_factor_z (1,1) double = 1   

        extra_args_plot cell = {...
                    '-',...
                    'Color',lib.cofigure_1.def_colors{uint8(1)},...
                    'LineWidth',1.5,...
                    'MarkerSize',2.5,...
                    "MarkerEdgeColor","black",...
                    "MarkerFaceColor","black"}

        x_extra_args_text cell = {'FontSize', 30, 'interpreter','latex'}
        y_extra_args_text cell = {'FontSize', 30, 'interpreter','latex'}
        z_extra_args_text cell = {'FontSize', 30, 'interpreter','latex'}
        
        % units_str cell
        units_str_x = lib.macro.unit_wrap("s")
        units_str_y = ""
        units_str_z = ""
    end

    methods        
        function this = configure_1(figure_type, varargin)
            this.figure_type = figure_type;
            fig_ = figure; fig_.Position = this.pos;
            switch this.figure_type
                case lib.plot.figure_type.figure
                    this.fig = fig_;
                case lib.plot.figure_type.subfigure
                    if nargin < 3
                        error("Must provide number of subplots")
                    end
                    this.fig = subplot(varargin{1},varargin{2},ii);
            end
        end

        function this = start(this, data)
            set(0, 'DefaultAxesFontSize', this.fnt_s, 'DefaultAxesFontWeight','bold')
            set(0, 'DefaultTextFontSize', this.fnt_s, 'DefaultTextFontWeight','bold')
            this.pos             = [50 50 1200 1200];
        
            for i = 1:this.n_figs
                fig_ = figure; fig_.Position = this.pos;
                for ii = 1:this.n_vars_plots{i}
                    if this.use_subfig{i}
                        this.fig{i}{ii} = subplot(this.subfig_ratio{i}(1),this.subfig_ratio{i}(2),ii);
                        tmp_fig = this.fig{i}{ii};
                    else
                        this.fig{i} = fig_;
                        tmp_fig = gca(this.fig{i});
                    end
                    
                    hold(tmp_fig,"on")
                    plot(tmp_fig,...
                        data.(this.data_header_cell_x{i}{ii})*this.conv_factor_x{i}{ii},...
                        data.(this.data_header_cell_y{i}{ii})*this.conv_factor_y{i}{ii},...
                        this.extra_args_plot{i}{ii}{:})
                    xlabel(tmp_fig,...
                        this.x_var_str{i}{ii} + ...
                        this.units_str_x{1,i}{ii},...
                        this.x_extra_args_text{i}{ii}{:})
                    ylabel(tmp_fig,...
                        this.y_var_str{i}{ii} + ...
                        this.units_str_y{1,i}{ii},...
                        this.y_extra_args_text{i}{ii}{:})
                    axis tight
                    
                    hold(tmp_fig,"off")
                end
            end
        end

        function this = add(this, data)
            for i_figure = 1:this.n_figs
                for i_subfigure = 1:this.n_vars_plots{i_figure}
                    if this.skip_data{i_figure}{i_subfigure}
                        continue
                    end
                    if this.use_subfig{i_figure}
                        tmp_fig = this.fig{i_figure}{i_subfigure};
                    else
                        tmp_fig = gca(this.fig{i_figure});
                    end
                    hold(tmp_fig,"on")
                    plot(tmp_fig,...
                        data.(this.data_header_cell_x{i_figure}{i_subfigure})*this.conv_factor_x{i_figure}{i_subfigure},...
                        data.(this.data_header_cell_y{i_figure}{i_subfigure})*this.conv_factor_y{i_figure}{i_subfigure},...
                        this.extra_args_plot{i_figure}{i_subfigure}{:})
                    xlabel(tmp_fig,...
                        this.x_var_str{i_figure}{i_subfigure} + ...
                        this.units_str_x{1,i_figure}{i_subfigure},...
                        this.x_extra_args_text{i_figure}{i_subfigure}{:})
                    ylabel(tmp_fig,...
                        this.y_var_str{i_figure}{i_subfigure} + ...
                        this.units_str_y{1,i_figure}{i_subfigure},...
                        this.y_extra_args_text{i_figure}{i_subfigure}{:})
        
                    if (this.add_leg{i_figure}{i_subfigure})
                        legend(tmp_fig,...
                            this.leg{i_figure}{i_subfigure}{:})
                    end
                    hold(tmp_fig,"off")
                end
            end
        end
    end
end