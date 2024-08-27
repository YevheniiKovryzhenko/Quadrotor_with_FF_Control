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

classdef configure < handle    
    properties
        %font sizes
        fnt_s (1,1) double = 23 %font size small small
        fnt (1,1) double = 30 %font size small
        fnt_tit (1,1) double = 36 %font size title
        
        %line width
        line_width_1 (1,1) double = 1.5 %width for all
        
        %marker sizes
        marker_size (1,1) double = 15
        marker_size_s (1,1) double = 2.5

        pos (1,4) double = [50 50 1200 1200];
    end
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

    properties (Access=protected)
        x_var_str cell
        y_var_str cell

        data_header_cell_x cell
        data_header_cell_y cell

        n_figs (1,1) double = 0;
        fig cell
        subfig_ratio cell
        use_subfig cell
        n_vars_plots cell
        
        conv_factor_x cell
        conv_factor_y cell        
        
        extra_args_plot cell
        x_extra_args_text cell
        y_extra_args_text cell
        
        % units_str cell
        units_str_x cell
        units_str_y cell

        skip_data cell
        add_leg cell
        leg cell
    end

    methods
        function this = configure(n_figs, n_plots_per_fig)
            this.y_var_str = cell(1,n_figs);
            for i = 1:n_figs
                this.y_var_str{i} = cell(1,n_plots_per_fig(i));
                this.y_var_str{i}(:) = {""};
            end
            this.data_header_cell_x = this.y_var_str;
            this.data_header_cell_y = this.y_var_str;
            
            color_id = 1;
            
            this.n_figs = numel(this.y_var_str);
            this.fig = cell(1,this.n_figs);
            this.subfig_ratio = cell(1,this.n_figs);
            this.use_subfig = cell(1,this.n_figs);
            this.n_vars_plots = cell(1,this.n_figs);
            this.conv_factor_x = cell(1,this.n_figs);
            this.conv_factor_y = cell(1,this.n_figs);
            this.x_var_str = cell(1,this.n_figs);
            this.extra_args_plot = cell(1,this.n_figs);
            this.x_extra_args_text = cell(1,this.n_figs);
            this.y_extra_args_text = cell(1,this.n_figs);
            % this.units_str = cell(1,this.n_figs);
            this.skip_data = cell(1,this.n_figs);
            this.add_leg = cell(1,this.n_figs);
            this.leg = cell(1,this.n_figs);
        
            for i = 1:this.n_figs
                this.n_vars_plots{i} = numel(this.y_var_str{i});
                this.use_subfig{i} = ~isequal(this.n_vars_plots{i},1);
                if ~this.use_subfig{i}
                    this.subfig_ratio{i} = [1,1];
                else
                    tmp = min(this.n_vars_plots{i},5);
                    for ii = 2:tmp
                        if (isequal(rem(this.n_vars_plots{i},ii),0) && this.n_vars_plots{i}/ii <= ii) % try to fit ina  square
                            this.subfig_ratio{i} = [this.n_vars_plots{i}/ii,ii];
                            break;
                        elseif ii < tmp
                            continue
                        end
                        this.subfig_ratio{i} = [ceil(this.n_vars_plots{i}/ii),ii];
                    end
                end
                this.skip_data{1,i}(:) = cell(1,this.n_vars_plots{i});
                this.add_leg{1,i}(:) = cell(1,this.n_vars_plots{i});
                this.leg{1,i}(:) = cell(1,this.n_vars_plots{i});
                this.fig{1,i} = cell(1,this.n_vars_plots{i});
                this.x_var_str{1,i} = cell(1,this.n_vars_plots{i});
                this.extra_args_plot{1,i} = cell(1,this.n_vars_plots{i});
                this.x_extra_args_text{1,i} = cell(1,this.n_vars_plots{i});
                this.y_extra_args_text{1,i} = cell(1,this.n_vars_plots{i});
                this.conv_factor_x{1,i} = cell(1,this.n_vars_plots{i});
                this.conv_factor_y{1,i} = cell(1,this.n_vars_plots{i});
                this.units_str_x{1,i} = cell(1,this.n_vars_plots{i});
                this.units_str_y{1,i} = cell(1,this.n_vars_plots{i});
                
                this.skip_data{1,i}(:) = {false};
                this.add_leg{1,i}(:) = {false};
                this.leg{1,i}(:) = {{'FontSize', this.fnt, 'fontweight','bold','interpreter','latex'}};
                this.x_var_str{1,i}(:) = {"\textbf{Time}"};
                this.extra_args_plot{1,i}(:) = {{...
                    '-',...
                    'Color',this.def_colors{uint8(color_id)},...
                    'LineWidth',this.line_width_1,...
                    'MarkerSize',this.marker_size_s,...
                    "MarkerEdgeColor","black",...
                    "MarkerFaceColor","black"}};
                this.x_extra_args_text{1,i}(:) = {{'FontSize', this.fnt, 'fontweight','bold','interpreter','latex'}};
                this.y_extra_args_text{1,i}(:) = {{'FontSize', this.fnt, 'fontweight','bold','interpreter','latex'}};
                this.units_str_x{1,i}(:) = {lib.macro.unit_wrap("s")};
                this.units_str_y{1,i}(:) = {""};
                this.conv_factor_x{1,i}(:) = {1};
                this.conv_factor_y{1,i}(:) = {1};
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