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

function plt_settings = start(plt_settings,data)
    % set(0, 'DefaultAxesFontSize', plt_settings.fnt_s, 'DefaultAxesFontWeight','bold')
    % set(0, 'DefaultTextFontSize', plt_settings.fnt_s, 'DefaultTextFontWeight','bold')
    set(0, 'DefaultAxesFontSize', plt_settings.fnt_s)
    set(0, 'DefaultTextFontSize', plt_settings.fnt_s)

    pos             = [50 50 1200 1200];

    for i = 1:plt_settings.n_figs
        fig = figure; fig.Position = pos;
        for ii = 1:plt_settings.n_vars_plots{i}
            if plt_settings.use_subfig{i}
                plt_settings.fig{i}{ii} = subplot(plt_settings.subfig_ratio{i}(1),plt_settings.subfig_ratio{i}(2),ii);
                tmp_fig = plt_settings.fig{i}{ii};
            else
                plt_settings.fig{i} = fig;
                tmp_fig = gca(plt_settings.fig{i});
            end
            
            hold(tmp_fig,"on")
            plot(tmp_fig,...
                data.(plt_settings.data_header_cell_x{i}{ii})*plt_settings.conv_factor_x{i}{ii},...
                data.(plt_settings.data_header_cell_y{i}{ii})*plt_settings.conv_factor_y{i}{ii},...
                plt_settings.extra_args_plot{i}{ii}{:})
            xlabel(tmp_fig,...
                plt_settings.x_var_str{i}{ii} + ...
                plt_settings.units_str_x{1,i}{ii},...
                plt_settings.x_extra_args_text{i}{ii}{:})
            ylabel(tmp_fig,...
                plt_settings.y_var_str{i}{ii} + ...
                plt_settings.units_str_y{1,i}{ii},...
                plt_settings.y_extra_args_text{i}{ii}{:})
            axis tight
            if plt_settings.grid_on_fl
                grid on
            end
            if plt_settings.box_on_fl
                box on
            end

            if ~isempty(plt_settings.x_nticks)
                min_val = (min(data.(plt_settings.data_header_cell_x{i}{ii})*plt_settings.conv_factor_x{i}{ii}));
                max_val = (max(data.(plt_settings.data_header_cell_x{i}{ii})*plt_settings.conv_factor_x{i}{ii}));
                if min_val > 0
                    min_val = 1.1*min_val;
                else
                    min_val = 0.9*min_val;
                end
                if max_val > 0
                    max_val = 0.9*max_val;
                else
                    max_val = 1.1*max_val;
                end
                if min_val ~= max_val && min_val < max_val
                    scaling = 10;
                    min_val = ceil(min_val * scaling)/scaling;
                    max_val = floor(max_val * scaling)/scaling;
                    if min_val ~= max_val && min_val < max_val
                        scaling = max_val - min_val;
                        xticks(round(linspace(min_val, max_val, plt_settings.x_nticks) / scaling, 1) * scaling);
                    end
                end
            end

            if ~isempty(plt_settings.y_nticks)
                min_val = (min(data.(plt_settings.data_header_cell_y{i}{ii})*plt_settings.conv_factor_y{i}{ii}));
                max_val = (max(data.(plt_settings.data_header_cell_y{i}{ii})*plt_settings.conv_factor_y{i}{ii}));
                if min_val > 0
                    min_val = 1.1*min_val;
                else
                    min_val = 0.9*min_val;
                end
                if max_val > 0
                    max_val = 0.9*max_val;
                else
                    max_val = 1.1*max_val;
                end
                if min_val ~= max_val && min_val < max_val
                    scaling = 10;
                    min_val = ceil(min_val * scaling)/scaling;
                    max_val = floor(max_val * scaling)/scaling;
                    if min_val ~= max_val && min_val < max_val
                        scaling = (max_val - min_val);
                        yticks(round(linspace(min_val, max_val, plt_settings.y_nticks) / scaling, 1) * scaling);
                    end
                end
            end
            
            hold(tmp_fig,"off")
        end
    end
end