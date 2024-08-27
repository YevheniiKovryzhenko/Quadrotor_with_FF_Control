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

classdef plotting < handle
    properties
        fig (1,2) cell
        fig_axes
        res TrajectoryOptimizer.solver
        col_tr = {'#D95319','#77AC30','#4DBEEE'};
        args_marker = {'o','MarkerSize', 3,'MarkerEdgeColor','black','MarkerFaceColor','blue'}
        args_marker_start = {'o','MarkerSize', 4,'MarkerEdgeColor','black','MarkerFaceColor','green'}
        args_marker_end = {'o','MarkerSize', 4,'MarkerEdgeColor','black','MarkerFaceColor','red'}
        fnt_args = {'FontWeight','bold','FontSize',16};
    end
    
    methods 
        function this = plotting(res)
            this.res = res;
            this = this.start_plot;            
            T = this.res.get_T;
            eval_time = linspace(T(1),T(end),1000);

            this.add_plot3_pos(this.fig_axes{1},eval_time)
            for i_dt = 1:this.res.N_dt+1
                this.add_plot_dts(this.fig_axes{1 + i_dt}, eval_time, i_dt - 1)
            end
        end
    end

    methods (Hidden = true)
        function this = start_plot(this)
            this.fig_axes = cell(this.res.N_dt+2);
            %initial setup
            pos = [50, 50, 900, 900];
            this.fig{1} = figure(); this.fig{1}.Position = pos;
            this.fig_axes{1} = gca(this.fig{1});
            this.hold_on(this.fig_axes{1});
            xlabel(this.fig_axes{1}, 'X (km)',this.fnt_args{:})
            ylabel(this.fig_axes{1}, 'Y (km)',this.fnt_args{:})
            zlabel(this.fig_axes{1}, 'Z (km)',this.fnt_args{:})
            this.hold_off(this.fig_axes{1});



            this.fig{2} = figure(); this.fig{2}.Position = pos;
            m = ceil((this.res.N_dt+1) / 2);
            n = ceil((this.res.N_dt+1) / m);
            for i_dt = 1:this.res.N_dt+1
                this.fig_axes{1 + i_dt} = subplot(n,m,i_dt);
                this.hold_on(this.fig_axes{1 + i_dt});
                xlabel(this.fig_axes{1 + i_dt}, 'Time (days)',this.fnt_args{:})
                if isequal(i_dt,1)
                    ylabel(this.fig_axes{1 + i_dt}, "Position (AU)", this.fnt_args{:})
                else
                    if isequal(i_dt,2)
                        ylabel(this.fig_axes{1 + i_dt}, sprintf("dt_{i} = %i (km/s)",i_dt-1),this.fnt_args{:})
                    else
                        ylabel(this.fig_axes{1 + i_dt}, sprintf("dt_{i} = %i (km/s^{%i})",i_dt-1,i_dt-1),this.fnt_args{:})
                    end                    
                end
                
                this.hold_off(this.fig_axes{1 + i_dt});
            end
        end

        function add_plot3_pos(this, fig_axes, eval_time)
            pos = this.res.eval(eval_time, 0);
            T_wpts = this.res.get_T;
            wpts = this.res.eval(T_wpts, 0);

            this.hold_on(fig_axes);
            plot3(fig_axes, pos(1,:), pos(2,:), pos(3,:),'-b');
            n = size(wpts,2);
            plot3(fig_axes, wpts(1,1), wpts(2,1), wpts(3,1),this.args_marker_start{:})
            plot3(fig_axes, wpts(1,n), wpts(2,n), wpts(3,n),this.args_marker_end{:})
            if n > 2
                plot3(fig_axes, wpts(1,2:n-1), wpts(2,2:n-1), wpts(3,2:n-1),this.args_marker{:})
            end
            this.hold_off(fig_axes);
        end

        function add_plot_dts(this, fig_axes, eval_time, i_dt)
            
            pos = this.res.eval(eval_time, i_dt);
            T_wpts = this.res.get_T;
            wpts = this.res.eval(T_wpts, i_dt);
            this.hold_on(fig_axes);
            n = size(wpts,2);
            
            for i_dim = 1:size(pos,1)
                plot(fig_axes, eval_time, pos(i_dim,:),'-','Color',this.col_tr{i_dim});
                plot(fig_axes, T_wpts, wpts(i_dim, :),this.args_marker{:})
                
                plot(fig_axes, T_wpts(1), wpts(i_dim, 1),this.args_marker_start{:})
                plot(fig_axes, T_wpts(n), wpts(i_dim, n),this.args_marker_end{:})
                if n > 2
                    plot(fig_axes, T_wpts(2:n-1), wpts(i_dim, 2:n-1),this.args_marker{:})
                end
            end            
            this.hold_off(fig_axes);
        end
        
    end

    methods (Static = true, Hidden =true)
        function hold_on(fig_axes)
            hold(fig_axes,"on")
        end

        function hold_off(fig_axes)
            axis(fig_axes,"tight")
            hold(fig_axes,"off")
        end
    end
end