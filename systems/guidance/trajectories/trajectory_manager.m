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

classdef trajectory_manager < handle %#codegen
    properties
        ID (1,1) trajectory_ID
        result (1,1)
    end
    methods 
        function this_ = trajectory_manager(opt)
            this_.ID = trajectory_ID(opt);
            this_.result = this_.get_solve_traj();
        end
    end
    methods (Access = private)
        function traj = get_solve_traj(this_)
            [wpts, ~, Tf, dt_eval, wptFnc] = trajectory_manager.get_traj(this_.ID);
            
            traj = this_.solve(wpts, Tf, dt_eval, wptFnc);
    
            if this_.ID == trajectory_ID.DEFAULT
                traj.minsnap_set.n_dim_ids = 0;
            end
        end
        
        
    end
    
    methods (Static, Hidden)
        function res = solve(wpts, Tf, dt_eval, wptFnc)
            timeWt = 0;
            res.wpts = wpts;
            res.timeWt = timeWt;
            
            scaling = 10;
            
            tpts = linspace(0,Tf,size(wpts,2));
            
            opt_time_allocation_fl = true;
            
            minSegmentTime = 0.1*diff(tpts);
            maxSegmentTime = Tf;
            
            % wptFnc = @(t) bcs_fun__(t, wpts);
            
            % tic
            sol = TrajectoryOptimizer.solver(tpts,...
                'WaypointFunction', wptFnc,...
                'TimeAllocation',opt_time_allocation_fl,...
                'ShowDetails', true, ...
                'TimeWeight', timeWt, ...
                'MinSegmentTime', minSegmentTime, ...
                'MaxSegmentTime', maxSegmentTime, ...
                'TU', scaling / Tf);
            % toc
            
            stats = sol.get_stats;
            minsnap_set = sol.get_res;
            
            
            
            if any(diff(minsnap_set.T) - minSegmentTime < 1E-3)
                fprintf("Warning: lower bound active for some segment times\n")
            end

            % minsnap_set.T = minsnap_set.T * Tf / scaling;
            
            eval_time = 0:dt_eval:minsnap_set.T(end);
            states = sol.eval(eval_time,0);
            d_states = sol.eval(eval_time,1);
            dd_states = sol.eval(eval_time,2);
            ddd_states = sol.eval(eval_time,3);
            dddd_states = sol.eval(eval_time,4);
            
            
            res.t = eval_time;
            res.p = states(1:3,:);
            res.v = d_states(1:3,:);
            res.a = dd_states(1:3,:);
            res.jerk = ddd_states(1:3,:);
            res.snap = dddd_states(1:3,:);
            
            res.states_cell = {states,d_states,dd_states,ddd_states,dddd_states};
            
            res.psi = states(4,:);
            res.psi_dot = d_states(4,:);
            res.psi_ddot = dd_states(4,:);
            
            res.minsnap_set = minsnap_set;
            res.cost = stats.J;
            res.iterations = stats.Iterations;
        end

        %% preset trajectories %%
        function [wpts, obst, Tf, dt_eval, wptFnc] = get_traj(traj_ID)            
            switch traj_ID    
                case trajectory_ID.yaw_1        
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_yaw_1;
                    
                case trajectory_ID.line_x_1
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_line_x_1;
                    
                case trajectory_ID.line_y_1
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_line_y_1;
                    
                case trajectory_ID.line_z_1
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_line_z_1;
                    
                case trajectory_ID.line_xy_1        
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_line_xy_1;
                    
                case trajectory_ID.line_xy_2        
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_line_xy_2;
                    
                case trajectory_ID.line_xyz_1        
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_line_xyz_1;
                    
                case trajectory_ID.line_xyz_2        
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_line_xyz_2;
                        
                case trajectory_ID.fig_8_1
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_8_1;
                    
                case trajectory_ID.fig_8_1_slow
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_8_1_slow;

                case trajectory_ID.fig_8_2_slow
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_8_2_slow;
                    
                case trajectory_ID.fig_8_yaw_1_slow
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_8_yaw_1_slow;
                    
                case trajectory_ID.fig_8_yaw_2_slow
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_8_yaw_2_slow;
                    
                case trajectory_ID.fig_8_yaw_1
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_8_yaw_1;
                    
                case trajectory_ID.fig_8_yaw_2
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_8_yaw_2;
                    
                case trajectory_ID.rrt_test_1
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_rrt_test_1;
                    
                case trajectory_ID.square_1
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_square_1;
                    
                case trajectory_ID.square_1_slow
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_square_1_slow;

                case trajectory_ID.square_2
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_square_2;
                    
                case trajectory_ID.circle_1
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_circle_1;
                    
                case trajectory_ID.spiral_1
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_spiral_1;
                    
                case trajectory_ID.spiral_yaw_1
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_spiral_yaw_1;

                case trajectory_ID.non_rest2rest
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_non_rest2rest;

                case trajectory_ID.collision_avoidance_loiter_1
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_collision_avoidance_loiter_1;

                case trajectory_ID.line_y_obst_1
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.traj_line_y_obst_1;
                    
                otherwise
                    [wpts, obst, Tf, dt_eval, wptFnc] = trajectory_manager.def_traj;
            end
        end


        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_yaw_1
            wpts = zeros(4,3);
            wpts(4,:) = linspace(0,pi, 3);
            
            dt_eval = 0.01; %s
            Tf = 5;

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_square_1
            wpts = [1,   0,     -1,     -1,     -1,     0,      1,   1,   1
                    1,   1,      1,     0,      -1,     -1,     -1,  0,   1
                    -1, -1, -1, -1, -1, -1, -1, -1, -1
                    0, 0, 0, 0, 0, 0, 0, 0, 0];
            
            dt_eval = 0.01; %s
            Tf = 10;

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_square_1_slow
            [wpts, obst, ~, dt_eval, wptFnc] = trajectory_manager.traj_square_1();
            Tf = 20;
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_square_2
            % offset = [0.0; 0.7; 0.0];
            offset = [0.0; 0.0; 0.0];
            % [wpts, ~, Tf, dt_eval, wptFnc] = trajectory_manager.traj_square_1();
            wpts = [1,   0,     -1,     -1,     -1,     0,      1,   1,   1
                    1,   1,      1,     0,      -1,     -1,     -1,  0,   1
                    -1, -1, -1, -1, -1, -1, -1, -1, -1
                    0, 0, 0, 0, 0, 0, 0, 0, 0];
            wpts(1:3,:) = wpts(1:3,:) + offset(:);
            dt_eval = 0.01; %s
            Tf = 10;

            wptFnc = @(t) bcs_fun__(t, wpts);
            

            obst = {...
                obstacle(1,[-1.6, 0.3, -1]+offset(:)',[1, 0.4, 3]), ...
                obstacle(2,[0, -0.8, -1]+offset(:)',[0.5, 0.3, 3]), ...
                obstacle(3,[0.7, -1, -1]+offset(:)',[0.5, 0.3, 3]), ...
                obstacle(4,[0.0, 0.0, -1]+offset(:)',[0.7, 0.7, 3])};

            % obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_spiral_yaw_1
            Tf = 15;
            
            tmp_t = 0:Tf/16:Tf;
            wpts = [2*tmp_t/Tf.*sin(3*2*pi*tmp_t/Tf)
                    2*tmp_t/Tf.*cos(3*2*pi*tmp_t/Tf)
                    -ones(1,length(tmp_t))
                    -(0:3*2*pi/16:3*2*pi)];
            
            dt_eval = 0.01; %s

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_spiral_1
            Tf = 15;
            
            tmp_t = 0:Tf/16:Tf;
            wpts = [2*tmp_t/Tf.*sin(3*2*pi*tmp_t/Tf)
                    2*tmp_t/Tf.*cos(3*2*pi*tmp_t/Tf)
                    -ones(1,length(tmp_t))
                    -zeros(1,length(tmp_t))];
            
            dt_eval = 0.01; %s

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_rrt_test_1
            wpts = [...
                                5.4 3 -0.4 pi
                                4.2 1.6 -1.4 pi
                                2.8 1.6 -1.4 pi 
                                0.5 1.8 -0.9 pi
                                0.4 0.8 -0.4 pi].';
            
            dt_eval = 0.01; %s
            Tf = 9;     

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_line_z_1
            wpts = zeros(4,3);
            wpts(3,:) = [0, 1, 3];
            
            dt_eval = 0.01; %s
            Tf = 5;    

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_line_y_1
            wpts = zeros(4,3);
            wpts(2,:) = [0, 1, 3];
            
            dt_eval = 0.01; %s
            Tf = 5;

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_line_y_obst_1
            wpts = zeros(4,7);
            wpts(3,:) = -1;
            wpts(2,:) = [-2, -1.5, -1, 0, 1, 1.5, 2];
            
            dt_eval = 0.01; %s
            Tf = 10;

            obst = {...
                obstacle(1,[0, -.1, -1],[0.2, 0.2, 3])};

            wptFnc = @(t) bcs_fun__(t, wpts);
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_line_xyz_2
            wpts = zeros(4,3);
            wpts(1,:) = [0, 1, 3];
            wpts(2,:) = [0, 0.75, 1.5];
            wpts(3,:) = -[0, 0.5, 2];
            
            dt_eval = 0.01; %s
            Tf = 5;        

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_line_xyz_1
            wpts = zeros(4,3);
            wpts(1,:) = [0, 1, 3];
            wpts(2,:) = [0, 1, 3];
            wpts(3,:) = [0, 1, 3];
            
            dt_eval = 0.01; %s
            Tf = 5;        

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_line_xy_2
            %line_x 
            wpts = zeros(4,3);
            wpts(1,:) = [0, 1, 3];
            wpts(2,:) = [0, 0.75, 1.5];
            
            dt_eval = 0.01; %s
            Tf = 5;     

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_line_xy_1
            wpts = zeros(4,3);
            wpts(1,:) = [0, 1, 3];
            wpts(2,:) = [0, 1, 3];
            
            dt_eval = 0.01; %s
            Tf = 5;        

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_line_x_1
            wpts = zeros(4,3);
            wpts(1,:) = [0, 1, 3];
            
            dt_eval = 0.01; %s
            Tf = 5;        

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_circle_1
            Tf = 10;
            
            tmp_t = 0:Tf/7:Tf;
            wpts = [2*sin(2*pi*tmp_t/Tf)
                    2*cos(2*pi*tmp_t/Tf)
                    -ones(1,length(tmp_t))
                    -zeros(1,length(tmp_t))];
            
            dt_eval = 0.01; %s    

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_8_yaw_2
            wpts = [0,       0.5,    0.0,    -0.5,   0.0,    0.5,    0.0,    -0.5,   0.0;...
                    0,       1.0,    2.0,    1.0,    0.0,    -1.0,   -2.0,   -1.0,   0.0;...
                    -[0,       0.25,   0.5,    0.25,   0.0,    -0.25,  -0.5,   -0.25,  0.0]-0.8;...
                    [45,      90.0,   180.0,  270.0,  315.0,  270.0,  180.0,  90.0,   45.0]*pi/180];
            
            dt_eval = 0.01; %s
            Tf = 15;    

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_8_yaw_1
            wpts = [0,       0.5,    0.0,    -0.5,   0.0,    0.5,    0.0,    -0.5,   0.0;...
                    0,       1.0,    2.0,    1.0,    0.0,    -1.0,   -2.0,   -1.0,   0.0;...
                    0,       0.25,   0.5,    0.25,   0.0,    -0.25,  -0.5,   -0.25,  0.0;...
                    ([45,      90.0,   180.0,  270.0,  315.0,  270.0,  180.0,  90.0,   45.0]-45)*pi/180];
            
            dt_eval = 0.01; %s
            Tf = 15;       

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_8_yaw_1_slow
            wpts = [0,       0.5,    0.0,    -0.5,   0.0,    0.5,    0.0,    -0.5,   0.0;...
                    0,       1.0,    2.0,    1.0,    0.0,    -1.0,   -2.0,   -1.0,   0.0;...
                    0,       0.25,   0.5,    0.25,   0.0,    -0.25,  -0.5,   -0.25,  0.0;...
                    ([45,      90.0,   180.0,  270.0,  315.0,  270.0,  180.0,  90.0,   45.0]-45)*pi/180];
            
            dt_eval = 0.01; %s
            Tf = 20;        

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_8_1
            wpts = [0,       0.5,    0.0,    -0.5,   0.0,    0.5,    0.0,    -0.5,   0.0;...
                    0,       1.0,    2.0,    1.0,    0.0,    -1.0,   -2.0,   -1.0,   0.0;...
                    [0,       0.25,   0.5,    0.25,   0.0,    -0.25,  -0.5,   -0.25,  0.0]-3;...
                    -[45,      90.0,   180.0,  270.0,  315.0,  270.0,  180.0,  90.0,   45.0]*0];
            
            dt_eval = 0.01; %s
            Tf = 15;        

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_8_1_slow
            wpts = [0,       0.5,    0.0,    -0.5,   0.0,    0.5,    0.0,    -0.5,   0.0;...
                    0,       1.0,    2.0,    1.0,    0.0,    -1.0,   -2.0,   -1.0,   0.0;...
                    0,       0.25,   0.5,    0.25,   0.0,    -0.25,  -0.5,   -0.25,  0.0;...
                    ([45,      90.0,   180.0,  270.0,  315.0,  270.0,  180.0,  90.0,   45.0]-45)*0.0];
            
            dt_eval = 0.01; %s
            Tf = 20;
            
            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_8_2_slow
            wpts = [0,       0.5,    0.0,    -0.5,   0.0,    0.5,    0.0,    -0.5,   0.0;...
                    0,       1.0,    2.0,    1.0,    0.0,    -1.0,   -2.0,   -1.0,   0.0;...
                    -[0,       0.25,   0.5,    0.25,   0.0,    -0.25,  -0.5,   -0.25,  0.0]-0.8;...
                    ([45,      90.0,   180.0,  270.0,  315.0,  270.0,  180.0,  90.0,   45.0]-45)*0.0];
            
            dt_eval = 0.01; %s
            Tf = 20;

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_8_yaw_2_slow
            wpts = [0,       0.5,    0.0,    -0.5,   0.0,    0.5,    0.0,    -0.5,   0.0;...
                    0,       1.0,    2.0,    1.0,    0.0,    -1.0,   -2.0,   -1.0,   0.0;...
                    -[0,       0.25,   0.5,    0.25,   0.0,    -0.25,  -0.5,   -0.25,  0.0]-0.8;...
                    ([45,      90.0,   180.0,  270.0,  315.0,  270.0,  180.0,  90.0,   45.0]-45)*pi/180];
            
            dt_eval = 0.01; %s
            Tf = 20;

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_non_rest2rest
            [wpts, obst, Tf, dt_eval] = trajectory_manager.traj_square_1;
            Tf = Tf + 3;
            wptFnc = @(t) bcs_fun__(t,wpts);
            
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                n_dt = 4;
                extra_bcs = cell(1,n_dt);
                [n_dims, n_wpts] = size(wpts);
                extra_bcs(:) = {NaN(n_dims, n_wpts)};
                for i_dt = 1:n_dt
                    for i_dim = 1:n_dims
                        extra_bcs{i_dt}(i_dim,1) = i_dim / i_dt / n_dims;
                        extra_bcs{i_dt}(i_dim,n_wpts) = 0;
                    end
                end
            end
        end
        
        function [wpts, obst, Tf, dt_eval, wptFnc] = def_traj
            wpts = zeros(4,2);
            wpts(3,:) = -1;
            
            dt_eval = 0.01; %s
            Tf = 10;     

            wptFnc = @(t) bcs_fun__(t, wpts);
            obst = {};
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end

        function [wpts, obst, Tf, dt_eval, wptFnc] = traj_collision_avoidance_loiter_1           
            dt_eval = 0.01; %s
            Tf = 60*2;
            go2pts = 4;
            returnpts = 4;

            loiter_center = [200;200;-30;0];
            loiter_radius = 50;
            loiter_go2merge_pt = 160;
            loiter_return_pt = -60;
            loiter_revs = 0;
            loiter_dir_ccw = false;
            loiter_wpts = 10;


            wpts = zeros(4,go2pts + loiter_wpts + returnpts);
            index = go2pts;
            wpts(:,1:go2pts) = [...
                linspace(0,loiter_radius*cosd(loiter_go2merge_pt) + loiter_center(1),go2pts)
                linspace(0,loiter_radius*sind(loiter_go2merge_pt) + loiter_center(2),go2pts)
                linspace(0,loiter_center(3),go2pts)
                zeros(1,go2pts)];
            

            if loiter_dir_ccw
                total_angle2loiter = rem(abs(loiter_return_pt - loiter_go2merge_pt), 360);
            else
                total_angle2loiter = rem(abs(loiter_go2merge_pt - loiter_return_pt), 360);
            end

            total_angle2loiter = total_angle2loiter +...
                + 360*loiter_revs;
            
            angle_step = total_angle2loiter / loiter_wpts;
            if (~loiter_dir_ccw)
                angle_step = -angle_step;
            end
            
            for i = 1:loiter_wpts
                index = index + 1;
                wpts(:,index) = loiter_center + ...
                    [...
                    loiter_radius*cosd(loiter_go2merge_pt + angle_step*(i));
                    loiter_radius*sind(loiter_go2merge_pt + angle_step*(i));
                    0;
                    0];

            end
            tmp = [...
                linspace(0, loiter_radius*cosd(loiter_return_pt) + loiter_center(1),returnpts+1)
                linspace(0, loiter_radius*sind(loiter_return_pt) + loiter_center(2),returnpts+1)
                linspace(0, loiter_center(3),returnpts+1)
                zeros(1,returnpts+1)];
            wpts(:,(returnpts:-1:1) + index) = tmp(:,1:returnpts);
            id = 0;

            id = id +1;
            fence_y = obstacle(id,...
                [loiter_center(1)/2; 0.2*loiter_center(2); (4*loiter_center(3)/32)],...
                [loiter_center(1)*2, 1, abs(3*loiter_center(3)/16)]);
            
            id = id +1;
            obst_central_building_wide_part = obstacle(id,...
                loiter_center(1:3) + [-2*loiter_radius + 3*loiter_radius/8; -3*loiter_radius/8 - 1.2*loiter_radius/2; -loiter_center(3) + 7*loiter_center(3)/32],...
                [4*loiter_radius, loiter_radius*1.2, abs(7*loiter_center(3)/16)]);
            id = id +1;
            obst_central_building_tall_part = obstacle(id,...
                loiter_center(1:3) + [0; 0; -loiter_center(3) + (14*loiter_center(3)/32)],...
                [3*loiter_radius/4, 3*loiter_radius/4, abs(14*loiter_center(3)/16)]);
            id = id +1;
            obst_central_building_wide_part_extension = obstacle(id,...
                [75, 125, -14],...
                [10, 5, 7]); 
            
            id = id +1;
            obst_tall_tower_1 = obstacle(id,loiter_center(1:3) + ...
                [loiter_radius*cosd(loiter_go2merge_pt + angle_step*3); ...
                loiter_radius*sind(loiter_go2merge_pt + angle_step*3); ...
                -loiter_center(3)-abs(loiter_center(3))],[3, 3, 2*abs(loiter_center(3))]);
            id = id +1;
            obst_tall_tower_2 = obstacle(id,loiter_center(1:3) + ...
                [loiter_radius*cosd(loiter_go2merge_pt + angle_step*6); ...
                loiter_radius*sind(loiter_go2merge_pt + angle_step*6); ...
                -loiter_center(3)-abs(loiter_center(3))],[3, 3, 2*abs(loiter_center(3))]);
            
            % building2fence_dist = abs(obst_central_building_wide_part.center(2) - fence_y.center(2));
            % fence_length = fence_y.width(2);

            % trees_rows = 6;
            % trees_cols = 20;
            % trees = cell(1,trees_rows * trees_cols);
            % start_id = 6;
            % ind = 0;
            % for i = 1:trees_rows
            %     for ii = 1:trees_cols
            %         ind = ind + 1;
            %         trees{ind} = obstacle(ind+start_id,...
            %             [fence_y.center(1) - 0.8*fence_length/2 + 0.8*fence_length / trees_cols * (ii); ...
            %             fence_y.center(2) + 0.9*building2fence_dist / trees_rows * (i); ...
            %             -abs(loiter_center(3))/6],...
            %             [1.5, 1.5, abs(loiter_center(3))/3]);
            %     end
            % end
            
            
            % obst_central_building_wide_part_extension
            obst = {fence_y, obst_central_building_wide_part, obst_central_building_tall_part, obst_tall_tower_1, obst_tall_tower_2};


            wptFnc = @(t) bcs_fun__(t, wpts);
            
            function [wpts, extra_bcs] = bcs_fun__(~, wpts)
                extra_bcs = {};
            end
        end
        
    end
end