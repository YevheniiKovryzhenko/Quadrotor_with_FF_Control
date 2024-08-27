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

classdef common < handle
    properties (Access = protected)
        constrFnc = []
        wptFnc = []
        n_dim_src (1,1) double
        n_dim (1,1) double
        n_dim_ids
        
        N_wps (1,1) double
        
        timeWt (1,1) double = 1;
        nontriv_wpts (1,1) double
        constraints

        waypoints_src
        waypoints_offset
        waypoints
        timePoints

        minSegmentTime
        maxSegmentTime
        
        %flags
        timeOptim (1,1) logical = false;
        print_stats_fl (1,1) logical = false;
        cost_is_good (1,1) logical = false;
        

        %results
        pp
        timeOfArrival
        J
        Iterations
        ExitFlag

        N_segments (1,1) double

        % Total number of states or boundary conditions
        stateSize (1,1) double % = segmentNumCoefficient*numSegments;

        DU_input_factor (1,1) double = 1;
        TU_input_factor (1,1) double = 1;
    end

    properties (Constant)
        N_dt (1,1) double = 4; %numer of derivatives (for constraints)
        J_dt (1,1) double = 4; %minimize derivative
        N_COEFS (1,1) double = (max(TrajectoryOptimizer.common.N_dt,TrajectoryOptimizer.common.J_dt) + 1)*2

        % DU_input_factor (1,1) double = 1;
        % TU_input_factor (1,1) double = 1;
    end

    methods
        function res = get_res(this_)
            res.pp = this_.pp;
            res.T = this_.get_T;
            res.offset = this_.waypoints_offset;

            res.n_dim = this_.n_dim;
            res.n_dim_src = this_.n_dim_src;
            res.n_dim_ids = this_.n_dim_ids;
            res.N_segments = this_.N_segments;
        end
        function stats = get_stats(this_)            
            stats.Iterations = this_.Iterations;
            stats.ExitFlag = this_.ExitFlag;
            stats.n_dim_src = this_.n_dim_src;
            stats.n_dim = this_.n_dim;
            stats.nontriv_wpts = this_.nontriv_wpts;
            stats.N_wps = this_.N_wps;
            stats.n_dim_ids = this_.n_dim_ids;

            if this_.check_solution
                stats.J = this_.J;%sqrt(this_.J) / this_.DU_input_factor * (this_.TU_input_factor)^this_.J_dt;
            else
                stats.J = realmax;
            end
        end

        function this_ = assign_bad_cost(this_)
            this_.cost_is_good = false;
        end

        function is_good = check_solution(this_)
            is_good = false;
            if all(this_.cost_is_good)
                if (this_.ExitFlag == 0 || this_.ExitFlag == 2)
                    is_good = true;
                end
            end
        end

        function [wpts, T] = get_wpts(this_)
            T = this_.get_T;
            wpts = this_.wptFnc(diff(T) * this_.TU_input_factor);
        end

        function T = get_T(this_)
            T = this_.timeOfArrival;
        end

        function this_ = update_TU(this_, TU_new)
            this_.TU_input_factor = TU_new;
        end

        function this_ = update_DU(this_, DU_new)
            this_.DU_input_factor = DU_new;
        end

        function [DU_, TU_] = get_DU_TU(this_)
            DU_ = this_.DU_input_factor;
            TU_ = this_.TU_input_factor;
        end

        function [res] = eval(this_, eval_time, i_dt)
            res = TrajectoryOptimizer.common.eval_pp(eval_time, i_dt, this_.pp, this_.timeOfArrival, ...
                this_.n_dim, this_.n_dim_src, this_.n_dim_ids, this_.N_segments, this_.waypoints_offset, ...
                this_.DU_input_factor, this_.TU_input_factor);
        end
    end

    methods (Static)
        function [res] = eval_pp(eval_time, i_dt, pp, timeOfArrival, ...
                n_dim, n_dim_src, n_dim_ids, n_int, waypoints_offset, ...
                DU_input_factor, TU_input_factor)
            %Interpolate to polynomial coefficients with the specified
            % number of samples
            
            %#codegen
            n_dt = TrajectoryOptimizer.common.N_dt;            
            % TU_input_factor = TrajectoryOptimizer.common.TU_input_factor;
            % DU_input_factor = TrajectoryOptimizer.common.DU_input_factor;
            
            eval_time = eval_time(:).' * TU_input_factor;
            timeOfArrival_ = timeOfArrival(:).' * TU_input_factor;

            numSamples = length(eval_time);
            if i_dt > n_dt
                i_dt = n_dt;
            elseif i_dt < 0
                i_dt = 0;
            end
            
            
            T_int = diff(timeOfArrival_);
            n_wpts = n_int + 1;
        
            % Initialize trajectory
            res = zeros(n_dim_src,numSamples);
            
            for i_eval = 1:numSamples
                t                                   = eval_time(i_eval);
            
                % Find the segment number in which the time samples fall
                if t                                <= 0
                    i_int                           = 1;
                    t                               = 0;
                elseif t                            >= timeOfArrival_(n_wpts)
                    i_int                           = n_int;
                    t                               = timeOfArrival_(n_wpts);
                else
                    i_int                           = nnz(timeOfArrival_ < t);
                end
                
                % Compute distance of time sample from the segment start time
                if i_int                            > n_int
                    tau                             = 1;
                    i_int                           = n_int;
                elseif i_int                        < 1
                    i_int                           = 1;
                    tau                             = 0;
                else
                    tau                             = (t - timeOfArrival_(i_int))/(T_int(i_int));
                end

                gain_T = (1/T_int(i_int))^i_dt;
                    
                for i_dim_non_triv = 1:n_dim
                    i_dim = n_dim_ids(i_dim_non_triv);
                    polys_tmp = pp(i_int,:,i_dim);           
                    
                    % Construct piece-wise polynomial from pp
                    res(i_dim,i_eval) = TrajectoryOptimizer.common.poly_val(polys_tmp(:).',tau,i_dt)*gain_T;
                end
            end
            if isequal(i_dt, 0)
                res = res + waypoints_offset;
            end

            res = res * (1 / DU_input_factor) * (TU_input_factor)^i_dt;
        end
    end

    methods (Hidden = true)

        function this_ = pp_reverse_order(this_, pp)
            % Reverse order of coeffs.            
            %#codegen
            
            this_.pp = zeros([this_.N_segments, this_.N_COEFS, this_.n_dim_src]);
            
            % reshape the pp matrix to required form 
            for i = 1:this_.N_segments    
                for k = 1:this_.n_dim
                    this_.pp(i,:,this_.n_dim_ids(k)) = pp(i,end:-1:1,k);
                end
            end
        end
    end

    methods (Static = true)
        function val    = poly_val(poly,time,i_dt)
            val         = 0;
            n           = length(poly)-1;
            if i_dt    <= 0
                for i   = 0:n
                    val = val+poly(i+1)*time^i;
                end
            else
                for i   = i_dt:n
                    a   = poly(i+1)*prod(i-i_dt+1:i)*time^(i-i_dt);
                    val = val + a;
                end
            end
        end        
    end
end