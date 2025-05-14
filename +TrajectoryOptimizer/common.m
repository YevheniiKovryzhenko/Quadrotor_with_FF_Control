% /****************************************************************************
%  *
%  *    Copyright (C) 2025  Yevhenii Kovryzhenko. All rights reserved.
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

%{
    CLASS: TrajectoryOptimizer.common

    Purpose:
        Provides a base class for trajectory optimization, encapsulating
        common properties and methods for polynomial trajectory generation,
        evaluation, and scaling.

    Attributes:
        - constraints, constrFnc, wptFnc: Handles for constraints and waypoints.
        - n_dim_src, n_dim, n_dim_ids: Dimensionality properties.
        - N_wps, timePoints, waypoints_src, waypoints_offset, waypoints: Waypoint/time properties.
        - minSegmentTime, maxSegmentTime, timeWt: Time optimization properties.
        - timeOptim, print_stats_fl, cost_is_good: Flags for optimization and status.
        - pp, timeOfArrival, J, Iterations, ExitFlag: Results and status.
        - N_segments, stateSize: Trajectory structure properties.
        - DU_input_factor, TU_input_factor: Scaling factors.
        - N_dt, J_dt, N_COEFS: Constants for derivatives and coefficients.

    Usage:
        This class is intended to be subclassed by specific trajectory
        optimizer implementations. It provides utility methods for
        polynomial evaluation, scaling, and result extraction.
%}
classdef common < handle
    properties (Access = protected)
        % Function handles for constraints and waypoints
        constraints = []           % Matrix of constraints for trajectory segments
        constrFnc = []             % Function handle for time constraints
        wptFnc = []                % Function handle for waypoints

        % Dimensional properties
        n_dim_src (1,1) double     % Number of source dimensions
        n_dim (1,1) double         % Number of non-trivial dimensions
        n_dim_ids                  % Indices of non-trivial dimensions

        % Waypoint and time properties
        N_wps (1,1) double         % Number of waypoints
        timePoints                 % Time points for waypoints
        waypoints_src              % Original waypoints (unfiltered)
        waypoints_offset           % Offset for waypoints (for normalization)
        waypoints                  % Filtered waypoints (non-trivial dims)

        % Time optimization properties
        minSegmentTime             % Minimum segment time(s)
        maxSegmentTime             % Maximum segment time(s)
        timeWt (1,1) double = 1;  % Weight for time in cost function
        nontriv_wpts (1,1) double  % Number of non-trivial waypoints

        % Flags
        timeOptim (1,1) logical = false;      % Enable time allocation optimization
        print_stats_fl (1,1) logical = false; % Print optimization statistics
        cost_is_good (1,1) logical = false;   % Indicates if cost is valid

        % Results
        pp                         % Polynomial coefficients for each segment
        timeOfArrival              % Time of arrival at each waypoint
        J                          % Cost value
        Iterations                 % Number of optimization iterations
        ExitFlag                   % Exit flag from optimizer
        N_segments (1,1) double    % Number of trajectory segments
        stateSize (1,1) double     % Total number of state variables

        % Scaling factors
        DU_input_factor (1,1) double = 1; % Distance unit scaling
        TU_input_factor (1,1) double = 1; % Time unit scaling
    end

    properties (Constant)
        % Constants for derivatives and coefficients
        N_dt (1,1) double = 4; % Number of derivatives (e.g., snap)
        J_dt (1,1) double = 4; % Minimized derivative order
        N_COEFS (1,1) double = (max(TrajectoryOptimizer.common.N_dt, TrajectoryOptimizer.common.J_dt) + 1) * 2
            % Number of coefficients per segment (twice the max derivative order + 1)
    end

    methods
        %{
            FUNCTION: get_res

            Purpose:
                Returns a struct containing the polynomial coefficients,
                time of arrival, and offset for the trajectory.

            Output:
                res (struct): Contains fields 'pp', 'T', 'offset', 'n_dim',
                              'n_dim_src', 'n_dim_ids', 'N_segments'.
        %}
        function res = get_res(this_)
            res.pp = this_.pp;
            res.T = this_.get_T;
            res.offset = this_.waypoints_offset;

            res.n_dim = this_.n_dim;
            res.n_dim_src = this_.n_dim_src;
            res.n_dim_ids = this_.n_dim_ids;
            res.N_segments = this_.N_segments;
        end

        %{
            FUNCTION: get_stats

            Purpose:
                Returns a struct with statistics about the optimization
                process and solution.

            Output:
                stats (struct): Contains fields such as 'Iterations',
                                'ExitFlag', 'n_dim_src', 'n_dim',
                                'nontriv_wpts', 'N_wps', 'n_dim_ids', 'J'.
        %}
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

        %{
            FUNCTION: assign_bad_cost

            Purpose:
                Marks the current solution as invalid by setting the
                cost_is_good flag to false.

            Output:
                this_ (object): Updated object with cost_is_good = false.
        %}
        function this_ = assign_bad_cost(this_)
            this_.cost_is_good = false;
        end

        %{
            FUNCTION: check_solution

            Purpose:
                Checks if the current solution is valid and optimal.

            Output:
                is_good (logical): True if solution is valid and optimal.
        %}
        function is_good = check_solution(this_)
            is_good = false;
            if all(this_.cost_is_good)
                if (this_.ExitFlag == 0 || this_.ExitFlag == 2)
                    is_good = true;
                end
            end
        end

        %{
            FUNCTION: get_wpts

            Purpose:
                Returns the waypoints and their corresponding time points.

            Output:
                wpts: Waypoints as returned by wptFnc.
                T:    Time points for waypoints.
        %}
        function [wpts, T] = get_wpts(this_)
            T = this_.get_T;
            wpts = this_.wptFnc(diff(T) * this_.TU_input_factor);
        end

        %{
            FUNCTION: get_T

            Purpose:
                Returns the time of arrival vector for the trajectory.

            Output:
                T: Time of arrival at each waypoint.
        %}
        function T = get_T(this_)
            T = this_.timeOfArrival;
        end

        %{
            FUNCTION: update_TU

            Purpose:
                Updates the time unit scaling factor.

            Input:
                TU_new (double): New time unit scaling factor.

            Output:
                this_ (object): Updated object.
        %}
        function this_ = update_TU(this_, TU_new)
            this_.TU_input_factor = TU_new;
        end

        %{
            FUNCTION: update_DU

            Purpose:
                Updates the distance unit scaling factor.

            Input:
                DU_new (double): New distance unit scaling factor.

            Output:
                this_ (object): Updated object.
        %}
        function this_ = update_DU(this_, DU_new)
            this_.DU_input_factor = DU_new;
        end

        %{
            FUNCTION: get_DU_TU

            Purpose:
                Returns the current distance and time unit scaling factors.

            Output:
                DU_ (double): Distance unit scaling factor.
                TU_ (double): Time unit scaling factor.
        %}
        function [DU_, TU_] = get_DU_TU(this_)
            DU_ = this_.DU_input_factor;
            TU_ = this_.TU_input_factor;
        end

        %{
            FUNCTION: eval

            Purpose:
                Evaluates the trajectory (and its derivatives) at specified
                times using the stored polynomial coefficients.

            Input:
                eval_time (vector): Times at which to evaluate the trajectory.
                i_dt (int): Order of derivative to evaluate (0 = position).

            Output:
                res (matrix): Evaluated trajectory (or derivative) values.
        %}
        function [res] = eval(this_, eval_time, i_dt)
            res = TrajectoryOptimizer.common.eval_pp(eval_time, i_dt, this_.pp, this_.timeOfArrival, ...
                this_.n_dim, this_.n_dim_src, this_.n_dim_ids, this_.N_segments, this_.waypoints_offset, ...
                this_.DU_input_factor, this_.TU_input_factor);
        end
    end

    methods (Static)
        %{
            FUNCTION: eval_pp

            Purpose:
                Static utility to evaluate a piecewise polynomial trajectory
                and its derivatives at specified times.

            Input:
                eval_time (vector): Times to evaluate.
                i_dt (int): Derivative order.
                pp: Polynomial coefficients.
                timeOfArrival: Time of arrival at waypoints.
                n_dim, n_dim_src, n_dim_ids: Dimensionality info.
                n_int: Number of segments.
                waypoints_offset: Offset for waypoints.
                DU_input_factor, TU_input_factor: Scaling factors.

            Output:
                res (matrix): Evaluated values (size: n_dim_src x numSamples).
        %}
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
        %{
            FUNCTION: pp_reverse_order

            Purpose:
                Reverses the order of polynomial coefficients for each
                segment and dimension, storing them in the object's pp field.

            Input:
                pp: Polynomial coefficients (segments x coefficients x dims).

            Output:
                this_ (object): Updated object with reversed pp.
        %}
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
        %{
            FUNCTION: poly_val

            Purpose:
                Evaluates a polynomial or its derivative at a given value.

            Input:
                poly (vector): Polynomial coefficients (lowest to highest order).
                time (double): Value at which to evaluate.
                i_dt (int): Derivative order (0 = polynomial itself).

            Output:
                val (double): Evaluated value.
        %}
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