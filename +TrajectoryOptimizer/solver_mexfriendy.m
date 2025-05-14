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
    CLASS: TrajectoryOptimizer.solver_mexfriendy

    Purpose:
        Provides a simplified, MEX-compatible solver for trajectory
        optimization, with streamlined input/output for code generation.

    Usage:
        Instantiate with required parameters for trajectory optimization.
        Designed for use in MEX or codegen environments.

    Methods:
        - solver_mexfriendy: Constructor for initialization.
%}
classdef solver_mexfriendy < TrajectoryOptimizer.solver_common
    %SOLVER_MEXFRIENDY A simplified solver for MEX compatibility
    %   Provides a streamlined interface for trajectory optimization
    %   with MEX-friendly inputs and outputs.
    
    methods 
        %{
            FUNCTION: solver_mexfriendy (constructor)

            Purpose:
                Initializes the MEX-friendly solver with provided parameters.

            Input:
                timePoints: Vector of time points for waypoints.
                TimeAllocation: Boolean, enable time allocation optimization.
                ShowDetails: Boolean, print optimization details.
                WaypointFunction: Function handle for waypoints.
                TimeConstraintFunction: Function handle for time constraints.
                MinSegmentTime: Minimum segment time(s).
                MaxSegmentTime: Maximum segment time(s).
                TimeWeight: Weight for time in cost function.
                TU: Time unit scaling factor.

            Output:
                this_ (object): Initialized solver object.
        %}
        function this_ = solver_mexfriendy(...
                timePoints, ...
                TimeAllocation, ...
                ShowDetails, ...
                WaypointFunction, ...
                TimeConstraintFunction, ...
                MinSegmentTime, ...
                MaxSegmentTime, ...
                TimeWeight, ...
                TU)
            % Constructor for the MEX-friendly solver
            % Initializes the solver with the provided parameters.

            %#codegen

            % Set the time unit scaling factor
            this_.TU_input_factor = TU;

            % Ensure timePoints is a row vector and scale by TU
            this_.timePoints = timePoints(:)' * this_.TU_input_factor;
            
            % Parse additional inputs
            this_.timeOptim = TimeAllocation;
            this_.print_stats_fl = ShowDetails;
            this_.wptFnc = WaypointFunction;
            this_.constrFnc = TimeConstraintFunction;
            this_.minSegmentTime = MinSegmentTime * this_.TU_input_factor;
            this_.maxSegmentTime = MaxSegmentTime * this_.TU_input_factor;
            this_.timeWt = TimeWeight;

            % Compute the polynomial segment coefficients and time of arrival
            this_ = this_.computePolyCoefAndTimeOfArrival;            
        end
    end  
end