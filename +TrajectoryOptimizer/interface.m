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
    CLASS: TrajectoryOptimizer.interface

    Purpose:
        Abstract interface for trajectory optimizer classes, providing
        input parsing, scaling, and initialization logic.

    Usage:
        Subclass this interface and implement the abstract method
        computePolyCoefAndTimeOfArrival.

    Methods:
        - computePolyCoefAndTimeOfArrival: Abstract method for subclasses.
        - interface: Constructor for input parsing and initialization.
        - check_input_double, check_input_fnc, check_input_bool: Static
          utility methods for input parsing and validation.
        - stringToChar: Static utility for string-to-char conversion.
%}
classdef interface < TrajectoryOptimizer.common
    methods (Abstract = true)
        %{
            FUNCTION: computePolyCoefAndTimeOfArrival

            Purpose:
                Abstract method to be implemented by subclasses for
                computing polynomial coefficients and time of arrival.

            Output:
                this_ (object): Updated object with computed trajectory.
        %}
        this_ = computePolyCoefAndTimeOfArrival(this_)
    end

    methods 
        %{
            FUNCTION: interface (constructor)

            Purpose:
                Handles input parsing, scaling, and initialization for
                trajectory optimizer objects.

            Input:
                timePoints: Vector of time points for waypoints.
                varargin: Name-value pairs for configuration.

            Output:
                this_ (object): Initialized object.
        %}
        function this_ = interface(timePoints, varargin)
            %#codegen
            narginchk(2, 1 + 9 * 2);
            numInputArgs = nargin;

            % Convert strings to chars for codegen support
            charInputs = this_.stringToChar(numInputArgs, varargin{:});

            % Scaling factors
            this_.DU_input_factor = this_.check_input_double(charInputs, 'DU', this_.DU_input_factor);
            this_.TU_input_factor = this_.check_input_double(charInputs, 'TU', this_.TU_input_factor);

            % Ensure timePoints is a row vector
            this_.timePoints = timePoints(:)' * this_.TU_input_factor;
            
            % Parse additional inputs
            this_.timeOptim = this_.check_input_bool(charInputs, 'TimeAllocation', this_.timeOptim);
            this_.print_stats_fl = this_.check_input_bool(charInputs, 'ShowDetails', this_.print_stats_fl);
            this_.wptFnc = this_.check_input_fnc(charInputs, 'WaypointFunction', this_.wptFnc);
            if this_.timeOptim
                this_.constrFnc = this_.check_input_fnc(charInputs, 'TimeConstraintFunction', this_.constrFnc);
                if isempty(this_.constrFnc)
                    % Default segment time constraints
                    minSegmentTimeDefault = 0.5 * min(diff(timePoints(:)'));
                    maxSegmentTimeDefault = 1.5 * max(diff(timePoints(:)'));
                    this_.minSegmentTime = this_.check_input_double(charInputs, 'MinSegmentTime', minSegmentTimeDefault) * this_.TU_input_factor;
                    this_.maxSegmentTime = this_.check_input_double(charInputs, 'MaxSegmentTime', maxSegmentTimeDefault) * this_.TU_input_factor;
                end
                this_.timeWt = this_.check_input_double(charInputs, 'TimeWeight', this_.timeWt);
            end
            
            % Compute polynomial coefficients and time of arrival
            this_ = this_.computePolyCoefAndTimeOfArrival;
        end
    end    

    methods (Static = true, Hidden = true)
        %{
            FUNCTION: check_input_double

            Purpose:
                Parses a double input from name-value pairs.

            Input:
                charInputs: Cell array of input arguments.
                check_str: Name of the parameter to check.
                def: Default value.

            Output:
                value (double): Parsed value or default.
        %}
        function value = check_input_double(charInputs,check_str,def)
            %#codegen
            %coder.varsize('value',1);
            value = double(def);
            n = numel(charInputs);
            tmp_bool = check_input_;
            if any(tmp_bool)                
                for i = 1:n-1
                    if tmp_bool(i)
                        tmp = charInputs{i+1};
                        if ~isempty(tmp) & isnumeric(tmp) & isreal(tmp) & ~any(isinf(tmp))                 
                            value = double(tmp);
                        end
                        return;
                    end
                end        
            end

            function out_bool = check_input_
                %#codegen
                out_bool = false(1,n); 
                for i_ = 1:n
                    out_bool(1,i_) = strcmp(charInputs{i_},check_str);
                end
            end
        end

        %{
            FUNCTION: check_input_fnc

            Purpose:
                Parses a function handle input from name-value pairs.

            Input:
                charInputs: Cell array of input arguments.
                check_str: Name of the parameter to check.
                def: Default value.

            Output:
                value (function_handle): Parsed value or default.
        %}
        function value = check_input_fnc(charInputs,check_str,def)
            %#codegen
            % coder.varsize('value',1);
            value = def;
            n = numel(charInputs);
            tmp_bool = check_input_;
            if any(tmp_bool)                
                for i = 1:n-1
                    if tmp_bool(i)
                        tmp = charInputs{i+1};
                        if ~isempty(tmp) & isa(tmp,'function_handle')                
                            value = tmp;
                        end
                        return;
                    end
                end        
            end

            function out_bool = check_input_
                %#codegen
                out_bool = false(1,n); 
                for i_ = 1:n
                    out_bool(1,i_) = strcmp(charInputs{i_},check_str);
                end
            end
        end
        
        %{
            FUNCTION: check_input_bool

            Purpose:
                Parses a boolean input from name-value pairs.

            Input:
                charInputs: Cell array of input arguments.
                check_str: Name of the parameter to check.
                def: Default value.

            Output:
                value (logical): Parsed value or default.
        %}
        function value = check_input_bool(charInputs,check_str,def)
            %#codegen
            coder.varsize('value',1);
            value = logical(def);
            n = numel(charInputs);
            tmp_bool = check_input_;
            if any(tmp_bool)                
                for i = 1:n-1
                    if tmp_bool(i)
                        tmp = charInputs{i+1};
                        if isscalar(tmp)                
                            value = boolean(tmp);
                        end
                        return;
                    end
                end        
            end

            function out_bool = check_input_
                %#codegen
                out_bool = false(1,n); 
                for i_ = 1:n
                    out_bool(1,i_) = strcmp(charInputs{i_},check_str);
                end
            end
        end
        
        %{
            FUNCTION: stringToChar

            Purpose:
                Converts string inputs to char arrays for codegen support.

            Input:
                nargin: Number of input arguments.
                varargin: Input arguments.

            Output:
                charInputs: Cell array of char arrays.
        %}
        function charInputs = stringToChar(nargin,varargin)
            %charInputs Convert strings to chars case by case for codegen support
            
            %#codegen
            
            if nargin > 22
                charInputs = cell(1,22);
                [charInputs{:}] = convertStringsToChars(varargin{:});
            elseif nargin > 20
                charInputs = cell(1,20);
                [charInputs{:}] = convertStringsToChars(varargin{:});
            elseif nargin > 18
                charInputs = cell(1,18);
                [charInputs{:}] = convertStringsToChars(varargin{:});
            elseif nargin > 16
                charInputs = cell(1,16);
                [charInputs{:}] = convertStringsToChars(varargin{:});
            elseif nargin > 14
                charInputs = cell(1,14);
                [charInputs{:}] = convertStringsToChars(varargin{:});
            elseif nargin > 12
                charInputs = cell(1,12);
                [charInputs{:}] = convertStringsToChars(varargin{:});
            elseif nargin > 10
                charInputs = cell(1,10);
                [charInputs{:}] = convertStringsToChars(varargin{:});
            elseif nargin > 8
                charInputs = cell(1,8);
                [charInputs{:}] = convertStringsToChars(varargin{:});
            elseif nargin > 6
                charInputs = cell(1,6);
                [charInputs{:}] = convertStringsToChars(varargin{:});
            elseif nargin > 4
                charInputs = cell(1,4);
                [charInputs{:}] = convertStringsToChars(varargin{:});
            elseif nargin > 2
                charInputs = cell(1,2);
                [charInputs{:}] = convertStringsToChars(varargin{:});
            else
                charInputs = {};
            end
        end
    end
end