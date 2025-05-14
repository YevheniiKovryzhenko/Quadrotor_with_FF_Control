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

classdef solver_common < TrajectoryOptimizer.common
    methods (Hidden)
        function this_ = update_constraints(this_, timePoints_)
            % Update constraints based on the provided time points
            % Ensure timePoints is a row vector
            timePoints_ = timePoints_(:)';

            % Evaluate waypoints and extra boundary conditions
            [waypoints, extra_bcs] = this_.wptFnc(timePoints_ / this_.TU_input_factor);
            waypoints = waypoints * this_.DU_input_factor;

            % Offset waypoints to simplify calculations
            this_.waypoints_offset = waypoints(:,1);
            waypoints = waypoints - this_.waypoints_offset;

            % Update dimensional properties
            this_.n_dim_src = size(waypoints, 1);
            this_.N_wps = size(waypoints, 2);
            this_.N_segments = this_.N_wps - 1;
            this_.stateSize = this_.N_COEFS * this_.N_segments;

            % Identify non-trivial dimensions
            nontriv_dim = false(this_.n_dim_src, 1);
            check_bcs__ = ~isempty(extra_bcs);
            for i_dim = 1:this_.n_dim_src
                nontriv_dim(i_dim) = any(waypoints(i_dim, :));
                if check_bcs__ && ~nontriv_dim(i_dim)
                    for i_dt = 1:min(this_.N_dt, length(extra_bcs))
                        nontriv_dim(i_dim) = ~isempty(extra_bcs{i_dt}) && any(isnan(extra_bcs{1}(1,:)));
                        if nontriv_dim(i_dim)
                            continue
                        end
                    end
                end
            end

            % Update non-trivial dimensions and waypoints
            this_.n_dim_ids = find(nontriv_dim);
            this_.n_dim = nnz(nontriv_dim);
            this_.waypoints = waypoints(nontriv_dim, :);
            this_.waypoints_src = waypoints;

            % Initialize constraints matrix
            this_.constraints = zeros((this_.N_dt + 1) * this_.N_wps, this_.n_dim);
            if this_.n_dim > 0
                % Populate constraints for each waypoint
                for k = 1:this_.N_wps
                    tmp_bcs = zeros(this_.n_dim, this_.N_dt + 1);
                    tmp_bcs(:, 1) = this_.waypoints(:, k);

                    % Apply specified boundary conditions
                    for i_dt = 1:min(length(extra_bcs), this_.N_dt)
                        tmp_bcs(:, i_dt + 1) = extra_bcs{i_dt}(this_.n_dim_ids, k) * ...
                            this_.DU_input_factor / this_.TU_input_factor^i_dt;
                    end

                    % Fill remaining boundary conditions with NaN for intermediate waypoints
                    for i_dt = length(extra_bcs) + 1:this_.N_dt
                        if k == 1 || k == this_.N_wps
                            continue;
                        end
                        tmp_bcs(:, i_dt + 1) = nan(this_.n_dim, 1);
                    end

                    % Assign constraints to the matrix
                    for i_dim = 1:this_.n_dim
                        for i_dt = 1:this_.N_dt + 1
                            this_.constraints((k - 1) * (this_.N_dt + 1) + i_dt, i_dim) = tmp_bcs(i_dim, i_dt);
                        end
                    end
                end
            end
        end

        function [this_] = computePolyCoefAndTimeOfArrival(this_)
            % Compute polynomial coefficients and time of arrival
            % Handles both fixed-time and time-optimized cases

            %#codegen

            % Initial guess for time segments
            initialGuess = diff(this_.timePoints);
            if this_.timeOptim && length(initialGuess) > 1
                % Perform time optimization
                [ppMatrix, tSegments, J, exitstruct] = this_.optimize(initialGuess);
                this_.Iterations = exitstruct.Iterations;

                % Print optimization status if enabled
                if this_.print_stats_fl
                    switch exitstruct.ExitFlag
                        case 0
                            fprintf("Problem solved. Local minimum found, iterations: %i\n", int32(exitstruct.Iterations));
                        case 1
                            fprintf("Failed to solve: time limit exceeded\n");
                        case 2
                            fprintf("Local minimum possible: step size below minimum, iterations: %i\n", int32(exitstruct.Iterations));
                        case 3
                            fprintf("Failed to solve: Hessian not positive semi-definite\n");
                        case 4
                            fprintf("Failed to solve: invalid search direction\n");
                        case 5
                            fprintf("Failed to solve: iteration limit exceeded\n");
                        otherwise
                            fprintf("Error in optimization: undefined exit flag: %i\n", int32(exitstruct.ExitFlag));
                    end
                end

                this_.ExitFlag = exitstruct.ExitFlag;
                timeOfArrival = [0 cumsum(tSegments)'];
            else
                % Solve polynomial coefficients for fixed-time case
                [ppMatrix, J] = this_.solvePoly(initialGuess);
                timeOfArrival = [0 cumsum(initialGuess(:))'];
                this_.Iterations = 0;

                if this_.print_stats_fl
                    fprintf("Fixed-time problem solved.\n");
                end

                this_.ExitFlag = 0;
            end

            % Update polynomial coefficients and time of arrival
            this_ = this_.pp_reverse_order(ppMatrix);
            this_.timeOfArrival = timeOfArrival / this_.TU_input_factor;
            this_.J = J;
            this_.cost_is_good = true;
        end

        function [p, t, J, exitstruct] = optimize(this_, initGuess)
            %This function is for internal use only. It may be removed in the future.
            %OPTIMIZE Find the polynomial coefficients and the optimal time segment lengths
            %   [P, T] = optimize(COSTWEIGHTIDX, CONSTRAINTS, KT,MINSEGMENTTIME,
            %   MAXSEGMENTTIME, INITGUESS, STATESIZE, NUMSEGMENTS,
            %   SEGMENTNUMCOEFFICIENT, SEGMENTORDER) computes the polynomial segment
            %   coefficients, P, and the optimal time segment lengths, T, while
            %   minimizing the jerk/snap. When the time segment lengths are specified,
            %   the polynomial coefficients that minimize the jerk/snap are obtained
            %   using matrix manipulations. An iterative process is used to find the
            %   optimal time allocation or find the time segment lengths. The inputs
            %   to the function are the cost weight index, COSTWEIGHTIDX, to either
            %   minimize jerk or snap, the boundary conditions specified in, CONSTRAINTS,
            %   the time weight, KT, the lower bound on the time segment length,
            %   MINSEGMENTTIME, the upper bound on the time segment length,
            %   MAXSEGMENTTIME, the initial guess for the time segment lengths,
            %   INITGUESS, the total number of state variables, STATESIZE, the number
            %   of polynomial segments, NUMSEGMENTS, the number of coefficients for
            %   each polynomial segment, SEGMENTNUMCOEFFICIENT and the polynomial order
            %   of the segments, SEGMENTORDER.
            
            %#codegen
            
            % Solver selected is damped BFGS gradient projection
            solver = TrajectoryOptimizer.DampedBFGSwGradientProjection;
        
            % Set solver parameters
            solver.ConstraintsOn = true;
            solver.RandomRestart = false;
        
            %Added this_ line to support code generation
            coder.varsize('initialGuess',[1,inf],[0,1]);
            initialGuess = initGuess;
        
            % Cost function
            solver.CostFcn = @(varargin)this_.trajectoryCost(varargin{:});
        
            % Pass extra arguments as a struct to the solver
            args.cost = 0;
            args.grads = zeros(1,numel(initialGuess));
            solver.ExtraArgs = args;

            
        
            % Random seed function
            % This is overridden due to codegen limitations and we are not using
            % random restart
            solver.RandomSeedFcn = @(varargin)this_.randfcn(varargin{:});
        
            % Specify the gradient function for the solver
            solver.GradientFcn = @(varargin)this_.gradient(varargin{:});
        
            % Specify the evaluation function for the solver
            solver.SolutionEvaluationFcn  = @(varargin)this_.solutionEval(varargin{:});
            
            if isempty(this_.constrFnc)
                % If minimum segment time is specified as a scalar, treat it as
                % minimum total time
                n = numel(initialGuess);
                if isscalar(this_.minSegmentTime)
                    % Set the constraint bounds for minimum total time
                    A11 = -eye(n); %each must be positive
                    A12 = -ones(n,1);%sum must be greater than min
    
                    A1 = [A11, A12];
    
                    b1 = -[zeros(n,1)
                        this_.minSegmentTime];
                else
                    % Set the constraint bounds for minimum segment time
                    A1 = -eye(n);
                    % Use the minimum segment time specified as a vector
                    b1 = -this_.minSegmentTime(:);
                end
            
                % Same as variable b1
                if isscalar(this_.maxSegmentTime)
                    A2 = ones(n,1); %sum must be less than max
    
                    b2 = [this_.maxSegmentTime];
                else
                    % Set the constraint bounds for max segment time
                    A2 = eye(n);
    
                    b2 = this_.maxSegmentTime(:);
                end
    
                solver.ConstraintMatrix = [A1 A2];        
                solver.ConstraintBound = [b1;b2];
            else
                [A, b] = this_.constrFnc();
                solver.ConstraintMatrix = A;        
                solver.ConstraintBound = b * this_.TU_input_factor;
            end
            
        
            % Solve the optimization problem with the given initial guess.
            %coder.varsize('t',[1,inf]);
            [t, exitstruct] = solver.solve(initialGuess);
            [p, J] = this_.solvePoly(t);
            % J = J + sum(t,"all");
        end

        function [p, J] = solvePoly(this_, T)
            % Update Constraints
            this_ = this_.update_constraints(T);

            % Initialize coefficients
            p = zeros(this_.N_segments, this_.N_COEFS, this_.n_dim);
        
            % Compute optimal polynomial coefficients for each dimension from the
            % given time segment lengths
            J= 0;

            for dimIdx = 1:this_.n_dim
                [p(:, :, dimIdx),tmp_J] = this_.solvePoly_(T, this_.constraints(:,dimIdx));
                J = J + tmp_J;
            end
        end

        function [p, J] = solvePoly_(this_, T, constr)            
            % Compute the optimal polynomial given time allocation and
            % constrained segment derivatives
            %     Returns
            %        p is polynomial coefficients for each segments
            %        J is the total cost
            %
            %
            %     CONSTRAINTS is a (numSegments+1)*segmentNumCoefficient/2
            %     vector. For each segment, you can constrain
            %     0:SegmentNumCoefficient/2-1 order of derivatives. For any
            %     free constraints, specify as NaN
            %
            %     Example for a 2 segment order-3 polynomial, constraints
            %     [1 0 1.5 NaN 2 0] limits the first waypoint to 1, its
            %     derivative to 0, limits the last waypoint to 2, its
            %     derivative to 0, the intermediate waypoint to 1.5, its
            %     derivative is free.
            %
            %     STATESIZE is the total number of parameters to be solved. It is equal
            %     to the numSegments*segmentNumCoefficient
            %
            %     NUMSEGMENTS is the total number of segments. This is equal to the
            %     number of waypoints-1
            %     SEGMENTNUMCOEFFICIENT is the total number of coefficients per
            %     segment. This is equal to the polynomial order + 1
            %
            
            %#codegen
            fixed_constr_ids = ~isnan(constr);
            numConstraints = sum(fixed_constr_ids);

            segmentNumCoefficient = this_.N_COEFS;
            numSegments = this_.N_segments;
        
            %Compute cost and boundary mapping matrices
            A_total_sp = spalloc(segmentNumCoefficient*numSegments,segmentNumCoefficient*numSegments,segmentNumCoefficient*segmentNumCoefficient*numSegments);
            Q_prime_total_sp = spalloc(segmentNumCoefficient*numSegments,segmentNumCoefficient*numSegments,segmentNumCoefficient*segmentNumCoefficient*numSegments);
            
            for segment = 1:numSegments
                offset = (segment-1)*segmentNumCoefficient;
                tmp_A = TrajectoryOptimizer.get_A(T(segment));
                tmp_Q = TrajectoryOptimizer.get_Q_prime(T(segment));
                
                    
                [tmp_i_row, tmp_i_col] = find(tmp_A);
                A_total_sp(tmp_i_row + offset, tmp_i_col + offset) = tmp_A(tmp_i_row, tmp_i_col);
                [tmp_i_row,tmp_i_col] = find(tmp_Q);
                Q_prime_total_sp(tmp_i_row + offset, tmp_i_col + offset) = tmp_Q(tmp_i_row, tmp_i_col);
            end
            A_total = A_total_sp;
            Q_prime_total = Q_prime_total_sp;
        
            %Compute matrix to convert problem from constrained to unconstrained
            %problem
            M = this_.constructM(constr);
        
            % Refer equations 15-17 in "Aggressive Flight of Fixed-Wing and Quadrotor
            % Aircraft in Dense Indoor Environments" [1].
            
            R = M'*Q_prime_total*M;
        
            upper = numConstraints;
            DF = constr(~isnan(constr));
            
            RPP = R(upper+1:end, upper+1:end);
            RPF = R(upper+1:end, 1:upper);
        
            DP = -RPP\(RPF*DF);
            D = [DF;DP];
        
            %solve for the polynomial coefficients  
            
            p = A_total\(M*D);
            p = reshape(p, segmentNumCoefficient, numSegments);
            p = p(end:-1:1, :);
            p = p';
            
            J = D'*R*D;
        end

        function M = constructM(this_, xCons)
            %constructM Duplicate intermediate waypoint derivative and continuity matrix
            % Matrix M duplicates each intermediate waypoint derivative value to appear
            % both at the end of one segment and at the beginning of the subsequent
            % segment. This maintains continuity at the intermediate waypoints
            
            %#codegen
            
            % M matrix same as the C matrix in equation 11 in "Polynomial
            % Planning for Aggressive Quadrotor Flight in Dense Indoor Environments" [2].
            nCons = this_.N_wps * (this_.N_dt + 1);
            
            % M1 = zeros(nCons,nCons);
            tmp = isnan(xCons);
            fixedBCIdx = find(~tmp);
            freeBCIdx = find(tmp);
            
            M_r = sparse([fixedBCIdx;freeBCIdx],1:nCons,ones(1,nCons));
            
            %each segment has numDerivatives as constraints at each endpoint.
            col_vec = zeros(1,this_.stateSize);
            row_vec = 1:this_.stateSize;
            for row = 1:this_.stateSize
                %the boundary conditions of a segments end point is equal to the
                %starting boundary conditions of the next segment. Continuity
                %enforcement
                segmentNumber = ceil(row/(this_.N_COEFS));
                col = row - (segmentNumber - 1)*(this_.N_dt+1);
            
                col_vec(row) = col;
            end
            M_c = sparse(row_vec, col_vec, ones(1,this_.stateSize));
            
            M = M_c*M_r;
        end

        function err = gradient(this_, T, args)
        %gradient Gradient function for solver
            err = this_.computeJacobian(T);
        end
        
        function err = solutionEval(this_, ~, args)
        %solutionEval Evaluation function for solver
            err = args.cost;
        end
        
        function z = randfcn(this_, ~)
        %randfcn
        % Since RandomRestart is set to false, this_ function is not required to be
        % defined. But this_ function needs to be defined here to support code
        % generation.
            z = 0.1*ones(1,numel(this_.N_segments));
        end

        function [J,a,b,args] = trajectoryCost(this_, T, args)
            %This function is for internal use only. It may be removed in the future.
            %trajectoryCost Compute the cost function.
            % This consists of two parts. First one computes jerk/snap cost and the
            % second computes the time cost
            
            % Copyright 2021 The MathWorks, Inc.
            %#codegen

            [~, J] = this_.solvePoly(T);
            J = J + this_.timeWt*sum(T,"all");
            
            args.cost = J;
        
            % a and b refer to the weight matrix and the Jacobian. We are not
            % computing Jacobian here and the weight matrix is not relevant to our
            % cost function. But the solver expects this_ function to have the given
            % signature.
        
            a = [];
        
            b = [];
        end

        function Jac = computeJacobian(this_, T)
            %This function is for internal use only. It may be removed in the future.
            %computeJacobian Computes the Jacobian matrix
            % This function computes the Jacobian of the cost function using a
            % numerical method.
            
            % Copyright 2021 The MathWorks, Inc.
            %#codegen
        
            Jac = zeros(1,this_.N_segments);
            %this_ value was changed from 1e-9 to 1e-5 to match fmincon results
            delta = 1e-5;
            scalar = 1/(2*delta);
        
            for kk = 1:numel(T)
                deltavec = zeros(this_.N_segments,1);
                deltavec(kk) = delta;
                deltaTP = T + deltavec;
                deltaTN = T - deltavec;

                %compute the perturbed costs
                [~,er1] = this_.solvePoly(deltaTP);
                [~,er2] = this_.solvePoly(deltaTN);
                er1 = er1 + this_.timeWt*(sum(deltaTP,"all"));
                er2 = er2 + this_.timeWt*(sum(deltaTN,"all"));
        
                Jac(kk) = (er1-er2)*scalar;
            end
        end
        
    end
end