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

classdef solver_common < TrajectoryOptimizer.common
    methods (Hidden)
        function this_ = update_constraints(this_, timePoints_)
            % Ensure timePoints is a row vector
            timePoints_ = timePoints_(:)';

            [waypoints, extra_bcs] = this_.wptFnc(timePoints_ / this_.TU_input_factor);
            waypoints = waypoints*this_.DU_input_factor;

            this_.waypoints_offset = waypoints(:,1);
            waypoints = waypoints - this_.waypoints_offset;
            
            % Dimension of the waypoint
            this_.n_dim_src = size(waypoints,1);
            % Number of waypoints
            this_.N_wps = size(waypoints,2);
            this_.N_segments = this_.N_wps - 1;

            this_.stateSize = this_.N_COEFS*this_.N_segments;
            
            %check for trivial solution:
            nontriv_dim = false(this_.n_dim_src,1);
            check_bcs__ = ~isempty(extra_bcs);
            for i_dim = 1:this_.n_dim_src
                nontriv_dim(i_dim) = any(waypoints(i_dim,:));
                if (check_bcs__ && ~nontriv_dim(i_dim))
                    for i_dt = 1:min(this_.N_dt, length(extra_bcs))
                        nontriv_dim(i_dim) = ~isempty(extra_bcs{i_dt}) && any(isnan(extra_bcs{1}(1,:)));
                        if nontriv_dim(i_dim)
                            continue
                        end
                    end
                end
            end
            n_nontriv_dim = nnz(nontriv_dim);
            this_.n_dim_ids = find(nontriv_dim);
            this_.n_dim = n_nontriv_dim;
            
            % nontriv_wpts = zeros(n_nontriv_dim,N_wps);
            this_.waypoints = waypoints(nontriv_dim,:);
            this_.waypoints_src = waypoints;

            % Re-order constraints in desired format. Note that the input is
            % transposed before passing as arguments.

            % Each column will correspond to all the constraints for each
            % dimension. For example, for a 2D problem with 3 waypoints, and for
            % jerk
            % constraints = [x1 dx1 ddx1 dddx1 x2 dx2 ddx2 dddx2 x3 dx3 ddx3 dddx3;
            %                y1 dy1 ddy1 dddy1 y2 dy2 ddy2 dddy2 y3 dy3 ddy3 dddy3]'
            % and for snap
            % constraints = [x1 dx1 ddx1 dddx1 ddddx1 x2 dx2 ddx2 dddx2 ddddx2 x3 dx3 ddx3 dddx3 ddddx3;
            %                y1 dy1 ddy1 dddy1 ddddy1 y2 dy2 ddy2 dddy2 ddddy2 y3 dy3 ddy3 dddy3 ddddy3]'

            % Default boundary conditions. Zero at start and end waypoints. NaN at
            % intermediate waypoints
            
            n_specified_bcs = min(length(extra_bcs), this_.N_dt);

            % BCs_cell = cell(1,this_.N_wps);
            this_.constraints = zeros((this_.N_dt+1)*this_.N_wps, n_nontriv_dim);
            if n_nontriv_dim > 0
                for k = 1:this_.N_wps
                    tmp_bcs = zeros(this_.n_dim,this_.N_dt+1);
                    tmp_wpts = this_.waypoints(:,k);
                    tmp_bcs(1:this_.n_dim, 1) = tmp_wpts(:);
                    
                    i_dt = 0;
                    for i = 1:n_specified_bcs
                        i_dt = i_dt + 1;
                        % tmp_ = extra_bcs{i_dt}(this_.n_dim_ids,k) * this_.DU_input_factor / this_.TU_input_factor^i_dt;
                        % tmp_bcs(1:this_.n_dim, i_dt+1) = tmp_(1:n_nontriv_dim,k);
                        tmp_bcs(1:this_.n_dim, i_dt+1) = extra_bcs{i_dt}(this_.n_dim_ids,k) * this_.DU_input_factor / this_.TU_input_factor^i_dt;                        
                    end
    
                    for i = 1:this_.N_dt - n_specified_bcs
                        if isequal(k, 1) || isequal(k,this_.N_wps)
                            continue
                        else
                            tmp = nan(this_.n_dim,1);
                        end
                        tmp_bcs(1:this_.n_dim, i_dt+i+1) = tmp(:);
                    end
    
                    % BCs_cell{k} = tmp_bcs;
                    for i = 1:this_.n_dim
                        for ii = 1:this_.N_dt+1
                            this_.constraints(ii + (k-1)*(this_.N_dt+1),i) = tmp_bcs(i,ii);
                        end
                    end
                end
                % this_.constraints =  [BCs_cell{:}]';
            end            
        end

        % function orderedConstraints = orderConstraints(this_)
        %     %orderedConstraints Re-order the constraints in desired format. Each column
        %     % corresponds to constraints for a particular dimension.
        %     %#codegen
        % 
        %     % Each column will correspond to all the constraints for each
        %     % dimension. For example, for a 2D problem with 3 waypoints, and for
        %     % jerk
        %     % constraints = [x1 dx1 ddx1 dddx1 x2 dx2 ddx2 dddx2 x3 dx3 ddx3 dddx3;
        %     %                y1 dy1 ddy1 dddy1 y2 dy2 ddy2 dddy2 y3 dy3 ddy3 dddy3]'
        %     % and for snap
        %     % constraints = [x1 dx1 ddx1 dddx1 ddddx1 x2 dx2 ddx2 dddx2 ddddx2 x3 dx3 ddx3 dddx3 ddddx3;
        %     %                y1 dy1 ddy1 dddy1 ddddy1 y2 dy2 ddy2 dddy2 ddddy2 y3 dy3 ddy3 dddy3 ddddy3]'
        % 
        %     % Default boundary conditions. Zero at start and end waypoints. NaN at
        %     % intermediate waypoints
        %     BCDefault = [zeros(n_nontriv_dim,1) nan(n_nontriv_dim,this_.N_wps-2) zeros(n_nontriv_dim,1)];
        % 
        %     [~, ] = this_.wptFnc(timePoints_);
        % 
        %     % assign boundary conditions
        %     BCs = repmat(BCDefault',[1,1,this_.N_dt]);
        % 
        %     constraints = zeros((this_.N_dt+1)*this_.N_wps,this_.n_dim);
        % 
        %     for k = 1:this_.N_wps
        %         if isequal(this_.n_dim,1)
        %             constraints((this_.N_dt+1)*(k-1)+1:(this_.N_dt+1)*k,:) = [this_.waypoints(:,k)';(squeeze(BCs(k,:,:)))];
        %         else
        %             constraints((this_.N_dt+1)*(k-1)+1:(this_.N_dt+1)*k,:) = [this_.waypoints(:,k)';(squeeze(BCs(k,:,:)))'];
        %         end
        %     end
        %     orderedConstraints = constraints;
        % end

        function [this_] = computePolyCoefAndTimeOfArrival(this_)
            %computePolyCoefAndTimeOfArrival Compute polynomial segment coefficients and time of arrival
            % This function computes the polynomial coefficients and the time of
            % arrival given the cost weight, time weight, minimum segment weight,
            % maximum segment weight and the segment order. If time optimization
            % property is set to true, then this_ function returns the optimal polynomial
            % coefficients and the time of arrival.
            
            %#codegen        
        
            % Initial guess for the time segment lengths when time optimization is
            % selected
            initialGuess = diff(this_.timePoints);
            if this_.timeOptim && length(initialGuess) > 1
                % Perform time optimization and jerk minimization
                [ppMatrix, tSegments, J, exitstruct] = this_.optimize(initialGuess);
                this_.Iterations = exitstruct.Iterations;
                if this_.print_stats_fl
                    switch exitstruct.ExitFlag
                        case 0
                            fprintf("Problem solved. Local minimum found, number of iterations: %i\n",int32(exitstruct.Iterations))
                        case 1
                            fprintf("Failed to solve the problem: time limit exceeded\n");
                        case 2
                            fprintf("Local minimum possible: step size below minimum, number of iterations: %i\n",int32(exitstruct.Iterations));
                        case 3
                            fprintf("Failed to solve the problem: hessian not positive semi-definite\n");
                        case 4
                            fprintf("Failed to solve the problem: search direction invalid\n");
                        case 5
                            fprintf("Failed to solve the problem: iteration limit exceeded\n");
                        otherwise
                            fprintf("Error in optimize: undefined exit flag: %i",int32(exitstruct.ExitFlag))
                    end
                end
                this_.ExitFlag = exitstruct.ExitFlag;
        
                timeOfArrival = [0 cumsum(tSegments)'];
            else
        
                % Solve the polynomial coefficients. Since the time of arrival is
                % specified, no time optimization or allocation is required here.
                % The polynomial coefficients are obtained by simple matrix
                % manipulations and inversion
                [ppMatrix, J] = this_.solvePoly(initialGuess);
            
                % timeofArrival is same as the specified time points
                timeOfArrival = [0 cumsum(initialGuess(:))'];

                this_.Iterations = 0;
                if this_.print_stats_fl
                    fprintf("Fixed-time problem solved.\n")
                end

                this_.ExitFlag = 0;
            end            
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