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

classdef DampedBFGSwGradientProjection < TrajectoryOptimizer.NLPSolverInterface
    %This class is for internal use only. It may be removed in the future.
    
    %DAMPEDBFGSWGRADIENTPROJECTION Solver for problems with nonlinear cost
    %   and linear constraints. 
    %
    %   min    F(x)
    %    x
    %   s.t.  A* x <= b
    %   where x is an n-by-1 vector; F(x) is a scalar and a generic nonlinear
    %   function of x; A is an m-by-n matrix, b is an m-by-1 vector.
    %   Note, m is the number of constraints, n is the number of variables.
    
    
    %   Copyright 2016-2019 The MathWorks, Inc.
    %
    %   References:
    %
    %   [1] J. Zhao and N. Badler, Inverse kinematics positioning using
    %   nonlinear programming for highly articulated figures
    %   ACM Transactions on Graphics, Vol. 13, No. 4, 1994.
    %
    %   [2] H. Badreddine, S. Vandewalle and J. Meyers, Sequential
    %   quadratic programming for optimal control in direct numerical
    %   simulation of turbulent flow
    %   Journal of Computational Physics, 256, 1-16, 2014
    %
    %   [3] D. Goldfarb, Extension of Davidon's variable metric method to
    %   maximization under linear inequality and equality constraints
    %   SIAM Journal of Applied Mathematics, vol. 17, No. 4, 1969
    %
    %   [4] J. Nocedal and S. Wright, Numerical Optimization (Second Ed.)
    %   Springer, New York, 2006 
    % 
    %   [5] D. Bertsekas, Nonlinear Programming (Second Ed.)
    %   Athena Scientific, Belmont, MA, 1999
      
    %#codegen
    
    properties (Access = protected)
        
        %GradientTolerance The iteration stops if the norm of the gradient 
        %   falls below this value (a positive value)
        GradientTolerance
        
        %ArmijoRule parameters (for line search)
        %   The Armijo condition for a viable step:
        %   f(x) - f(x + step) >= - sigma* grad(x) * step
        %   where step = direction * beta^k
        %ArmijoRuleBeta A scalar between 0 and 1
        ArmijoRuleBeta
        
        %ArmijoRuleSigma A small positive number
        ArmijoRuleSigma

        %TimeObjInternal An object of SystemTimeProvider, for timing in
        %   solveInternal
        TimeObjInternal 
        
        %MaxNumConstraints Max number of constraints, declared nontunable. 
        %   Needed for codegen, esp. when ConstraintsOn property is false
        MaxNumConstraints = 200
        
        %MaxNumVariables Max number of variables, declared nontunable. 
        %   Needed for codegen, esp. when ConstraintsOn property is false
        MaxNumVariables = 200
    end
    
    methods
        function obj = DampedBFGSwGradientProjection(varargin)
            %DampedBFGSwGradientProjection Constructor
            obj.MaxNumIteration = 1000;
            obj.MaxTime = 30;
            obj.GradientTolerance = 1e-7;
            obj.SolutionTolerance = 1e-6;
            obj.ArmijoRuleBeta = 0.4;
            obj.ArmijoRuleSigma = 1e-5;
            obj.ConstraintsOn = true;
            obj.RandomRestart = true;
            obj.StepTolerance = 1e-14;
            obj.Name = 'BFGSGradientProjection';
            
            coder.varsize('A', [obj.MaxNumVariables, obj.MaxNumConstraints], [true, true]);
            coder.varsize('b', [obj.MaxNumConstraints, 1], [true, false]);
            A = zeros(0,0);
            b = zeros(0,1);
            obj.ConstraintMatrix = A;
            obj.ConstraintBound = b;
            
            narginchk(0,1);
            %This solver can be called with the logical flag, UseTimer. It
            %can be either TRUE (default) or FALSE. This input is for
            %internal use only, and may not be supported in a future
            %release.
            %
            %When the value is TRUE, the solver uses the a timer object
            %that relies on platform-specific code, SystemTimeProvide
            %MockSystemTimeProvider, a placeholder that has the same
            %properties and methods, but which does not require any
            %platform specific code and is therefore compatible with
            %cross-platform deployment. In that case, the solver does not
            %check for a maximum solver time, as no timer object is given.
            %When the value is FALSE, the solver uses
            
            if nargin > 0
                obj.UseTimer = varargin{:};
            else
                obj.UseTimer = true;
            end
            
            if obj.UseTimer
                obj.TimeObj = robotics.core.internal.SystemTimeProvider();
                obj.TimeObjInternal = robotics.core.internal.SystemTimeProvider();
            else
                obj.TimeObj = robotics.core.internal.MockSystemTimeProvider();
                obj.TimeObjInternal = robotics.core.internal.MockSystemTimeProvider();
            end
        end
        
        function params = getSolverParams(obj)
            params.Name = obj.Name;
            params.MaxNumIteration = obj.MaxNumIteration;
            params.MaxTime = obj.MaxTime;
            params.GradientTolerance = obj.GradientTolerance;
            params.SolutionTolerance = obj.SolutionTolerance;
            params.ArmijoRuleBeta = obj.ArmijoRuleBeta;
            params.ArmijoRuleSigma = obj.ArmijoRuleSigma;
            params.ConstraintsOn = obj.ConstraintsOn;
            params.RandomRestart = obj.RandomRestart;
            params.StepTolerance = obj.StepTolerance;  
        end

        function setSolverParams(obj, params)
            obj.MaxNumIteration =   params.MaxNumIteration;
            obj.MaxTime =           params.MaxTime;
            obj.GradientTolerance = params.GradientTolerance;
            obj.SolutionTolerance = params.SolutionTolerance;
            obj.ConstraintsOn =     params.ConstraintsOn;
            obj.RandomRestart =     params.RandomRestart;
            obj.StepTolerance = params.StepTolerance;  
        end
        

        function newobj = copy(obj)
            %COPY
            newobj = DampedBFGSwGradientProjection();
                        
            newobj.Name = obj.Name;
            newobj.ConstraintBound = obj.ConstraintBound;
            newobj.ConstraintMatrix = obj.ConstraintMatrix;
            newobj.ConstraintsOn = obj.ConstraintsOn;
            newobj.SolutionTolerance = obj.SolutionTolerance;
            newobj.RandomRestart = obj.RandomRestart;

            newobj.SeedInternal = obj.SeedInternal;
            newobj.MaxTimeInternal = obj.MaxTimeInternal;
            newobj.MaxNumIterationInternal = obj.MaxNumIterationInternal;
            
            newobj.CostFcn = obj.CostFcn;
            newobj.GradientFcn = obj.GradientFcn;
            newobj.SolutionEvaluationFcn = obj.SolutionEvaluationFcn;
            newobj.RandomSeedFcn = obj.RandomSeedFcn;
            newobj.BoundHandlingFcn = obj.BoundHandlingFcn;
            
            newobj.ExtraArgs = obj.ExtraArgs;
            
            newobj.setSolverParams(obj.getSolverParams);
            
            if obj.UseTimer
                newobj.TimeObj = obj.TimeObj;
                newobj.TimeObjInternal = obj.TimeObjInternal;
            end
        end
        
    end
    
    methods (Access = {?TrajectoryOptimizer.NLPSolverInterface}) %, ...
                       %?matlab.unittest.TestCase
        
        function [xSol, exitFlag, err, iter] = solveInternal(obj)
            
            x = obj.SeedInternal;
            
            if obj.UseTimer
                obj.TimeObjInternal.reset();
            end
            
            % Dimension
            n = size(x,1);
            
            [cost, ~, ~, obj.ExtraArgs] = obj.CostFcn(x, obj.ExtraArgs);
            grad = obj.GradientFcn(x, obj.ExtraArgs);
            grad = grad(:);

            
            
            % H starts as a positive-definite matrix (so symmetric)
            H = eye(n);
            
            if obj.ConstraintsOn
                % Identify active-set (constraints that hit equality)
                activeSet = obj.ConstraintMatrix'*x >= obj.ConstraintBound;
                A = obj.ConstraintMatrix(:, activeSet);
            else
                activeSet = false(size(obj.ConstraintBound));
                A = zeros(n,0);
            end

            % Project Hessian Inverse to the valid manifold using a way 
            % similar to Gram-Schmidt process
            % This ensures that the H update does not violate constraints
            for k = 1:size(A,2)
                a = A(:,k);
                rho = a'*H*a; % a scalar
                H = H - (1/rho)*H*(a*a')*H;
            end
            
            xNew = x;
            iter = 0; 

            if any(isnan(cost)) || any(isnan(grad))
                fprintf("Objective should not be NaN\n")
                xSol = x;
                exitFlag = 3;
                err = realmax;
                return
            end
            
            % Main loop
            for i = 1:obj.MaxNumIterationInternal
                
                if obj.UseTimer
                    if timeLimitExceeded(obj, obj.TimeObjInternal.getElapsedTime)
                        xSol = x;
                        exitFlag = 1;
                        err = obj.SolutionEvaluationFcn(xSol, obj.ExtraArgs);
                        iter = i;
                        return 
                    end
                end
                
                if isempty(A)
                    alpha = 0;
                else
                    alpha = (A'*A)\A'*grad; % alpha is an indicator
                end
                
                Hg = H*grad;
                if atLocalMinimum(obj, Hg, alpha) 
                    % Evaluate found local minimum
                    xSol = x;
                    exitFlag = 0;
                    err = obj.SolutionEvaluationFcn(xSol, obj.ExtraArgs);
                    iter = i;
                    return;
                end
                
                
                if obj.ConstraintsOn && ~isempty(A)
                    B = inv(A'*A);
                    [threshold, p] = max(alpha./sqrt(diag(B))); 
                    
                    % If the norm of the modified gradient Hg is smaller than
                    % threshold, drop the constraint corresponding to the
                    % threshold value.
                    if norm(Hg) < 0.5*threshold
                         % Find the constraint to drop
                         activeConstraintIndices = find(activeSet);
                         idxp = activeConstraintIndices(p);

                         % Update active constraints A
                         activeSet(idxp) = false;
                         A = obj.ConstraintMatrix(:, activeSet);

                         % Compute Projection 
                         P = eye(n) - A*((A'*A)\A');

                         % Update H, allowing more flexibility in update
                         % direction
                         ap = obj.ConstraintMatrix(:, idxp);
                         H = H + (1/(ap'*P*ap))*P*(ap*ap')*P;

                         continue
                    end
                end
                
                % Take a test step, then screen current inactive constraints 
                % and see if any becomes active, and determine the step 
                % scaling factor upper bound lambda 
                % s.t. ai'*(x + lambdai*s) - bi <=0 for all i
                % lambdai needs to be positive
                
                s = - Hg;
                idxl = -1;
                if obj.ConstraintsOn ...
                        && any(~activeSet) % if there is inactive constraints

                    bIn = obj.ConstraintBound(~activeSet);
                    AIn = obj.ConstraintMatrix(:,~activeSet);
                    inactiveConstraintIndices = find(~activeSet);
                    lambdas = (bIn - AIn'*x)./(AIn'*s);
                   
                    % We don't care the case when ai'*x-bi < 0 and ai'*s<0
                    L = find(lambdas>0);
                    if ~isempty(L)
                        [lambda, l] = min(lambdas(lambdas>0));
                        idxl = inactiveConstraintIndices(L(l));
                    else
                        lambda = 0;
                    end
                    
                else
                    lambda = 0;
                end
                
                
                if lambda > 0
                    gamma = min(1, lambda);  
                else % When the test step does not hit any constraint
                    gamma = 1; 
                end
                
                s0 = s;
                
                % Line search
                
                beta = obj.ArmijoRuleBeta;
                sigma = obj.ArmijoRuleSigma;
                
                [costNew,~,~, obj.ExtraArgs] = obj.CostFcn(x+gamma*s0, obj.ExtraArgs);
                m = 0;
                while cost - costNew < -sigma * grad'*(gamma*s0)
                    
                    if stepSizeBelowMinimum(obj, gamma)
                        xSol = x;
                        exitFlag = 2;
                        err = obj.SolutionEvaluationFcn(xSol, obj.ExtraArgs);
                        iter = i;
                        return
                    end
                    
                    gamma = beta * gamma;
                    m = m+1;
                    [costNew,~,~, obj.ExtraArgs] = obj.CostFcn(x + gamma*s0, obj.ExtraArgs);
                end
                xNew = x + gamma*s0;
                gradNew = obj.GradientFcn(xNew, obj.ExtraArgs);
                gradNew = gradNew(:);
                
                % Case 1: gamma = 1 < lambda, xNew has not reached new
                % new constraint.
                % Case 2: gamma = 1 and lambda = 0, no new constraints.
                % In both cases, H is updated in an unconstrained
                % manner using damped BFGS
                % When gamma == lambda, H is projected to the updated
                % constrained manifold as new constraint is activated.
                
                if m==0 && abs(gamma - lambda)< sqrt(eps) % ?
                    a = obj.ConstraintMatrix(:,idxl);
                    
                    activeSet(idxl) = true;
                    A = obj.ConstraintMatrix(:, activeSet);
                    
                    % Project the search direction to constrained manifold
                    H = H - (1/(a'*H*a))*(H*(a*a'*H));
                else
                    % Update search direction using damped BFGS
                    y = gradNew - grad;
                    
                    d1 = 0.2;
                    d2 = 0.8;
                    
                    % Hessian damping formulation 1 (update s)
                    if s'*y < d1*y'*H*y
                        theta = d2*y'*H*y/(y'*H*y - s'*y);
                    else
                        theta = 1; 
                    end
                    sNew = theta*s + (1-theta)*H*y;
                    rho = sNew'*y;
                    V = eye(n)- (sNew*y')/rho;
                    H = V*H*V' + (sNew*sNew')/rho;
                    
                    % % Hessian damping formula 2 (update y)
                    % if s'*y < d1*s'*H*s
                    % 
                    %     th = d2*(s'*H*s)/(s'*H*s - s'*y);
                    %     y = th*y + (1 - th)*H*s;
                    % end
                    % 
                    % H = (eye(n) - (s*y')/(y'*s))*H*(eye(n)- (y*s')/(y'*s)) + s*s'/(y'*s);
                    
                    
                    % Making sure H is not non-PSD
                    if ~robotics.core.internal.isPositiveDefinite(H + sqrt(eps)*eye(size(H))) 
                        xSol = xNew;
                        exitFlag = 3;
                        err = obj.SolutionEvaluationFcn(xSol, obj.ExtraArgs);
                        iter = i;
                        return
                    end
                end
                
                if searchDirectionInvalid(obj, xNew)
                    % This really should not happen, just in case
                    xSol = x;
                    exitFlag = 4;
                    err = obj.SolutionEvaluationFcn(xSol, obj.ExtraArgs);
                    iter = i;
                    return
                end
                
                x = xNew;
                grad = gradNew;
                cost = costNew;
            end
            
            xSol = xNew;
            exitFlag = 5;
            err = obj.SolutionEvaluationFcn(xSol, obj.ExtraArgs);
            iter = obj.MaxNumIterationInternal;
        end
        
        function flag = atLocalMinimum(obj, Hg, alpha)
            flag = norm(Hg)<obj.GradientTolerance && all(alpha<=0);
        end
        
        function flag = searchDirectionInvalid(obj, xNew)
            flag = obj.ConstraintsOn && any(obj.ConstraintMatrix'*xNew - obj.ConstraintBound > sqrt(eps));
        end
        
        function flag = stepSizeBelowMinimum(obj, gamma)
            flag = gamma < obj.StepTolerance;
        end
        
    end
    
    methods (Static, Hidden)
        function props = matlabCodegenNontunableProperties(~)
            props = {'MaxNumConstraints', 'MaxNumVariables','UseTimer'};
        end
    end
end