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

classdef NLPSolverInterface < handle
    %NLPSOLVERINTERFACE Base class for RST internal nonlinear program solvers
    %   This is the solver interface for problems shown below (nonlinear costs
    %   with linear constraints)
    %   
    %   min F(x)
    %    x
    %   s.t. 
    %       Ax - b <= 0
    
    %   Copyright 2016-2019 The MathWorks, Inc.
    
    %#codegen


    properties
        
        %Name Name of the solver
        Name
        
        %ConstraintMatrix
        ConstraintMatrix
        
        %ConstraintBound
        ConstraintBound
        
        %ConstraintsOn
        ConstraintsOn
        
        %CostFcn Function handle to compute cost
        CostFcn
    
        %GradientFcn Function handle to compute gradient
        GradientFcn
        
        %SolutionEvaluationFcn Function handle to evaluate found solution
        SolutionEvaluationFcn
        
        %SolutionTolerance
        SolutionTolerance
        
        %BoundHandlingFcn Function handle to deal with out-of-bound
        %   variables
        BoundHandlingFcn
        
        %RandomRestart
        RandomRestart
        
        %RandomSeedFcn Function handle to generate random initial guess
        %   (that satisfies constraints)
        RandomSeedFcn
        
        %ExtraArgs Extra arguments for function handles
        ExtraArgs
        
        %MaxNumIteration Maximum number of iterations
        MaxNumIteration
        
        %MaxTime Maximum solving time
        MaxTime

    end
    
    properties (Access = {?TrajectoryOptimizer.NLPSolverInterface})
        % Intermediate variables for internal use
        SeedInternal
        
        MaxTimeInternal
        
        MaxNumIterationInternal
        
        %StepTolerance Minimum step size
        StepTolerance
        
        %TimeObj An object of SystemTimeProvider, for timing in solve
        %   method
        TimeObj   
        
        %UseTimer Internal property to ensure System Time Provider is called only when needed
        UseTimer = true
    end
    
    
    methods (Abstract)
        
        params = getSolverParams(obj)
        
    end
    
    methods (Abstract, Access = {?TrajectoryOptimizer.NLPSolverInterface}) %, ...
                                 % ?matlab.unittest.TestCase
        
        [xSol, exitFlag, err, iter, XDebug] = solveInternal(obj);
        
    end
    
    methods
        
        function [xSol, solutionInfo] = solve(obj, seed)
            %solve
            obj.MaxNumIterationInternal = obj.MaxNumIteration;
            obj.MaxTimeInternal = obj.MaxTime;
            obj.SeedInternal = seed(:);
            tol = obj.SolutionTolerance; 
            
            if obj.UseTimer
                obj.TimeObj.reset();
            end
            
            [xSol, exitFlag, err, iter] = solveInternal(obj);
            rrAttempts = 0;
            
            iterations = iter;
            xSolPrev = xSol;
            errPrev = err;
            exitFlagPrev = exitFlag;
            
            while obj.RandomRestart && (err > tol)
                obj.MaxNumIterationInternal = obj.MaxNumIterationInternal - iter;
                
                if obj.UseTimer
                    obj.MaxTimeInternal = obj.MaxTime - obj.TimeObj.getElapsedTime;
                end
                
                % This check prevents solveInternal from being called with 
                % an iteration limit of 0, in the case where the previous
                % call to solveInternal terminated on the final
                % iteration, but for a reason other than hitting the
                % iteration limit.
                if obj.MaxNumIterationInternal <= 0
                    exitFlag = 5;%NLPSolverExitFlags.IterationLimitExceeded;
                end
                
                if exitFlag == 5 || exitFlag == 1%NLPSolverExitFlags.IterationLimitExceeded ...
                        %|| exitFlag == NLPSolverExitFlags.TimeLimitExceeded
                    % if time out or maximum iteration, then stop trying
                    exitFlagPrev = exitFlag;
                    break 
                end
                
                newseed = obj.RandomSeedFcn(obj.ExtraArgs);
                obj.SeedInternal(:) = newseed(:);
                [xSol, exitFlag, err, iter] = solveInternal(obj);
                
                % only keeping the best available solution
                if (err < errPrev)
                    xSolPrev = xSol;
                    errPrev = err;
                    exitFlagPrev = exitFlag;
                end
                
                rrAttempts = rrAttempts + 1;
                iterations = iterations + iter;
            end
            
            xSol = xSolPrev;
            err = errPrev;
            exitFlag = exitFlagPrev;
            
            solutionInfo.Iterations = iterations;
            solutionInfo.RRAttempts = rrAttempts;
            solutionInfo.Error = err;
            solutionInfo.ExitFlag = double(exitFlag);
            
            if err < tol
                solutionInfo.Status = 'success';
            else
                solutionInfo.Status = 'best available';
            end
        end
        
        function flag = timeLimitExceeded(obj, t)
            flag = t > obj.MaxTimeInternal;
        end
        
    end
    
end