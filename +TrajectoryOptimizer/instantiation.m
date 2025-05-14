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
    CLASS: TrajectoryOptimizer.instantiation

    Purpose:
        Ensures that required symbolic computation files for trajectory
        optimization (e.g., get_A, get_Q_prime) are present. If missing,
        generates them using symbolic math and moves them to the appropriate
        directory.

    Attributes:
        - top_level_dir: Root directory for the project.
        - data_gen_fld: Directory for generated files.
        - file_list: List of required symbolic computation files.

    Usage:
        This class is intended to be used as a base for classes that require
        symbolic computation files. Instantiation checks for and generates
        required files as needed.
%}
classdef instantiation < TrajectoryOptimizer.common
    properties (Constant = true)
        top_level_dir = pwd; % Top-level directory of the project
        data_gen_fld = fullfile(TrajectoryOptimizer.instantiation.top_level_dir, "+TrajectoryOptimizer/"); % Directory for generated files
        file_list = { "get_A", "get_Q_prime" }; % List of required symbolic computation files
    end

    methods (Access = protected)
        %{
            FUNCTION: instantiation (constructor)

            Purpose:
                Constructor that checks for required symbolic computation
                files and generates them if missing.

            Usage:
                obj = TrajectoryOptimizer.instantiation();
        %}
        function this = instantiation()
            % Constructor checks for required files and generates them if missing
            this.check_files;
        end

        %{
            FUNCTION: check_files

            Purpose:
                Checks if all required symbolic computation files exist.
                If any are missing, triggers file generation.

            Usage:
                obj.check_files();
        %}
        function check_files(this)
            % Check if required files exist, generate them if missing
            for i = 1:length(this.file_list)
                if ~isfile(fullfile(this.data_gen_fld, sprintf("%s.m", this.file_list{i})))
                    this.gen_files;
                    return;
                end
            end
        end        
    end    

    methods (Hidden = true)
        %{
            FUNCTION: gen_files

            Purpose:
                Generates required symbolic computation files for trajectory
                optimization (e.g., get_A, get_Q_prime) using symbolic math.
                Moves generated files to the appropriate directory.

            Usage:
                obj.gen_files();

            Assumptions:
                - Symbolic Math Toolbox is available.
                - The directory structure allows file movement.
        %}
        function gen_files(this)
            % Generate required symbolic computation files
            n_dt = this.N_dt;
            min_dt = this.J_dt;
            N_COEFS = this.N_COEFS;

            syms t T
            assume(t, {'positive', 'real'});
            assume(T, {'positive', 'real'});            
            p = sym('p_', [N_COEFS, 1], 'real');
            
            % Polynomial definition
            P = p.' .* ((t / T).^(0:(N_COEFS-1)));
            
            % Derivatives and matrix computations
            Pdt = cell(1, n_dt);
            A = sym('A_', [n_dt, N_COEFS]);
            A(1,:) = jacobian(sum(P), p);
            for i_dt = 1:n_dt
                Pdt{i_dt} = diff(P, 't', i_dt);
                A(i_dt+1,:) = jacobian(sum(Pdt{i_dt}), p);
            end
            
            % Boundary conditions
            Ai = subs(A, 't', 0);
            Af = subs(A, 't', T);
            A = [Ai; Af];
            Ainv = inv(A);

            % Cost function and Hessian
            J_ = sum(diff(P, 't', min_dt))^2;            
            J = int(J_, 't', [0, T]);
            Q = 1/2 * hessian(J, p);
            Q_prime = Ainv' * Q * Ainv;
            
            % Generate MATLAB functions
            matlabFunction(A, 'File', this.file_list{1}, 'Vars', {T}, 'Sparse', true);
            matlabFunction(Q_prime, 'File', this.file_list{2}, 'Vars', {T}, 'Sparse', true);

            % Move generated files to the appropriate directory
            for i = 1:length(this.file_list)
                gen_file_str_i = this.file_list{i};
                fileList_ = dir(fullfile(this.top_level_dir, sprintf("%s*", gen_file_str_i)));
                for ii = 1:length(fileList_)
                    movefile(fileList_(ii).name, this.data_gen_fld, 'f');
                end
            end
        end
    end
end