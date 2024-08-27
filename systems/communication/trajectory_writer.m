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

classdef trajectory_writer
    properties (Constant)
        n_int_max = 50;
        n_coeffs_max = 10;
        n_dofs_max = 4;
    end
    
    properties
        n_coeffs (1,1) uint8 {mustBeLessThanOrEqual(n_coeffs,10)}
        n_int (1,1) uint8 {mustBeLessThanOrEqual(n_int,50)}
        n_dofs (1,1) uint8 {mustBeLessThanOrEqual(n_dofs,4)}
        
        t_int single
        coeffs single
        file_name
        fileID
    end

    methods (Access = public)
        function this = trajectory_writer(res)
            [n_int_, n_coeffs_, n_dofs_] = size(res.pp);
            this.n_coeffs = uint8(n_coeffs_);
            this.n_int = uint8(n_int_);
            this.n_dofs = uint8(n_dofs_);

            this.coeffs = single(res.pp);
            this.t_int = single(diff(res.T));
        end

        function write(this, new_file_name)
            this = this.open(new_file_name);

            try
                this.write_header();
                for i_int = 1:this.n_int
                    t_int_i = this.t_int(i_int);
                    for i_dof = 1:this.n_dofs
                        i_coeffs = squeeze(this.coeffs(i_int, 1:this.n_coeffs, i_dof));
                        this.write_data(i_int-1, i_dof-1, t_int_i, i_coeffs);
                    end
                end
            catch
                fprintf("ERROR: Failed to save trajectory.\n");
            end
            this.close();
        end
    end
    
    methods (Access = private)
        

        function this = open(this, name_str)
            this.fileID = fopen(name_str,'w');
        end

        function this = close(this)
            fclose(this.fileID);
        end
        
        function this = write_header(this)
            header = uint8([this.n_coeffs, this.n_int, this.n_dofs]);
            fwrite(this.fileID, header,"uint8");
        end        

        function this = write_data(this, ...
                i_int, i_dof, t_int, coeffs)
            data = uint8([ ...
                i_int, ...
                i_dof, ...
                typecast(single(t_int), 'uint8'), ...
                typecast(single(coeffs), 'uint8')]);
            fwrite(this.fileID, data,"uint8");
        end
    end
end