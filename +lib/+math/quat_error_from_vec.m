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

function q = quat_error_from_vec(src, dst)
    eps = 1e-5;
    q = zeros(1,4);
    cr = cross(src, dst);
    dt = dot(src,dst);
    if (norm(cr) < eps && dt < 0) 
        % // handle corner cases with 180 degree rotations
        % // if the two vectors are parallel, cross product is zero
        % // if they point opposite, the dot product is negative
        cr = abs(src);
        if (cr(1) < cr(2)) 
            if (cr(1) < cr(3)) 
                cr(:) = [1, 0, 0];
            else 
                cr(:) = [0, 0, 1];
            end         
        else 
            if (cr(2) < cr(3))
                cr(:) = [0, 1, 0];
            else
                cr(:) = [0, 0, 1];
            end
        end
        q(1) = 0;
        cr = cross(src,cr);     
    else 
        % // normal case, do half-way quaternion solution
        q(1) = dt + sqrt(norm(src)^2 * norm(dst)^2);
    end
    q(2) = cr(1);
    q(3) = cr(2);
    q(4) = cr(3);
    q = q / norm(q);
end