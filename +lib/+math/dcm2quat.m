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

function q = dcm2quat(R)
q = zeros(1,4);
t = trace(R);
if (t > 0) 
    t = sqrt(1 + t);
    q(1) = 0.5 * t;
    t = 0.5 / t;
    q(2) = (R(3,2) - R(2,3)) * t;
    q(3) = (R(1,3) - R(3,1)) * t;
    q(4) = (R(2,1) - R(1,2)) * t;
elseif (R(1,1) > R(2,2) && R(1,1) > R(3,3))
    t = sqrt(1 + R(1,1) - R(2,2) - R(3,3));
    q(2) = 0.5 * t;
    t = 0.5 / t;
    q(1) = (R(3,2) - R(2,3)) * t;
    q(3) = (R(2,1) + R(1,2)) * t;
    q(4) = (R(1,3) + R(3,1)) * t;
elseif (R(2,2) > R(3,3))
    t = sqrt(1 - R(1,1) + R(2,2) - R(3,3));
    q(3) = 0.5 * t;
    t = 0.5 / t;
    q(1) = (R(1,3) - R(3,1)) * t;
    q(2) = (R(2,1) + R(1,2)) * t;
    q(4) = (R(3,2) + R(2,3)) * t;
else
    t = sqrt(1 - R(1,1) - R(2,2) + R(3,3));
    q(4) = 0.5 * t;
    t = 0.5 / t;
    q(1) = (R(2,1) - R(1,2)) * t;
    q(2) = (R(1,3) + R(3,1)) * t;
    q(3) = (R(3,2) + R(2,3)) * t;
end
q = lib.math.get_cannonical_quat(q);
end