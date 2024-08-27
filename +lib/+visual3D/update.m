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

function animation_3D = update(animation_3D, q, p)
q = q(:).';
p = p(:);

q(:) = q(:)./norm(q(:));

tmp = quat2axang(q);
a_vec = tmp(1:3);
a_vec([1,2]) = -a_vec([1,2]);
phi = tmp(4);

m = makehgtform( ...
    'translate',(p(1)), (p(2)), (p(3)),...
    'axisrotate',a_vec,phi,...
    'translate',-animation_3D.init.x(1),-animation_3D.init.y(1),-animation_3D.init.z(1));

animation_3D.object.Matrix = m;
% t.String = sprintf("    u=[%6.4f, %6.4f, %6.4f]E-7 N-m",u(i,1)*1E7, u(i,2)*1E7, u(i,3)*1E7);

end