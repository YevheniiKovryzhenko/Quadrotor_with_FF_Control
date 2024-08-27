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

function plot_box(center, width, parent)
%{
X
|
A--B    E--F
|up|    |dw|
C--D    G--H ---Y
%}

A = [center(1) + width(1)/2, center(2) + width(2)/2, center(3) + width(3)/2];
B = [center(1) + width(1)/2, center(2) - width(2)/2, center(3) + width(3)/2];
C = [center(1) - width(1)/2, center(2) + width(2)/2, center(3) + width(3)/2];
D = [center(1) - width(1)/2, center(2) - width(2)/2, center(3) + width(3)/2];

E = [center(1) + width(1)/2, center(2) + width(2)/2, center(3) - width(3)/2];
F = [center(1) + width(1)/2, center(2) - width(2)/2, center(3) - width(3)/2];
G = [center(1) - width(1)/2, center(2) + width(2)/2, center(3) - width(3)/2];
H = [center(1) - width(1)/2, center(2) - width(2)/2, center(3) - width(3)/2];

face = cell([1,6]);
face{1} = [...
    A
    B
    D
    C];

face{2} = [...
    A
    B
    F
    E];

face{3} = [...
    A
    C
    G
    E];

face{4} = [...
    B
    D
    H
    F];

face{5} = [...
    E
    F
    H
    G];

face{6} = [...
    C
    D
    H
    G];


color_RGB = [...
    0.325 0.325 0.85];
for i = 1:6
    fill3(face{i}(:,1),face{i}(:,2),face{i}(:,3),color_RGB,'Parent',parent);
end

end