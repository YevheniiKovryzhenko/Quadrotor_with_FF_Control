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

classdef trajectory_ID < uint32
   enumeration
       DEFAULT (0)       
       line_x_1 (1)
       line_y_1 (2)
       line_z_1 (3)
       line_xy_1 (4)
       line_xy_2 (5)
       line_xyz_1 (6)
       line_xyz_2 (7)
       yaw_1 (8)
       fig_8_1 (9)
       fig_8_1_slow (10)
       fig_8_2_slow (11)
       fig_8_yaw_1_slow (12)
       fig_8_yaw_2_slow (13)
       fig_8_yaw_1 (14)
       fig_8_yaw_2 (15)
       rrt_test_1 (16)
       square_1 (17)
       square_1_slow (18)
       square_2 (19)
       circle_1 (20)
       spiral_1 (21)
       spiral_yaw_1 (22)
       non_rest2rest(23)
       collision_avoidance_loiter_1(24)
       line_y_obst_1 (25)
   end
end