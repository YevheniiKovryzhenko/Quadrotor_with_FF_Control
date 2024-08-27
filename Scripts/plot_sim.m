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

if exist('out','var') == 1
    if exist('obstacles','var') == 1
        plot_sim_(res, save_dir, out, obstacles, Parameters);
    else
        plot_sim_(res, save_dir, out, [], Parameters);
    end
end

function plot_sim_(res, save_dir, out, obstacles, Parameters)
    out = keep_AUTONOMOUS_only(out);
    % out = remove_datas(out, 500, 3900);
    % out = remove_datas(out, 150, 3250);
    % out = remove_datas(out, 5000, 41000);
    out = remove_offset_time(out);
    pr_color = 'b';
    col_tr = {'#D95319','#77AC30','#4DBEEE'}; %color order for 3D vectors
    plot_sim_res(res, pr_color, col_tr, save_dir, false, out, false, obstacles, Parameters);
end