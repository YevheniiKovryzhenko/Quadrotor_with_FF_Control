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

function move_3D_multi(t_cell,p_cell,q_cell, ...
    scaling, write_gif, FolderName, fig, pause_dt)

n_objs = length(t_cell);

n = zeros(1,n_objs);
animation_3D = cell(1,n_objs);
for i = 1:n_objs
    n(i) = length(t_cell{i});
    % animation_3D{i} = lib.visual3D.create_body_NED(fig, p_cell{i}(:,1), scaling(i));
    animation_3D{i} = lib.visual3D.create_quadcopter_NED(fig, p_cell{i}(:,1), scaling(i));
end

ax = gca(fig);
view(ax,-45,20)

if write_gif
    time           = clock;
    FigNames_gif   = sprintf('%s/%d_%d_%d_%d_%d_%d.gif',FolderName,time(1),time(2),time(3),time(4),time(5),round(time(6)));
    if ~exist(FolderName, 'dir')
       mkdir(FolderName)
    end
    dt_gif = 0.02;
    
    lib.gif.start(fig,FigNames_gif,dt_gif)

end

max_n = max(n);
for i = 2:max_n
    for i_obj = 1:n_objs
        if i <= n(i_obj)
            animation_3D{i_obj} = lib.visual3D.update(animation_3D{i_obj},q_cell{i_obj}(:,i),p_cell{i_obj}(:,i));
        end
    end
    view(ax,-45 + (90)*(i/max_n),20 + (15)* (i/max_n));
    drawnow

    if write_gif        
        lib.gif.append(fig,FigNames_gif,dt_gif);
    end
    if pause_dt > 0 && ~write_gif
        pause(pause_dt)
    end
end
end