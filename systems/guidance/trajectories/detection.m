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

classdef detection < handle %#codegen
    properties
        id (1,1) int64 = 0
        R (1,1) double = 1
        center (3,1) = [0;0;0]

        ignore_fl (1,1) logical = false
        color_RGB (1,3) double = [1.0 1.0 1.0]
        opacity_faces (1,1) double = 0.01
        opacity_edges (1,1) double = 0.05
        n_faces (1,1) double = 30;

    end
    methods 
        function this_ = detection(new_id, new_center, new_R)
            this_.id = new_id;
            this_.center(:) = new_center(:);
            this_.R = new_R;
        end

        function out = draw(this_, fig)
            if ~this_.ignore_fl
                out = this_.plot_sphere(this_.center, this_.R, fig, this_.n_faces, this_.color_RGB, this_.opacity_faces, this_.opacity_edges);
            else
                out = [];
            end
        end
        function out = draw_flip_z(this_, fig)
            if ~this_.ignore_fl
                out = this_.plot_sphere(this_.center.*[1;1;-1], this_.R, fig, this_.n_faces, this_.color_RGB, this_.opacity_faces, this_.opacity_edges);
            else
                out = [];
            end            
        end

        function draw_move(this_, obj)
            if ~this_.ignore_fl
                this_.draw_move_sphere(obj, this_.center, this_.R, this_.n_faces);
            end
        end
        function draw_move_flip_z(this_, obj)
            if ~this_.ignore_fl
                this_.draw_move_sphere(obj, this_.center.*[1;1;-1], this_.R, this_.n_faces);
            end
        end
    end
    methods (Access = private)
        
    end
    
    methods (Static, Hidden)


        function pts = get_points_sphere(center, R, n_faces)
            [range_sp_x, range_sp_y, range_sp_z] = sphere(n_faces);

            pts = {range_sp_x*R + center(1), range_sp_y*R + center(2), range_sp_z*R + center(3)};
        end

        function hide_obj_from_plot(obj)
            for i = 1:length(obj)
                set(obj(i),'FaceAlpha', 0.0);
                set(obj(i),'EdgeAlpha', 0.0);
            end
        end

        function show_obj_on_plot(obj, opacity_face, opacity_edge)
            for i = 1:length(obj)
                set(obj(i),'FaceAlpha', opacity_face);
                set(obj(i),'EdgeAlpha', opacity_edge);
            end
        end
        
        function out = plot_sphere(center, R, parent, n_faces, color_RGB, opacity_face, opacity_edge)
            pts = detection.get_points_sphere(center, R, n_faces);
            out = surf(parent, ...
                    pts{1},...
                    pts{2},...
                    pts{3},...
                    'FaceColor',color_RGB, ...
                    'FaceAlpha', opacity_face, 'EdgeAlpha', opacity_edge);
        end

        function draw_move_sphere(obj, center, R, n_faces)
            pts = detection.get_points_sphere(center, R, n_faces);
            set(obj,'XData', pts{1});
            set(obj,'YData', pts{2});
            set(obj,'ZData', pts{3});
        end
    end
end