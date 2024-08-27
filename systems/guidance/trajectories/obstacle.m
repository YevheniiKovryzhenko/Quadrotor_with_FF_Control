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

classdef obstacle < handle %#codegen
    properties
        id (1,1) int64 = 0
        center (3,1) double = [0;0;0]
        width (3,1) double = [0;0;0]

        is_visible (1,1) logical = false
        was_processed (1,1) logical = false

        ignore_fl (1,1) logical = false
        color_RGB (1,3) double = [0.325 0.325 0.85]
        opacity_faces (1,1) double = 0.3
        opacity_edges (1,1) double = 0.3
    end
    methods

        function this_ = obstacle(varargin)
            narginchk(0,3)
            n = length(varargin);
            switch n 
                case 0
                   this_.ignore_fl = true; 
                case 3
                    this_.id = varargin{1};
                    this_.center(:) = varargin{2}(:);
                    this_.width(:) = varargin{3}(:);
                otherwise
                    error("Wrong argument list")
            end
            
        end

        function out = draw(this_, fig)
            if ~this_.ignore_fl
                out = this_.plot_box(this_.center, this_.width, fig, this_.color_RGB, this_.opacity_faces, this_.opacity_edges);
            else
                out = [];
            end
        end
        function out = draw_flip_z(this_, fig)
            if ~this_.ignore_fl
                out = this_.plot_box(this_.center.*[1;1;-1], this_.width, fig, this_.color_RGB, this_.opacity_faces, this_.opacity_edges);
            else
                out = [];
            end
            
        end
        
        function [is_interior] = check_if_obs_is_interior(this_, location, R) %this checks if a bounding box is within a bounding sphere around a location
            if this_.ignore_fl
                is_interior = false;
                return;
            end
            location_ = location(1:3);
            is_interior = this_.get_closest_dist(location_(:), 0) <= R;
        end

        function [dist, closest_pt, is_interior] = get_closest_dist(this_, org, tol)
            closest_pt = zeros(3,1);
            org_ = org(1:3);
            is_interior = this_.check_if_traj_is_interior(org_,tol);
            if is_interior
                dist = 0;
                closest_pt(:) = org_(:);
            else
                width_ = (this_.width + tol) / 2;
                for i = 1:3
                    if org_(i) >= width_(i) + this_.center(i)
                        closest_pt(i) = width_(i) + this_.center(i);
                    elseif org_(i) <= -width_(i) + this_.center(i)
                        closest_pt(i) = -width_(i) + this_.center(i);
                    else
                        closest_pt(i) = org_(i);
                    end
                end
                dist_vector = closest_pt - org_(:);
                dist = norm(dist_vector);
            end
        end

        function [dist, closest_pt, is_interior, is_exactly_in_center] = get_closest_dist2boundary(this_, org, tol) %does not work properly!
            closest_pt = zeros(3,1);
            org_ = org(1:3);
            width_ = (this_.width + tol) / 2;
            is_interior = this_.check_if_traj_is_interior(org_,tol);
            is_exactly_in_center = false;
            if ~is_interior
                for i = 1:3
                    if org_(i) >= width_(i) + this_.center(i)
                        closest_pt(i) = width_(i) + this_.center(i);
                    elseif org_(i) <= -width_(i) + this_.center(i)
                        closest_pt(i) = -width_(i) + this_.center(i);
                    else
                        closest_pt(i) = org_(i);
                    end
                end
                dist_vector = closest_pt - org_(:);
                dist = norm(dist_vector);
            else
                check_center_fl = false;   %should be true              
                is_exactly_in_center = (check_center_fl && all(org_(i) == this_.center));
                if is_exactly_in_center %this case is wrong
                    ind = 0;
                    min_dist = realmax;
                    for i = 1:3
                        if width_(i) < min_dist
                            min_dist = width_(i);
                            ind = i;
                        end
                    end

                    closest_pt(ind) = min_dist; %should be min_dist + center(ind)
                    dist = min_dist;
                else
                    ind = 0;
                    min_dist = realmax;
                    for i = 1:3
                        dist_left = org_(i) - (this_.center(i) - width_(i));
                        dist_right = (this_.center(i)+width_(i)) - org_(i);
    
                        if dist_left < dist_right
                            if abs(min_dist) > dist_left
                                min_dist = -dist_left;
                                ind = i; %should this be outside of if?
                            end
                        elseif dist_left == dist_right
                            continue
                        else
                            if abs(min_dist) > dist_right
                                min_dist = dist_right;
                                ind = i; %should this be outside of if?
                            end
                        end
                    end
                    closest_pt(:) = org_(:);
                    closest_pt(ind) = closest_pt(ind) + min_dist;
                    dist_vector = closest_pt - org_(:);
                    dist = norm(dist_vector);
                end                
            end
        end

        function [intercept_fl, dist, intercept_pt] = get_ray_dist(this_,ray_origin, ray_dir, tol)
            faces = this_.get_faces_box(this_.center, this_.width + tol);
            intercept_fl = false;
            dist = realmax;
            intercept_pt = zeros(3,1);
            for i = 1:length(faces)
                i_face_ = faces{i};
                pt_A_ = i_face_(1,:)';
                pt_B_ = i_face_(2,:)';
                % pt_D_ = i_face_(3,:)'; %should this be C? -no this is the correct order
                pt_C_ = i_face_(4,:)'; %should this be D?

                n_vec_ = cross(pt_B_-pt_A_, pt_C_-pt_A_);
                n_vec_ = n_vec_ / norm(n_vec_);

                dist_ = dot(n_vec_,(ray_origin(:) - pt_A_)) / dot(n_vec_,-ray_dir(:));
                intercept_pt_ = ray_origin(:) + ray_dir(:)*dist_;
                if(this_.check_if_traj_is_interior(intercept_pt_, tol))
                    if abs(dist_) < dist && dist_ > 0
                        intercept_fl = true;
                        dist = abs(dist_);
                        intercept_pt(:) = intercept_pt_(:);
                    end
                end
            end
        end

        function [is_interior] = check_if_traj_is_interior(this_, pts2check, tol) %this checks if a point is within a bounding box 
            [n_dim, n_pts] = size(pts2check);
            is_interior = false(1,n_pts);
            if this_.ignore_fl
                return;
            end
            pts = this_.get_points_box(this_.center, this_.width + tol);
            A = pts{1};
            H = pts{8};
            for ind = 1:n_pts
                is_interior(ind) = all(pts2check(1:3,ind) <= A(:),"all") && all(pts2check(1:3,ind) >= H(:),"all");
            end
        end

        

        function [is_interior, n_new_wpts, new_wpts, new_T] = check_traj(this_, T, pts2check, tol, push_away_dist)
            [is_interior] = this_.check_if_traj_is_interior(pts2check, tol);
            n_new_wpts = 0;
            new_wpts = [];       
            new_T = [];
            if this_.ignore_fl
                return;
            end
            
            if nnz(is_interior) > 0

                %count number of continuous penetrations
                was_interiror_ = false;
                for i = 1:length(is_interior)
                    if (is_interior(i))
                        if (~was_interiror_)
                            n_new_wpts = n_new_wpts + 1;
                        end
                        was_interiror_ = true;
                    else
                        was_interiror_ = false;
                    end
                end

                %now we find the best location outside the obstacle bound
                new_wpts = zeros(3, n_new_wpts);
                new_T = zeros(1, n_new_wpts);
                was_interiror_ = false;
                last_exterior_pt_ind_ = 0;
                % first_pt = zeros(3,1);
                % first_T = 0;
                
                % last_pt = zeros(3,1);
                % last_T = 0;
                ind__ = 0;
                % ind_key__ = find(is_interior);
                % first_key__ = 0;
                for i = 1:length(is_interior)
                    if (is_interior(i))
                        if (~was_interiror_)
                            % first_pt = pts2check(1:3,i);
                            last_exterior_pt_ind_ = i-1;
                            % first_T = T(i);                            
                        end
                        was_interiror_ = true;
                    else
                        if (was_interiror_)
                            if last_exterior_pt_ind_ == i-2 %only one point inside the boundary
                                [~, closest_pt] = this_.get_closest_dist2boundary(pts2check(1:3,last_exterior_pt_ind_), tol+push_away_dist);
                                ind__ = ind__ + 1;
                                new_wpts(1:3,ind__) = closest_pt(:);

                                % vect = first_pt - closest_pt;
                                % tmp_dir = vect / norm(vect);                                
                                % [~, dist] = this_.get_ray_dist(this_.center, tmp_dir, tol);
                                % new_wpts(1:3,ind__) =  tmp_dir*(dist + push_away_dist) + this_.center;
                                new_T(ind__) = (T(i) - T(last_exterior_pt_ind_))/2+T(last_exterior_pt_ind_); 
                                % if (first_T == last_T)
                                %     new_T(ind__) = first_T;
                                % else
                                %     new_T(ind__) = (last_T - first_T) / 2 + first_T;
                                % end
                            else % there are entry and exit points, let's pick the central pt place R away from the center
                                % [~, last_exterior_pt_boundary__] = this_.get_closest_dist2boundary(pts2check(1:3,last_exterior_pt_ind_), tol);
                                % [~, first_exterior_pt_boundary__] = this_.get_closest_dist2boundary(pts2check(1:3,i), tol);
                                last_exterior_pt__ = pts2check(1:3,last_exterior_pt_ind_);
                                first_exterior_pt__ = pts2check(1:3,i);
                                [~, pseudo_center__] = this_.get_closest_dist2boundary(pts2check(1:3,last_exterior_pt_ind_), 0);
                                % pseudo_center__ = (last_exterior_pt_boundary__ + first_exterior_pt_boundary__) / 2;
                                % pseudo_center__ = this_.center;

                                entry_pt_vect__ = last_exterior_pt__ - pseudo_center__;
                                exit_pt_vect__ = first_exterior_pt__ - pseudo_center__;
                                
                                entry_pt_dist = norm(entry_pt_vect__);
                                exit_pt_dist = norm(exit_pt_vect__);
                                
                                cut_plane_normal__ = cross(entry_pt_vect__ / entry_pt_dist, exit_pt_vect__ / exit_pt_dist);
                                if isnan(cut_plane_normal__)
                                    fprintf("Plane is undefined")
                                end
                                entry2exit_angle__ = acos(dot(entry_pt_vect__, exit_pt_vect__) / (entry_pt_dist*exit_pt_dist));
                                
                                if (nnz(cut_plane_normal__) < 1)
                                    cut_plane_normal__(3) = -1;
                                end
                                exit_midpoint_dir__ = axang2rotm([cut_plane_normal__(:)'/norm(cut_plane_normal__), entry2exit_angle__/2]) * entry_pt_vect__ / entry_pt_dist;
                                [~, dist2midpoint_boundary__] = this_.get_ray_dist(pseudo_center__, exit_midpoint_dir__, tol);
                                
                                ind__ = ind__ + 1;
                                new_wpts(1:3,ind__) =  exit_midpoint_dir__*(dist2midpoint_boundary__ + push_away_dist) + pseudo_center__;
                                % new_wpts(1:3,ind__) = tmp_dir * R + this_.center;

                                % [is_interior__] = this_.check_if_traj_is_interior(new_wpts(1:3,ind__), tol);
                                % if is_interior__
                                %     new_wpts(1:3,ind__) = this_.center + tmp_dir.*(this_.width/2 + tol);
                                % end
                                % new_T(ind__) = (last_T - first_T) / 2 + first_T;

                                new_T(ind__) = (T(i) - T(last_exterior_pt_ind_))/2+T(last_exterior_pt_ind_);
                            end
                        end
                        was_interiror_ = false;
                    end

                    % last_pt = pts2check(1:3,i);
                    % last_exterior_pt_ind = i;
                    % last_T = T(i);
                end
            end

        end


        function [new_wpts_merged, new_T_merged] = insert_new_wpts(this_, T_new_wpts, n_new_wpts, new_wpts, wpts, T, tol)
            [old_wpt_is_interior] = this_.check_if_traj_is_interior(wpts(1:3,2:end), tol);
            [n_dim_old, n_wpts_old] = size(wpts);
            n_wpts_merged = n_wpts_old + n_new_wpts - nnz(old_wpt_is_interior);
            new_wpts_merged_ = zeros(n_dim_old, n_wpts_merged);
            new_T_merged_ = zeros(1, n_wpts_merged);
            
            
            index_merged__ = 1;
            new_wpts_merged_(:,1) =  wpts(:,1);
            new_T_merged_(1) = T(1);
            
            v_old = zeros(1,n_wpts_old-1);
            for i = 1:n_wpts_old-1
                v_old(i) =  norm(wpts(1:3,i+1) - wpts(1:3,i)) / (T(i+1) - T(i));
            end
            
            cum_sum_T = 0;
            for i = 2:n_wpts_old
                insert_these_here__ = T(i-1) <= T_new_wpts & T_new_wpts <= T(i);
                if any(insert_these_here__,"all")
                    index_take__ = find(insert_these_here__);
                    for ii = 1:nnz(insert_these_here__)
                        index_merged__ = index_merged__ + 1;
                        new_wpts_merged_(1:3,index_merged__) =  new_wpts(1:3,index_take__(ii));
                        new_wpts_merged_(4, index_merged__) = new_wpts_merged_(4, index_merged__-1);

                        cum_sum_T = cum_sum_T + norm(new_wpts_merged_(1:3,index_merged__) - new_wpts_merged_(1:3,index_merged__-1))/v_old(i-1);
                        new_T_merged_(index_merged__) = cum_sum_T;
                    end
                end
                if ~old_wpt_is_interior(i-1)
                    index_merged__ = index_merged__ + 1;
                    new_wpts_merged_(:,index_merged__) = wpts(:,i);

                    cum_sum_T = cum_sum_T + norm(new_wpts_merged_(1:3,index_merged__) - new_wpts_merged_(1:3,index_merged__-1))/v_old(i-1);
                    new_T_merged_(index_merged__) = cum_sum_T;
                end
            end

            new_wpts_merged = new_wpts_merged_(:,1:index_merged__);
            new_T_merged = new_T_merged_(1:index_merged__);
        end
    end
    methods (Access = private)
        
    end
    
    methods (Static, Hidden)


        function pts = get_points_box(center, width)
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

            pts = {A, B, C, D, E, F, G, H};
        end

        function [faces, pts] = get_faces_box(center, width)
            %{
            X
            |
            A--B    E--F
            |up|    |dw|
            C--D    G--H ---Y
            %}
            
            pts = obstacle.get_points_box(center, width);
            A = pts{1};
            B = pts{2};
            C = pts{3};
            D = pts{4};
            E = pts{5};
            F = pts{6};
            G = pts{7};
            H = pts{8};

            faces = cell([1,6]);
            faces{1} = [...
                A
                B
                D
                C];
            
            faces{2} = [...
                A
                B
                F
                E];
            
            faces{3} = [...
                A
                C
                G
                E];
            
            faces{4} = [...
                B
                D
                H
                F];
            
            faces{5} = [...
                E
                F
                H
                G];
            
            faces{6} = [...
                C
                D
                H
                G];
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
        
        function out = plot_box(center, width, parent, color_RGB, opacity_faces, opacity_edges)
            faces = obstacle.get_faces_box(center, width);
            tmp1 = fill3(faces{1}(:,1),faces{1}(:,2),faces{1}(:,3),color_RGB,'Parent',parent,'FaceAlpha',opacity_faces, 'EdgeAlpha', opacity_edges);
            tmp2 = fill3(faces{2}(:,1),faces{2}(:,2),faces{2}(:,3),color_RGB,'Parent',parent,'FaceAlpha',opacity_faces, 'EdgeAlpha', opacity_edges);
            tmp3 = fill3(faces{3}(:,1),faces{3}(:,2),faces{3}(:,3),color_RGB,'Parent',parent,'FaceAlpha',opacity_faces, 'EdgeAlpha', opacity_edges);
            tmp4 = fill3(faces{4}(:,1),faces{4}(:,2),faces{4}(:,3),color_RGB,'Parent',parent,'FaceAlpha',opacity_faces, 'EdgeAlpha', opacity_edges);
            tmp5 = fill3(faces{5}(:,1),faces{5}(:,2),faces{5}(:,3),color_RGB,'Parent',parent,'FaceAlpha',opacity_faces, 'EdgeAlpha', opacity_edges);
            tmp6 = fill3(faces{6}(:,1),faces{6}(:,2),faces{6}(:,3),color_RGB,'Parent',parent,'FaceAlpha',opacity_faces, 'EdgeAlpha', opacity_edges);
            out = [tmp1, tmp2, tmp3, tmp4, tmp5, tmp6];          
        end
    end
end