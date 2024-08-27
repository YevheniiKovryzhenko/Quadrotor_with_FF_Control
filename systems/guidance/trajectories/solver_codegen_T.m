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

function [T, pp, n_dim, n_dim_ids, N_segments, offset,...
    Cost, ExitFlag, Iterations] = ...
    solver_codegen_T(wpts, tpts,...
    ShowDetails) %#codegen
    timeWt = 0;
    
    minSegmentTime = 0.1*diff(tpts);
    maxSegmentTime = tpts(end);
    
    wptFun = @(t) bcs_fun__(t, wpts);
    
    sol = TrajectoryOptimizer.solver_mexfriendy( ...
        tpts,...
        false,...
        ShowDetails, ...
        wptFun, ...
        [], ...
        minSegmentTime, ...
        maxSegmentTime, ...
        timeWt);
    
    stats = sol.get_stats;
    minsnap_set = sol.get_res;

    ExitFlag = stats.ExitFlag;
    Iterations = stats.Iterations;
    Cost = stats.J;

    pp = minsnap_set.pp;
    T = minsnap_set.T;
    n_dim = minsnap_set.n_dim;
    n_dim_src = minsnap_set.n_dim_src;
    n_dim_ids = minsnap_set.n_dim_ids;
    N_segments = minsnap_set.N_segments;
    offset = minsnap_set.offset;
    
    if ShowDetails
        for i_seg = 1:N_segments
            fprintf("Segment %i:\n", int32(i_seg));
            fprintf("T[%i]->T[%i] = %f s -> %f s\n", int32(i_seg), int32(i_seg+1), T(i_seg), T(i_seg+1));
            for i_dim = 1:n_dim
                fprintf("pp[dim=%i] = [ ",int32(i_dim))
                for i_coefs = 1:10
                    fprintf("%f ", pp(i_seg, i_coefs, i_dim))
                end
                fprintf("]\n")
            end   
            
            fprintf("\n")
        end

        for i_dim = 1:n_dim_src
            fprintf("offset[%i] = %f\n",int32(i_dim), offset(i_dim))
        end
    end
    
    function [wpts, extra_bcs] = bcs_fun__(~, wpts)
        extra_bcs = {};
    end
end