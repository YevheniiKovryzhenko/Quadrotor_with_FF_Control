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

function out = remove_datas(in,n_start_idsremove, n_end_idsremove)
    if class(in) == "Simulink.SimulationOutput"
        fields = in.who;
    elseif class(in) == "struct"
        fields = fieldnames(in);
    else
        fprintf("ERROR: unknown input type!\n")
        out = in;
        return
    end
    
    out = in;
    for i = 1:length(fields)
        tmp = in.(fields{i});
        if fields{i} == "traj"
            out.traj.time_since_start = [];
            out.traj.T = [];
            out.traj.pp = [];
            out.traj.n_dim = [];
            out.traj.n_dim_src = [];
            out.traj.n_dim_ids = [];
            out.traj.N_segments = [];
            out.traj.state_0 = [];
            out.traj.wpts = [];

            out.traj.time_since_start.Data = tmp.time_since_start.Data(n_start_idsremove:end-n_end_idsremove,1);
            out.traj.T.Data = tmp.T.Data(n_start_idsremove:end-n_end_idsremove);
            out.traj.pp.Data = tmp.pp.Data(n_start_idsremove:end-n_end_idsremove);
            out.traj.n_dim.Data = tmp.n_dim.Data(n_start_idsremove:end-n_end_idsremove);
            out.traj.n_dim_src.Data = tmp.n_dim_src.Data(n_start_idsremove:end-n_end_idsremove);
            out.traj.n_dim_ids.Data = tmp.n_dim_ids.Data(n_start_idsremove:end-n_end_idsremove);
            out.traj.N_segments.Data = tmp.N_segments.Data(n_start_idsremove:end-n_end_idsremove);
            out.traj.state_0.Data = tmp.state_0.Data(n_start_idsremove:end-n_end_idsremove,:);
            out.traj.wpts.Data = tmp.wpts.Data(n_start_idsremove:end-n_end_idsremove);
        else        
            out.(fields{i}) = tmp(n_start_idsremove:end-n_end_idsremove,:);
        end
    end
end