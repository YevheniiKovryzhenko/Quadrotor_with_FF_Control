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

function data = get_control_data_from_sim(sim_data_out)
    data.t = sim_data_out.t_s(:)';

    %%position, velocity and acceleration control data:
    data.x = sim_data_out.p_e(:,1)';
    data.y = sim_data_out.p_e(:,2)';
    data.z = sim_data_out.p_e(:,3)';
    
    data.vx = sim_data_out.v_e(:,1)';
    data.vy = sim_data_out.v_e(:,2)';
    data.vz = sim_data_out.v_e(:,3)';
    
    data.ax = sim_data_out.a_e(:,1)';
    data.ay = sim_data_out.a_e(:,2)';
    data.az = sim_data_out.a_e(:,3)';
    
    data.x_ref = sim_data_out.p_e_ref(:,1)';
    data.y_ref = sim_data_out.p_e_ref(:,2)';
    data.z_ref = sim_data_out.p_e_ref(:,3)';
    
    data.vx_ref = sim_data_out.v_e_ref(:,1)';
    data.vy_ref = sim_data_out.v_e_ref(:,2)';
    data.vz_ref = sim_data_out.v_e_ref(:,3)';
    
    data.ax_ref = sim_data_out.a_e_ref(:,1)';
    data.ay_ref = sim_data_out.a_e_ref(:,2)';
    data.az_ref = sim_data_out.a_e_ref(:,3)';
    
    data.x_err = data.x_ref - data.x;
    data.y_err = data.y_ref - data.y;
    data.z_err = data.z_ref - data.z;
    
    data.vx_err = data.vx_ref - data.vx;
    data.vy_err = data.vy_ref - data.vy;
    data.vz_err = data.vz_ref - data.vz;
    
    data.ax_err = data.ax_ref - data.ax;
    data.ay_err = data.ay_ref - data.ay;
    data.az_err = data.az_ref - data.az;


    %% attitude (angle,rates) control
    q = sim_data_out.q;
    q_ref = sim_data_out.q_ref;
    n_datas = length(data.t);
    att_ref = zeros(n_datas,3);
    att = zeros(n_datas,3);
    for i = 1:n_datas
        att(i,:) = quat2eul(q(i,:),"XYZ");
        att_ref(i,:) = quat2eul(q_ref(i,:),"XYZ");
    end
    
    data.phi = att(:,1).';
    data.theta = att(:,2).';
    data.psi = att(:,3).';
    
    data.p = sim_data_out.omega(:,1).';
    data.q = sim_data_out.omega(:,2).';
    data.r = sim_data_out.omega(:,3).';
    
    data.phi_ref = att_ref(:,1).';
    data.theta_ref = att_ref(:,2).';
    data.psi_ref = att_ref(:,3).';
    
    data.p_ref = sim_data_out.omega_ref(:,1).';
    data.q_ref = sim_data_out.omega_ref(:,2).';
    data.r_ref = sim_data_out.omega_ref(:,3).';
    
    data.phi_err = lib.math.get_angle_err_vec(data.phi_ref, data.phi);
    data.theta_err = lib.math.get_angle_err_vec(data.theta_ref, data.theta);
    data.psi_err = lib.math.get_angle_err_vec(data.psi_ref, data.psi);
    
    data.p_err = data.p_ref - data.p;
    data.q_err = data.q_ref - data.q;
    data.r_err = data.r_ref - data.r;
end