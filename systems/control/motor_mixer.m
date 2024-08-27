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

function actuators  = motor_mixer(T, Tau, ...
    MX2T, MY2T, MZ2T, ...
    MAX_THRUST, MAX_TORQUE_ROLL, MAX_TORQUE_PITCH, MAX_TORQUE_YAW)
actuators = zeros(16,1);

%{
Quadrotor X:
3-cw    1-ccw
    X
    /\
    ||
    -Z ====>Y

2-ccw   4-cw
%}
motor_mixing_ = ...
    [...
    -1, -MX2T, MY2T, MZ2T
    -1, MX2T, -MY2T, MZ2T
    -1, MX2T, MY2T, -MZ2T
    -1, -MX2T, -MY2T, -MZ2T];

nMotors = 4;
nDOFs = 4;

actuators(1:nMotors) = mix_motors_(T, Tau(:), motor_mixing_, ...
    MAX_THRUST, MAX_TORQUE_ROLL, MAX_TORQUE_PITCH, MAX_TORQUE_YAW,...
    nMotors, nDOFs);
end

function out = mix_motors_(T, Tau, motor_mixing,...
    MAX_THRUST, MAX_TORQUE_ROLL, MAX_TORQUE_PITCH, MAX_TORQUE_YAW, ...
    nMotors, nDOFs)

    THRUST_IND  = 1;
    ROLL_IND    = 2;
    PITCH_IND   = 3;
    YAW_IND     = 4;

    out = zeros(nMotors,1);
    nACT = zeros(1,4);
    for i_dof = 1:nDOFs
        nACT(i_dof) = nnz(motor_mixing(:,i_dof));
    end
    
    for i_actuator_ = 1:nMotors
        roll_force_ = 0;
        if nACT(2) > 0
            roll_force_ = motor_mixing(i_actuator_,ROLL_IND) * check_saturation__(Tau(1)/nACT(2), -MAX_TORQUE_ROLL, MAX_TORQUE_ROLL);
        end

        pitch_force_ = 0;
        if nACT(3) > 0
            pitch_force_ = motor_mixing(i_actuator_,PITCH_IND) * check_saturation__(Tau(2)/nACT(3), -MAX_TORQUE_PITCH, MAX_TORQUE_PITCH);
        end

        yaw_force_ = 0;
        if nACT(4) > 0
            yaw_force_ = motor_mixing(i_actuator_,YAW_IND) * check_saturation__(Tau(3)/nACT(4), -MAX_TORQUE_YAW, MAX_TORQUE_YAW);
        end

        thrust_force_ = 0;
        if nACT(1) > 0
            thrust_force_ = -motor_mixing(i_actuator_,THRUST_IND) * T / nACT(1);
        end
        
        out(i_actuator_) = check_saturation__(...
            thrust_force_ + roll_force_ + pitch_force_ + yaw_force_,...
            0, MAX_THRUST) / MAX_THRUST;
    end
end


function [out, saturated] = check_saturation__(in, out_min, out_max)        
    in_ = in(1);
    saturated = false;
    if in_ > out_max
        out = out_max;
        saturated = true;
        return;
    elseif in_ < out_min
        out = out_min;
        saturated = true;
        return;
    else
        out = in_;
        return;
    end
end