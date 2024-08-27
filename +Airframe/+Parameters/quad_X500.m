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

function Const = quad_X500
%% This file contanis all the measured parameters of a specific vehicle %%
% 450mm X-frame large drone with rc_pilot/BBB, 920KV motors and 4S battery
Const.m             = 1.805;                      %quad mass in kg = frame + batt.
Const.mw            = 10/1000+8.22/1000;            %mass of prop + motor in kg

% Const.Iw           = 10/1000*(0.130/2)^2 + 8.22/1000*(27.65/2000)^2;  %moment of inertia of propellers and motors=M_prop*R_prop^2
Const.Iw            = 4.3821e-05;

% Const.I = eye(3)*1E-3;
Const.I             = [...
		15010373.22,		6112.20,        1310.42;
		6112.20,            24426054.77,    4892.30;
		1310.42,            4892.30,		14549231.17].*1E-9; %kg * m^2
Const.Idot          = zeros(3);

Const.MX2T              = 1/(190 *1.0e-3);                  % 1/roll moment arm (1/m)
Const.MY2T              = 1/(190 *1.0e-3);                  % 1/pitch moment arm (1/m)

Const.RADPS2T           = 1.021656497830555e-05;        %[N-s^2/rad^2] thrust = b*(rad/s)^2 thrust curve constant 
Const.RADPS2M           = 1.385995107908286e-07;        %[N-m-s^2/rad^2] torque = d*(rad/s)^2 moment curve constant
Const.MZ2T              = Const.RADPS2T / Const.RADPS2M;%                  

Const.RPM_max       = 10269.0;                       %max RPM
Const.RadPS_max     = Const.RPM_max*pi/30;          %max Rad/s
Const.motor_time_constant_s = 0.05;%/5; %electric motor time constant

Const.dof           = 4;
Const.ThrustMap     = 4;
Const.ControlConfig = 1;

Const.MAX_THRUST_N              = Const.RADPS2T*Const.RadPS_max^2; %N
Const.MAX_TORQUE_ROLL_NM        = Const.MAX_THRUST_N / Const.MX2T; %N-m
Const.MAX_TORQUE_PITCH_NM       = Const.MAX_THRUST_N / Const.MY2T; %N-m
Const.MAX_TORQUE_YAW_NM         = Const.RADPS2M*Const.RadPS_max^2; %N-m
% Const.Tsqrt2CMD                 = 1/(sqrt(Const.RADPS2T)*Const.RadPS_max);

% Const.MAX_ROLL_REF          = 60*pi/180;
% Const.MAX_PITCH_REF         = 60*pi/180;
% Const.MAX_ROLL_COMPONENT	= 0.4;
% Const.MAX_PITCH_COMPONENT   = 0.4;
% Const.MAX_YAW_COMPONENT     = 0.25;
% Const.MAX_X_COMPONENT       = 1.0;
% Const.MAX_Y_COMPONENT       = 1.0;
% Const.MAX_THRUST_COMPONENT  = -0.05;
% Const.MIN_THRUST_COMPONENT  = -0.75;

%Aerodynamic model:
Const.Cd_lumped = diag([0.04,0.04, 0]); %lumped drag coefficient

Const.D = diag([0.544, 0.386, 0]*0); %[1/s]

Const.A = zeros(3,3);
Const.B = zeros(3,3);

Const.k_h = zeros(1,1);
% Const.k_h = 0.009; %[1/m]
end