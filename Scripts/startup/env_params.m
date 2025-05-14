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

% environment params

Parameters.g = 9.81;
Parameters.rho = 1.225;

Parameters.Simulink.EN_ACT_MODEL = true;
Parameters.Simulink.EN_ACT_DELAY_MODEL = true;

Parameters.Simulink.EN_AERO_MODEL = true;

%% Global Position:
%Auburn, Alabama:
Parameters.GlobalPosition.REF_LAT_DEG = 32.6099;
Parameters.GlobalPosition.REF_LONG_DEG = 85.4808;
Parameters.GlobalPosition.REF_ALT_M = 214;

%% Sensors:

%will need to make these part of Parameters.Sensors structure
ForwardVelocityCutoff = 3;
SensorAAFiltNum = 4.386e+06;
SensorAAFiltDen = [1 2.96e+03 4.386e+06];
ReferenceFilterNum = 0.04877;
ReferenceFilterDen = [1 -0.9512];

%% wind model:
Parameters.Simulink.EN_WIND_MODEL = true;

Parameters.Simulink.wind_dir_deg = 38; %wind dirrection at 6 m  (degrees clockwise from north)
Parameters.Simulink.wind_speed = 1; %wind speed at 6 m defines the low-altitide intensity (m/s)

%dryden wind turbulence model:
Parameters.Simulink.wind_scale_length = 100; %scale length at medium/high altitude(m)
Parameters.Simulink.wind_ref_wing_span = 0.3; %m
Parameters.Simulink.wind_noise = 0.1; %band limited noise and discrete filter sample time (sec)
Parameters.Simulink.wind_noise_seeds = [23341 23342 23343 23344]; %Noise seeds [ug vg wg pg]

%horizontal wind model:
Parameters.Simulink.wind_dir_alt_deg = 55;
Parameters.Simulink.wind_speed_alt = 3;