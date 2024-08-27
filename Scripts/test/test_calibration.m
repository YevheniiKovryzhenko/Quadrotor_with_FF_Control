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

test_calibration_(joystick_cal_file_str);

function test_calibration_(cal_file_str)
f = figure; f.Position = [300 500 100 100];
PushButton = uicontrol(gcf,'Style', 'togglebutton', 'String', 'Press When Gone','Position', [1 1 120 100],'CallBack', @PushB);
upate_rate_Hz = 300;

set(PushButton,"Value",1);
fprintf("Testing reading calibrated sticks...\n");
joy = lib.joystick.load_cal(cal_file_str);
set(PushButton,"Value",1);
while get(PushButton,"Value")
    joy = lib.joystick.read(joy);
    joy = lib.joystick.apply_cal(joy);
    fprintf("\r[%+10.8f",joy.sticks.cal.vals(1));
    for i = 2:joy.sticks.n
        fprintf(", %+10.8f",joy.sticks.cal.vals(i));
    end
    fprintf("]\t");
    fprintf("buttons=[%i",joy.buttons.raw(1));
    for i = 2:joy.buttons.n
        fprintf(", %i",joy.buttons.raw(i));
    end
    fprintf("]");
    pause(1/upate_rate_Hz);
end

close(f);
fprintf("\nDone!\n");
end

function PushB(source,event)
% fprintf("Value %i\n",source.Value);
end