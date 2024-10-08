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

function [ hex ] = rgb2hex(rgb)
% rgb2hex converts rgb color values to hex color format. 
% 
% This function assumes rgb values are in [r g b] format on the 0 to 1
% scale.  If, however, any value r, g, or b exceed 1, the function assumes
% [r g b] are scaled between 0 and 255. 
% 
% * * * * * * * * * * * * * * * * * * * * 
% SYNTAX:
% hex = rgb2hex(rgb) returns the hexadecimal color value of the n x 3 rgb
%                    values. rgb can be an array. 
% 
% * * * * * * * * * * * * * * * * * * * * 
% EXAMPLES: 
% 
% myhexvalue = rgb2hex([0 1 0])
%    = #00FF00
% 
% myhexvalue = rgb2hex([0 255 0])
%    = #00FF00
% 
% myrgbvalues = [.2 .3 .4;
%                .5 .6 .7; 
%                .8 .6 .2;
%                .2 .2 .9];
% myhexvalues = rgb2hex(myrgbvalues) 
%    = #334D66
%      #8099B3
%      #CC9933
%      #3333E6
% 
% * * * * * * * * * * * * * * * * * * * * 
% Chad A. Greene, April 2014
% 
% Updated August 2014: Functionality remains exactly the same, but it's a
% little more efficient and more robust. Thanks to Stephen Cobeldick for
% his suggestions. 
% 
% * * * * * * * * * * * * * * * * * * * * 
% See also hex2rgb, dec2hex, hex2num, and ColorSpec. 
%% Check inputs: 
assert(nargin==1,'This function requires an RGB input.') 
assert(isnumeric(rgb)==1,'Function input must be numeric.') 
sizergb = size(rgb); 
assert(sizergb(2)==3,'rgb value must have three components in the form [r g b].')
assert(max(rgb(:))<=255& min(rgb(:))>=0,'rgb values must be on a scale of 0 to 1 or 0 to 255')
%% If no value in RGB exceeds unity, scale from 0 to 255: 
if max(rgb(:))<=1
    rgb = round(rgb*255); 
else
    rgb = round(rgb); 
end
%% Convert (Thanks to Stephen Cobeldick for this clever, efficient solution):
hex(:,2:7) = reshape(sprintf('%02X',rgb.'),6,[]).'; 
hex(:,1) = '#';
end