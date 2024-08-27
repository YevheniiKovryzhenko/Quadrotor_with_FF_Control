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

function res = constrain2D(v0, v1, max)
res = zeros(2,1);
v0_ = [v0(1); v0(2)];
v1_ = [v1(1); v1(2)];

res(:) = v0_ + v1_;
if (norm(res) <= max)
    %vector does not exceed maximum magnitude
    return;
elseif norm(v0_) >=max
    %the magnitude along v0, which has priority, already exceeds maximum.
    res(:) = v0_ / norm(v0_)*max;
    return;
elseif norm(v1_ - v0_) < 0.001
    %the two vectors are equal
    res(:) = v0_ / norm(v0_)*max;
    return;
elseif norm(v0_) < 0.001
    %the first vector is 0.
    res(:) = v1_ / norm(v1_)*max;
	return;
else
    %{
    // vf = final vector with ||vf|| <= max
	// s = scaling factor
	// u1 = unit of v1
	// vf = v0 + v1 = v0 + s * u1
	// constraint: ||vf|| <= max
	//
	// solve for s: ||vf|| = ||v0 + s * u1|| <= max
	//
	// Derivation:
	// For simplicity, replace v0 -> v, u1 -> u
	// 				   		   v0(0/1/2) -> v0/1/2
	// 				   		   u1(0/1/2) -> u0/1/2
	//
	// ||v + s * u||^2 = (v0+s*u0)^2+(v1+s*u1)^2+(v2+s*u2)^2 = max^2
	// v0^2+2*s*u0*v0+s^2*u0^2 + v1^2+2*s*u1*v1+s^2*u1^2 + v2^2+2*s*u2*v2+s^2*u2^2 = max^2
	// s^2*(u0^2+u1^2+u2^2) + s*2*(u0*v0+u1*v1+u2*v2) + (v0^2+v1^2+v2^2-max^2) = 0
	//
	// quadratic equation:
	// -> s^2*a + s*b + c = 0 with solution: s1/2 = (-b +- sqrt(b^2 - 4*a*c))/(2*a)
	//
	// b = 2 * u.dot(v)
	// a = 1 (because u is normalized)
	// c = (v0^2+v1^2+v2^2-max^2) = -max^2 + ||v||^2
	//
	// sqrt(b^2 - 4*a*c) =
	// 		sqrt(4*u.dot(v)^2 - 4*(||v||^2 - max^2)) = 2*sqrt(u.dot(v)^2 +- (||v||^2 -max^2))
	//
	// s1/2 = ( -2*u.dot(v) +- 2*sqrt(u.dot(v)^2 - (||v||^2 -max^2)) / 2
	//      =  -u.dot(v) +- sqrt(u.dot(v)^2 - (||v||^2 -max^2))
	// m = u.dot(v)
	// s = -m + sqrt(m^2 - c)
	//
	//
	//
	// notes:
	// 	- s (=scaling factor) needs to be positive
	// 	- (max - ||v||) always larger than zero, otherwise it never entered this if-statement
	Vector2f u1 = v1.normalized();
	float m = u1.dot(v0);
	float c = v0.dot(v0) - max * max;
	float s = -m + sqrtf(m * m - c);
	return v0 + u1 * s;
    %}
    u1 = v1_ / norm(v1_);
    m = dot(u1, v0_);
    c = dot(v0_,v0_) - max*max;
    s = -m + sqrt(m*m - c);
    res(:) = v0_ + u1*s;
    return;
end

end