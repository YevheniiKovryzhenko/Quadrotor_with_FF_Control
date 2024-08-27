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

function subfolder_str = save_fig(FigName, subfolder_str, FigHandle)
if ~exist(subfolder_str, 'dir')
   mkdir(subfolder_str)
end
PNG_subfolder_str = fullfile(subfolder_str, "/PNG/");
if ~exist(PNG_subfolder_str, 'dir')
   mkdir(PNG_subfolder_str)
end
FIG_subfolder_str = fullfile(subfolder_str, "/FIG/");
if ~exist(FIG_subfolder_str, 'dir')
   mkdir(FIG_subfolder_str)
end
EPS_subfolder_str = fullfile(subfolder_str, "/EPS/");
if ~exist(EPS_subfolder_str, 'dir')
   mkdir(EPS_subfolder_str)
end


% FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
% for iFig = 1:length(FigList)
  % FigHandle = FigList(iFig);
  set(0, 'CurrentFigure', FigHandle);
  savefig(fullfile(FIG_subfolder_str, [FigName '.fig']));
  saveas(FigHandle,fullfile(PNG_subfolder_str, [FigName '.jpg']));
  saveas(FigHandle,fullfile(EPS_subfolder_str, [FigName '.eps']),'epsc');
% end
end