% success = RobotGoInitial(robot, home)

% Copyright (C) 2008-2010 Rosen Diankov
% 
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
function success = RobotGoInitial(robot, home)
global probs robothome

success = 0;
armjoints = robot.manips{robot.activemanip}.armjoints;
handjoints = robot.manips{robot.activemanip}.handjoints;
robotid = robot.id;

if( ~exist('home','var') )
    if( length(robothome) == robot.dof )
        home = robothome;
    else
        disp('robothome not set');
        home = zeros([robot.dof 1]);
    end
end

trajdata = orProblemSendCommand(['MoveManipulator execute 0 outputtraj goal ' sprintf('%f ', home(armjoints+1))], probs.manip);

if( isempty(trajdata) )
    return;
end

success = StartTrajectory(robotid, trajdata)
if( ~success )
    return;
end

disp('moving hand');
success = RobotMoveJointValues(robotid, home(handjoints+1),handjoints)

%orRobotSetActiveDOFs(robotid,0:(robot.dof-1));
%curvalues = orRobotGetDOFValues(robotid);
%orRobotStartActiveTrajectory(robotid,[curvalues home(:)]);
%WaitForRobot(robotid);
