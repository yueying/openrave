#!/usr/bin/env python
# Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from optparse import OptionParser
import time
from openravepy import __build_doc__
if not __build_doc__:
    from numpy import *
    from openravepy import *
else:
    from openravepy import with_destroy

@with_destroy
def run(args=None):
    """Executes the collision example
    
    :type args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Example shows how to query collision detection information using openravepy')
    OpenRAVEGlobalArguments.addOptions(parser)
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    env.Load('data/pr2test1.env.xml')
    robot=env.GetRobots()[0]
    raw_input('press key to show at least one contact point')

    with env:
        # move both arms to collision
        lindex = robot.GetJoint('l_shoulder_pan_joint').GetDOFIndex()
        rindex = robot.GetJoint('r_shoulder_pan_joint').GetDOFIndex()
        robot.SetDOFValues([0.226,-1.058],[lindex,rindex])

        # setup the collision checker to return contacts
        env.GetCollisionChecker().SetCollisionOptions(CollisionOptions.Contacts)

        # get first collision
        report = CollisionReport()
        collision=env.CheckCollision(robot,report=report)
        print '%d contacts'%len(report.contacts)
        positions = [c.pos for c in report.contacts]

    h1=env.plot3(array(positions),20)

    raw_input('press key to show collisions with links')

    with env:
        # collisions with individual links
        positions = []
        for link in robot.GetLinks():
            collision=env.CheckCollision(link,report=report)
            if len(report.contacts) > 0:
                print 'link %s %d contacts'%(link.GetName(),len(report.contacts))
                positions += [c.pos for c in report.contacts]

    h2=env.plot3(array(positions),20)

    raw_input('press any key to exit')


if __name__ == "__main__":
    run()
