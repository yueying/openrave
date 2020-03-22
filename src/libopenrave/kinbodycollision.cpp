// -*- coding: utf-8 -*-
// Copyright (C) 2006-2017 Rosen Diankov (rosen.diankov@gmail.com)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "libopenrave.h"
#include <openrave/grabbed.h>
#include <openrave/kinbody.h>
#include <openrave/numerical.h>

namespace OpenRAVE {

bool KinBody::CheckSelfCollision(CollisionReportPtr report, CollisionCheckerBasePtr collisionchecker) const
{
    if (!collisionchecker) 
	{
        collisionchecker = self_collision_checker_;
        if (!collisionchecker) 
		{
            collisionchecker = GetEnv()->GetCollisionChecker();
            if (!collisionchecker) 
			{
                // no checker set
                return false;
            }
        } else 
		{
            // have to set the same options as GetEnv()->GetCollisionChecker() since stuff like CO_ActiveDOFs is only set on the global checker
            collisionchecker->SetCollisionOptions(GetEnv()->GetCollisionChecker()->GetCollisionOptions());
        }
    }

    bool is_all_link_collisions = !!(collisionchecker->GetCollisionOptions() & CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if (!!report && is_all_link_collisions && report->nKeepPrevious == 0) 
	{
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }

    bool is_collision = false;
    if (collisionchecker->CheckStandaloneSelfCollision(shared_kinbody_const(), report)) 
	{
        if (!!report) {
            if (IS_DEBUGLEVEL(Level_Verbose)) {
                std::vector<OpenRAVE::dReal> v;
                GetDOFValues(v);
                stringstream ss;
                ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10 + 1);
                ss << "self collision report=" << report->__str__() << " ";
                for (size_t i = 0; i < v.size(); ++i) {
                    if (i > 0) {
                        ss << "," << v[i];
                    } else {
                        ss << "colvalues=[" << v[i];
                    }
                }
                ss << "]";
                RAVELOG_VERBOSE(ss.str());
            }
        }
        if (!is_all_link_collisions) { // if checking all collisions, have to continue
            return true;
        }

        is_collision = true;
    }

    // if collision checker is set to distance checking, have to compare reports for the minimum distance
    int coloptions = collisionchecker->GetCollisionOptions();
    CollisionReport tempreport;
    CollisionReportPtr pusereport = report;
    if (!!report && (coloptions & CO_Distance)) {
        pusereport = std::shared_ptr<CollisionReport>(&tempreport, utils::null_deleter());
    }

    // check all grabbed bodies with (TODO: support CO_ActiveDOFs option)
    FOREACH(itgrabbed, grabbed_bodies_)
    {
        GrabbedConstPtr pgrabbed = std::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
        KinBodyPtr pbody = pgrabbed->grabbed_body_.lock();
        if (!pbody) {
            RAVELOG_WARN_FORMAT("grabbed body on %s has already been destroyed, ignoring.", GetName());
            continue;
        }
        FOREACH(itrobotlink, pgrabbed->non_colliding_links_)
        {
            KinBody::LinkConstPtr robotlink = *itrobotlink;
            KinBodyPtr parentlink = (*itrobotlink)->GetParent(true);
            if (!parentlink) {
                RAVELOG_WARN_FORMAT("non_colliding_links_ has invalid link %s:%d", (*itrobotlink)->GetName() % (*itrobotlink)->GetIndex());
                robotlink = links_vector_.at((*itrobotlink)->GetIndex());
            }

            // have to use link/link collision since link/body checks attached bodies
            FOREACHC(itbodylink, pbody->GetLinks())
            {
                if (collisionchecker->CheckCollision(robotlink, KinBody::LinkConstPtr(*itbodylink), pusereport)) {
                    is_collision = true;
                    if (!is_all_link_collisions) { // if checking all collisions, have to continue
                        break;
                    }
                }
                if (!!pusereport && pusereport->minDistance < report->minDistance) {
                    *report = *pusereport;
                }
            }
            if (is_collision) {
                if (!is_all_link_collisions) { // if checking all collisions, have to continue
                    break;
                }
            }
        }
        if (is_collision) {
            if (!is_all_link_collisions) { // if checking all collisions, have to continue
                break;
            }
        }

        if (pbody->CheckSelfCollision(pusereport, collisionchecker)) {
            is_collision = true;
            if (!is_all_link_collisions) { // if checking all collisions, have to continue
                break;
            }
        }
        if (!!pusereport && pusereport->minDistance < report->minDistance) {
            *report = *pusereport;
        }

        // check attached bodies with each other, this is actually tricky since they are attached "with each other", so regular CheckCollision will not work.
        // Instead, we will compare each of the body's links with every other
        if (grabbed_bodies_.size() > 1) {
            FOREACHC(itgrabbed2, grabbed_bodies_)
            {
                GrabbedConstPtr pgrabbed2 = std::dynamic_pointer_cast<Grabbed const>(*itgrabbed2);
                KinBodyPtr pbody2 = pgrabbed2->grabbed_body_.lock();
                if (!pbody2) {
                    RAVELOG_WARN_FORMAT("grabbed body on %s has already been destroyed, so ignoring.", GetName());
                    continue;
                }
                if (pbody == pbody2) {
                    continue;
                }
                FOREACHC(itlink2, pbody2->GetLinks())
                {
                    // make sure the two bodies were not initially colliding
                    if (find(pgrabbed->non_colliding_links_.begin(), pgrabbed->non_colliding_links_.end(), *itlink2) != pgrabbed->non_colliding_links_.end()) {
                        FOREACHC(itlink, pbody->GetLinks())
                        {
                            if (find(pgrabbed2->non_colliding_links_.begin(), pgrabbed2->non_colliding_links_.end(), *itlink) != pgrabbed2->non_colliding_links_.end()) {
                                if (collisionchecker->CheckCollision(KinBody::LinkConstPtr(*itlink), KinBody::LinkConstPtr(*itlink2), pusereport)) {
                                    is_collision = true;
                                    if (!is_all_link_collisions) { // if checking all collisions, have to continue
                                        break;
                                    }
                                }
                                if (!!pusereport && pusereport->minDistance < report->minDistance) {
                                    *report = *pusereport;
                                }
                            }
                            if (is_collision) {
                                if (!is_all_link_collisions) { // if checking all collisions, have to continue
                                    break;
                                }
                            }
                        }
                        if (is_collision) {
                            if (!is_all_link_collisions) { // if checking all collisions, have to continue
                                break;
                            }
                        }
                    }
                }
                if (is_collision) {
                    if (!is_all_link_collisions) { // if checking all collisions, have to continue
                        break;
                    }
                }
            }
            if (is_collision) {
                if (!is_all_link_collisions) { // if checking all collisions, have to continue
                    break;
                }
            }
        }
    }

    if (is_collision && !!report) {
        if (report != pusereport) {
            *report = *pusereport;
        }
        if (IS_DEBUGLEVEL(Level_Verbose)) {
            std::vector<OpenRAVE::dReal> v;
            GetDOFValues(v);
            stringstream ss;
            ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10 + 1);
            ss << "self collision report=" << report->__str__() << " ";
            for (size_t i = 0; i < v.size(); ++i) {
                if (i > 0) {
                    ss << "," << v[i];
                } else {
                    ss << "colvalues=[" << v[i];
                }
            }
            ss << "]";
            RAVELOG_VERBOSE(ss.str());
        }
    }

    return is_collision;
}

bool KinBody::CheckLinkCollision(int ilinkindex, const Transform& tlinktrans, CollisionReportPtr report)
{
    LinkPtr plink = links_vector_.at(ilinkindex);
    CollisionCheckerBasePtr pchecker = GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions() & CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if (!!report && bAllLinkCollisions && report->nKeepPrevious == 0) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }

    bool bincollision = false;
    if (plink->IsEnabled()) {
        std::shared_ptr<TransformSaver<LinkPtr>> linksaver(new TransformSaver<LinkPtr>(plink)); // gcc optimization bug when linksaver is on stack?
        plink->SetTransform(tlinktrans);
        if (pchecker->CheckCollision(LinkConstPtr(plink), report)) {
            if (!bAllLinkCollisions) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }

    // check if any grabbed bodies are attached to this link, and if so check their collisions with the environment
    // it is important to make sure to add all other attached bodies in the ignored list!
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    FOREACHC(itgrabbed, grabbed_bodies_)
    {
        GrabbedConstPtr pgrabbed = std::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
        if (pgrabbed->link_robot_ == plink) {
            KinBodyPtr pbody = pgrabbed->grabbed_body_.lock();
            if (!!pbody) {
                vbodyexcluded.resize(0);
                vbodyexcluded.push_back(shared_kinbody_const());
                FOREACHC(itgrabbed2, grabbed_bodies_)
                {
                    if (itgrabbed2 != itgrabbed) {
                        GrabbedConstPtr pgrabbed2 = std::dynamic_pointer_cast<Grabbed const>(*itgrabbed2);
                        KinBodyPtr pbody2 = pgrabbed2->grabbed_body_.lock();
                        if (!!pbody2) {
                            vbodyexcluded.push_back(pbody2);
                        }
                    }
                }
                KinBodyStateSaver bodysaver(pbody, Save_LinkTransformation);
                pbody->SetTransform(tlinktrans * pgrabbed->_troot);
                if (pchecker->CheckCollision(KinBodyConstPtr(pbody), vbodyexcluded, vlinkexcluded, report)) {
                    if (!bAllLinkCollisions) { // if checking all collisions, have to continue
                        return true;
                    }
                    bincollision = true;
                }
            }
        }
    }
    return bincollision;
}

bool KinBody::CheckLinkCollision(int ilinkindex, CollisionReportPtr report)
{
    LinkPtr plink = links_vector_.at(ilinkindex);
    CollisionCheckerBasePtr pchecker = GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions() & CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if (!!report && bAllLinkCollisions && report->nKeepPrevious == 0) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }
    bool bincollision = false;
    if (plink->IsEnabled()) {
        if (pchecker->CheckCollision(LinkConstPtr(plink), report)) {
            if (!bAllLinkCollisions) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }

    // check if any grabbed bodies are attached to this link, and if so check their collisions with the environment
    // it is important to make sure to add all other attached bodies in the ignored list!
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    FOREACHC(itgrabbed, grabbed_bodies_)
    {
        GrabbedConstPtr pgrabbed = std::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
        if (pgrabbed->link_robot_ == plink) {
            KinBodyPtr pbody = pgrabbed->grabbed_body_.lock();
            if (!!pbody) {
                vbodyexcluded.resize(0);
                vbodyexcluded.push_back(shared_kinbody_const());
                FOREACHC(itgrabbed2, grabbed_bodies_)
                {
                    if (itgrabbed2 != itgrabbed) {
                        GrabbedConstPtr pgrabbed2 = std::dynamic_pointer_cast<Grabbed const>(*itgrabbed2);
                        KinBodyPtr pbody2 = pgrabbed2->grabbed_body_.lock();
                        if (!!pbody2) {
                            vbodyexcluded.push_back(pbody2);
                        }
                    }
                }
                if (pchecker->CheckCollision(KinBodyConstPtr(pbody), vbodyexcluded, vlinkexcluded, report)) {
                    if (!bAllLinkCollisions) { // if checking all collisions, have to continue
                        return true;
                    }
                    bincollision = true;
                }
            }
        }
    }
    return bincollision;
}

bool KinBody::CheckLinkSelfCollision(int ilinkindex, CollisionReportPtr report)
{
    // TODO: have to consider rigidly attached links??
    CollisionCheckerBasePtr pchecker = !!self_collision_checker_ ? self_collision_checker_ : GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions() & CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if (!!report && bAllLinkCollisions && report->nKeepPrevious == 0) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }
    bool bincollision = false;
    LinkPtr plink = links_vector_.at(ilinkindex);
    if (plink->IsEnabled()) {
        std::shared_ptr<TransformSaver<LinkPtr>> linksaver(new TransformSaver<LinkPtr>(plink)); // gcc optimization bug when linksaver is on stack?
        if (pchecker->CheckStandaloneSelfCollision(LinkConstPtr(plink), report)) {
            if (!bAllLinkCollisions) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }

    KinBodyStateSaverPtr linksaver;
    // check if any grabbed bodies are attached to this link, and if so check their collisions with the environment
    // it is important to make sure to add all other attached bodies in the ignored list!
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    FOREACHC(itgrabbed, grabbed_bodies_)
    {
        GrabbedConstPtr pgrabbed = std::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
        if (pgrabbed->link_robot_ == plink) {
            KinBodyPtr pbody = pgrabbed->grabbed_body_.lock();
            if (!!pbody) {
                if (!linksaver) {
                    linksaver.reset(new KinBodyStateSaver(shared_kinbody()));
                    plink->Enable(false);
                    // also disable rigidly attached links?
                }
                KinBodyStateSaver bodysaver(pbody, Save_LinkTransformation);
                if (pchecker->CheckCollision(shared_kinbody_const(), KinBodyConstPtr(pbody), report)) {
                    if (!bAllLinkCollisions) { // if checking all collisions, have to continue
                        return true;
                    }
                    bincollision = true;
                }
            }
        }
    }
    return bincollision;
}

bool KinBody::CheckLinkSelfCollision(int ilinkindex, const Transform& tlinktrans, CollisionReportPtr report)
{
    // TODO: have to consider rigidly attached links??
    CollisionCheckerBasePtr pchecker = !!self_collision_checker_ ? self_collision_checker_ : GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions() & CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if (!!report && bAllLinkCollisions && report->nKeepPrevious == 0) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }
    bool bincollision = false;
    LinkPtr plink = links_vector_.at(ilinkindex);
    if (plink->IsEnabled()) {
        std::shared_ptr<TransformSaver<LinkPtr>> linksaver(new TransformSaver<LinkPtr>(plink)); // gcc optimization bug when linksaver is on stack?
        plink->SetTransform(tlinktrans);
        if (pchecker->CheckStandaloneSelfCollision(LinkConstPtr(plink), report)) {
            if (!bAllLinkCollisions) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }

    KinBodyStateSaverPtr linksaver;
    // check if any grabbed bodies are attached to this link, and if so check their collisions with the environment
    // it is important to make sure to add all other attached bodies in the ignored list!
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    FOREACHC(itgrabbed, grabbed_bodies_)
    {
        GrabbedConstPtr pgrabbed = std::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
        if (pgrabbed->link_robot_ == plink) {
            KinBodyPtr pbody = pgrabbed->grabbed_body_.lock();
            if (!!pbody) {
                if (!linksaver) {
                    linksaver.reset(new KinBodyStateSaver(shared_kinbody()));
                    plink->Enable(false);
                    // also disable rigidly attached links?
                }
                KinBodyStateSaver bodysaver(pbody, Save_LinkTransformation);
                pbody->SetTransform(tlinktrans * pgrabbed->_troot);
                if (pchecker->CheckCollision(shared_kinbody_const(), KinBodyConstPtr(pbody), report)) {
                    if (!bAllLinkCollisions) { // if checking all collisions, have to continue
                        return true;
                    }
                    bincollision = true;
                }
            }
        }
    }
    return bincollision;
}

} // end namespace OpenRAVE
