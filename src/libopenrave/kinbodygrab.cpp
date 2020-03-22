﻿// -*- coding: utf-8 -*-
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
#include <openrave/kinbody.h>
#include <openrave/grabbed.h>

namespace OpenRAVE {

bool KinBody::Grab(KinBodyPtr pbody, LinkPtr plink)
{
    OPENRAVE_ASSERT_FORMAT(!!pbody, "body %s invalid body to grab",GetName(),ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(!!plink && plink->GetParent().get() == this, "body %s grabbing link needs to be part of body",GetName(),ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(pbody.get() != this,"body %s cannot grab itself",GetName(), ORE_InvalidArguments);

    //uint64_t starttime0 = utils::GetMicroTime();

    // if grabbing, check if the transforms are different. If they are, then update the transform
    GrabbedPtr pPreviousGrabbed;
    FOREACHC(itgrabbed, grabbed_bodies_) {
        GrabbedPtr pgrabbed = std::dynamic_pointer_cast<Grabbed>(*itgrabbed);
        if( pgrabbed->grabbed_body_.lock() == pbody ) {
            pPreviousGrabbed = pgrabbed;
            break;
        }
    }

    // double check since collision checkers might not support this case
    if( pbody->HasAttached() ) {
        if( !!pPreviousGrabbed ) {
            RAVELOG_INFO_FORMAT("env=%d, body %s is previously grabbed by %s, so", GetEnv()->GetId()%pbody->GetName()%GetName());
        }
        else {
            std::set<KinBodyPtr> setAttached;
            pbody->GetAttached(setAttached);
            std::stringstream ss;
            if( setAttached.size() > 1 ) {
                FOREACH(itbody, setAttached) {
                    ss << (*itbody)->GetName() << ", ";
                }
            }
            RAVELOG_WARN_FORMAT("env=%d, body %s trying to grab body %s with %d attached bodies [%s]", GetEnv()->GetId()%GetName()%pbody->GetName()%setAttached.size()%ss.str());
        }
    }

    Transform t = plink->GetTransform();
    Transform tbody = pbody->GetTransform();
    // new body velocity is measured from body link
    std::pair<Vector, Vector> velocity = plink->GetVelocity();
    velocity.first += velocity.second.cross(tbody.trans - t.trans);
    if( !!pPreviousGrabbed ) {
        dReal disterror = TransformDistance2(t*pPreviousGrabbed->_troot, tbody);
        if( pPreviousGrabbed->link_robot_ == plink && disterror <= g_fEpsilonLinear ) {
            // links and transforms are the same, so no worries
            return true;
        }
        RAVELOG_VERBOSE_FORMAT("Body %s: body %s already grabbed, but transforms differ by %f \n", GetName()%pbody->GetName()%disterror);
        _RemoveAttachedBody(*pbody);
        CallOnDestruction destructigonhook(boost::bind(&RobotBase::_AttachBody,this,pbody));
        pPreviousGrabbed->link_robot_ = plink;
        pPreviousGrabbed->_troot = t.inverse() * tbody;
        pPreviousGrabbed->ProcessCollidingLinks(pPreviousGrabbed->_setRobotLinksToIgnore);
        pbody->SetVelocity(velocity.first, velocity.second);
        return true;
    }

    GrabbedPtr pgrabbed(new Grabbed(pbody,plink));
    pgrabbed->_troot = t.inverse() * tbody;
    //uint64_t starttime1 = utils::GetMicroTime();
    // always ignore links that are statically attached to plink (ie assume they are always colliding with the body)

    std::vector<std::shared_ptr<Link> > vattachedlinks;
    plink->GetRigidlyAttachedLinks(vattachedlinks);
    std::set<int> setBodyLinksToIgnore;
    FOREACHC(itlink, vattachedlinks) {
        setBodyLinksToIgnore.insert((*itlink)->GetIndex());
    }
    if( !!self_collision_checker_ && self_collision_checker_ != GetEnv()->GetCollisionChecker() ) {
        // collision checking will not be automatically updated with environment calls, so need to do this manually
        //try {
        self_collision_checker_->InitKinBody(pbody);
//        }
//        catch (const std::exception& ex) {
//            RAVELOG_ERROR_FORMAT("env=%d, failed in self_collision_checker_->InitKinBody for body %s: %s", GetEnv()->GetId()%pbody->GetName()%ex.what());
//            throw;
//        }
    }
    //    try {
    pgrabbed->ProcessCollidingLinks(setBodyLinksToIgnore);
//    }
//    catch(const std::exception& ex) {
//        RAVELOG_ERROR_FORMAT("env=%d, failed in ProcessCollidingLinks for body %s: %s", GetEnv()->GetId()%pbody->GetName()%ex.what());
//        throw;
//    }

    pbody->SetVelocity(velocity.first, velocity.second);
    grabbed_bodies_.push_back(pgrabbed);
    //uint64_t starttime2 = utils::GetMicroTime();
    try {
        // if an exception happens in _AttachBody, have to remove from grabbed_bodies_
        _AttachBody(pbody);
    }
    catch(...) {
        RAVELOG_ERROR_FORMAT("env=%d, failed in attach body", GetEnv()->GetId());
        BOOST_ASSERT(grabbed_bodies_.back()==pgrabbed);
        // do not call self_collision_checker_->RemoveKinBody since the same object might be re-attached later on and we should preserve the structures.
        grabbed_bodies_.pop_back();
        throw;
    }
    //uint64_t starttime3 = utils::GetMicroTime();
    try {
        _PostprocessChangedParameters(Prop_RobotGrabbed);
    }
    catch (const std::exception& ex) {
        RAVELOG_ERROR_FORMAT("env=%d, failed in _PostprocessChangedParameters: %s", GetEnv()->GetId()%ex.what());
        throw;
    }
    //RAVELOG_DEBUG_FORMAT("env=%d, post process elapsed (%d) %fs, %fs, %fs, %fs", GetEnv()->GetId()%vattachedlinks.size()%(1e-6*(starttime1-starttime0))%(1e-6*(starttime2-starttime0))%(1e-6*(starttime3-starttime0))%(1e-6*(utils::GetMicroTime()-starttime0)));
    return true;
}

bool KinBody::Grab(KinBodyPtr pbody, LinkPtr pBodyLinkToGrabWith, const std::set<int>& setBodyLinksToIgnore)
{
    OPENRAVE_ASSERT_FORMAT(!!pbody && !!pBodyLinkToGrabWith && pBodyLinkToGrabWith->GetParent().get() == this, "body %s invalid grab arguments",GetName(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(pbody.get() != this, "body %s cannot grab itself",pbody->GetName(), ORE_InvalidArguments);
    if( IsGrabbing(*pbody) ) {
        if( setBodyLinksToIgnore.size() > 0 ) {
            // update the current grabbed info with setBodyLinksToIgnore
            FOREACHC(itgrabbed, grabbed_bodies_) {
                GrabbedPtr pgrabbed = std::dynamic_pointer_cast<Grabbed>(*itgrabbed);
                if( pgrabbed->grabbed_body_.lock() == pbody ) {
                    pgrabbed->AddMoreIgnoreLinks(setBodyLinksToIgnore);
                    break;
                }
            }
        }
        RAVELOG_VERBOSE(str(boost::format("Body %s: body %s already grabbed\n")%GetName()%pbody->GetName()));
        return true;
    }

    GrabbedPtr pgrabbed(new Grabbed(pbody,pBodyLinkToGrabWith));
    Transform t = pBodyLinkToGrabWith->GetTransform();
    Transform tbody = pbody->GetTransform();
    pgrabbed->_troot = t.inverse() * tbody;

    if( !!self_collision_checker_ && self_collision_checker_ != GetEnv()->GetCollisionChecker() ) {
        // collision checking will not be automatically updated with environment calls, so need to do this manually
        self_collision_checker_->InitKinBody(pbody);
    }
    pgrabbed->ProcessCollidingLinks(setBodyLinksToIgnore);

    // set velocity
    std::pair<Vector, Vector> velocity = pBodyLinkToGrabWith->GetVelocity();
    velocity.first += velocity.second.cross(tbody.trans - t.trans);
    pbody->SetVelocity(velocity.first, velocity.second);
    grabbed_bodies_.push_back(pgrabbed);
    try {
        // if an exception happens in _AttachBody, have to remove from grabbed_bodies_
        _AttachBody(pbody);
    }
    catch(...) {
        BOOST_ASSERT(grabbed_bodies_.back()==pgrabbed);
        // do not call self_collision_checker_->RemoveKinBody since the same object might be re-attached later on and we should preserve the structures.
        grabbed_bodies_.pop_back();
        throw;
    }
    _PostprocessChangedParameters(Prop_RobotGrabbed);
    return true;
}

void KinBody::Release(KinBody &body)
{
    FOREACH(itgrabbed, grabbed_bodies_) {
        GrabbedPtr pgrabbed = std::dynamic_pointer_cast<Grabbed>(*itgrabbed);
        KinBodyConstPtr pgrabbedbody = pgrabbed->grabbed_body_.lock();
        if( !!pgrabbedbody ) {
            bool bpointermatch = pgrabbedbody.get() == &body;
            bool bnamematch = pgrabbedbody->GetName() == body.GetName();
            if( bpointermatch != bnamematch ) {
                RAVELOG_WARN_FORMAT("env=%d, body %s has grabbed body %s (%d), but it does not match with %s (%d) ", GetEnv()->GetId()%GetName()%pgrabbedbody->GetName()%pgrabbedbody->GetEnvironmentId()%body.GetName()%body.GetEnvironmentId());
            }
            if( bpointermatch ) {
                grabbed_bodies_.erase(itgrabbed);
                _RemoveAttachedBody(body);
                _PostprocessChangedParameters(Prop_RobotGrabbed);
                return;
            }
        }
    }

    if( IS_DEBUGLEVEL(Level_Debug) ) {
        std::stringstream ss;
        FOREACH(itgrabbed, grabbed_bodies_) {
            GrabbedPtr pgrabbed = std::dynamic_pointer_cast<Grabbed>(*itgrabbed);
            KinBodyConstPtr pgrabbedbody = pgrabbed->grabbed_body_.lock();
            if( !!pgrabbedbody ) {
                ss << pgrabbedbody->GetName() << ", ";
            }
        }

        RAVELOG_DEBUG_FORMAT("env=%d, body %s is not grabbing body %s (%d), but grabbing bodies [%s]", GetEnv()->GetId()%GetName()%body.GetName()%body.GetEnvironmentId()%ss.str());
    }
}

void KinBody::ReleaseAllGrabbed()
{
    if( grabbed_bodies_.size() > 0 ) {
        FOREACH(itgrabbed, grabbed_bodies_) {
            GrabbedPtr pgrabbed = std::dynamic_pointer_cast<Grabbed>(*itgrabbed);
            KinBodyPtr pbody = pgrabbed->grabbed_body_.lock();
            if( !!pbody ) {
                _RemoveAttachedBody(*pbody);
            }
        }
        grabbed_bodies_.clear();
        _PostprocessChangedParameters(Prop_RobotGrabbed);
    }
}

void KinBody::ReleaseAllGrabbedWithLink(const KinBody::Link& bodyLinkToReleaseWith)
{
    OPENRAVE_ASSERT_FORMAT(bodyLinkToReleaseWith.GetParent().get() == this, "body %s invalid grab arguments",GetName(), ORE_InvalidArguments);

    if( grabbed_bodies_.size() > 0 ) {
        bool bReleased = false;
        int nCheckIndex = (int)grabbed_bodies_.size()-1;
        while(nCheckIndex >= 0) {
            GrabbedPtr pgrabbed = std::dynamic_pointer_cast<Grabbed>(grabbed_bodies_.at(nCheckIndex));
            if( pgrabbed->link_robot_.get() == &bodyLinkToReleaseWith ) {
                KinBodyPtr pbody = pgrabbed->grabbed_body_.lock();
                if( !!pbody ) {
                    _RemoveAttachedBody(*pbody);
                }
                grabbed_bodies_.erase(grabbed_bodies_.begin()+nCheckIndex);
                bReleased = true;
            }
            --nCheckIndex;
        }
        if( bReleased ) {
            _PostprocessChangedParameters(Prop_RobotGrabbed);
        }
    }
}

void KinBody::RegrabAll()
{
    CollisionCheckerBasePtr collisionchecker = !!self_collision_checker_ ? self_collision_checker_ : GetEnv()->GetCollisionChecker();
    CollisionOptionsStateSaver colsaver(collisionchecker,0); // have to reset the collision options
    std::vector<LinkPtr > vattachedlinks;
    FOREACH(itgrabbed, grabbed_bodies_) {
        GrabbedPtr pgrabbed = std::dynamic_pointer_cast<Grabbed>(*itgrabbed);
        KinBodyPtr pbody = pgrabbed->grabbed_body_.lock();
        if( !!pbody ) {
            _RemoveAttachedBody(*pbody);
            CallOnDestruction destructionhook(boost::bind(&RobotBase::_AttachBody,this,pbody));
            pgrabbed->ProcessCollidingLinks(pgrabbed->_setRobotLinksToIgnore);
        }
    }
}

void KinBody::_Regrab(UserDataPtr _pgrabbed)
{
    GrabbedPtr pgrabbed = std::dynamic_pointer_cast<Grabbed>(_pgrabbed);
    KinBodyPtr pgrabbedbody = pgrabbed->grabbed_body_.lock();
    if( !!pgrabbedbody ) {
        // have to re-grab the body, which means temporarily resetting the collision checker and attachment
        CollisionCheckerBasePtr collisionchecker = !!self_collision_checker_ ? self_collision_checker_ : GetEnv()->GetCollisionChecker();
        CollisionOptionsStateSaver colsaver(collisionchecker,0); // have to reset the collision options
        _RemoveAttachedBody(*pgrabbedbody);
        CallOnDestruction destructionhook(boost::bind(&RobotBase::_AttachBody,this,pgrabbedbody));
        pgrabbed->ProcessCollidingLinks(pgrabbed->_setRobotLinksToIgnore);
    }
}

KinBody::LinkPtr KinBody::IsGrabbing(const KinBody &body) const
{
    FOREACHC(itgrabbed, grabbed_bodies_) {
        GrabbedPtr pgrabbed = std::dynamic_pointer_cast<Grabbed>(*itgrabbed);
        KinBodyConstPtr pgrabbedbody = pgrabbed->grabbed_body_.lock();
        if( !!pgrabbedbody && pgrabbedbody.get() == &body ) {
            return pgrabbed->link_robot_;
        }
    }
    return LinkPtr();
}

void KinBody::GetGrabbed(std::vector<KinBodyPtr>& vbodies) const
{
    vbodies.resize(0);
    FOREACHC(itgrabbed, grabbed_bodies_) 
	{
        GrabbedConstPtr pgrabbed = std::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
        KinBodyPtr pbody = pgrabbed->grabbed_body_.lock();
        if( !!pbody && pbody->GetEnvironmentId() ) 
		{
            vbodies.push_back(pbody);
        }
    }
}

void KinBody::GetGrabbedInfo(std::vector<KinBody::GrabbedInfoPtr>& vgrabbedinfo) const
{
    vgrabbedinfo.reserve(grabbed_bodies_.size());
    vgrabbedinfo.clear();
    for(size_t i = 0; i < grabbed_bodies_.size(); ++i) {
        GrabbedConstPtr pgrabbed = std::dynamic_pointer_cast<Grabbed const>(grabbed_bodies_[i]);
        KinBodyPtr pgrabbedbody = pgrabbed->grabbed_body_.lock();
        // sometimes bodies can be removed before they are Released, this is ok and can happen during exceptions and stack unwinding
        if( !!pgrabbedbody ) {
            KinBody::GrabbedInfoPtr poutputinfo(new GrabbedInfo());
            poutputinfo->grabbed_name_ = pgrabbedbody->GetName();
            poutputinfo->robot_link_name_ = pgrabbed->link_robot_->GetName();
            poutputinfo->relative_transform_ = pgrabbed->_troot;
            poutputinfo->robot_links_to_ignore_set_ = pgrabbed->_setRobotLinksToIgnore;
            FOREACHC(itlink, links_vector_) {
                if( find(pgrabbed->non_colliding_links_.begin(), pgrabbed->non_colliding_links_.end(), *itlink) == pgrabbed->non_colliding_links_.end() ) {
                    poutputinfo->robot_links_to_ignore_set_.insert((*itlink)->GetIndex());
                }
            }
            vgrabbedinfo.push_back(poutputinfo);
        }
    }
}

void KinBody::GrabbedInfo::SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal unit_scale, int options) const
{
    openravejson::SetJsonValueByKey(value, "grabbedName", grabbed_name_, allocator);
    openravejson::SetJsonValueByKey(value, "robotLinkName", robot_link_name_, allocator);
    openravejson::SetJsonValueByKey(value, "transform", relative_transform_, allocator);
    openravejson::SetJsonValueByKey(value, "robotLinksToIgnoreSet", robot_links_to_ignore_set_, allocator);
}

void KinBody::GrabbedInfo::DeserializeJSON(const rapidjson::Value& value, dReal unit_scale)
{
    openravejson::LoadJsonValueByKey(value, "grabbedName", grabbed_name_);
    openravejson::LoadJsonValueByKey(value, "robotLinkName", robot_link_name_);
    openravejson::LoadJsonValueByKey(value, "transform", relative_transform_);
    openravejson::LoadJsonValueByKey(value, "robotLinksToIgnoreSet", robot_links_to_ignore_set_);
}

void KinBody::ResetGrabbed(const std::vector<KinBody::GrabbedInfoConstPtr>& vgrabbedinfo)
{
    ReleaseAllGrabbed();
    if( vgrabbedinfo.size() > 0 ) {
        CollisionCheckerBasePtr collisionchecker = !!self_collision_checker_ ? self_collision_checker_ : GetEnv()->GetCollisionChecker();
        CollisionOptionsStateSaver colsaver(collisionchecker,0); // have to reset the collision options
        FOREACHC(itgrabbedinfo, vgrabbedinfo) {
            GrabbedInfoConstPtr pgrabbedinfo = *itgrabbedinfo;
            KinBodyPtr pbody = GetEnv()->GetKinBody(pgrabbedinfo->grabbed_name_);
            KinBody::LinkPtr pBodyLinkToGrabWith = GetLink(pgrabbedinfo->robot_link_name_);
            OPENRAVE_ASSERT_FORMAT(!!pbody && !!pBodyLinkToGrabWith, "body %s invalid grab arguments",GetName(), ORE_InvalidArguments);
            OPENRAVE_ASSERT_FORMAT(pbody.get() != this, "body %s cannot grab itself",pbody->GetName(), ORE_InvalidArguments);
            if( IsGrabbing(*pbody) ) {
                RAVELOG_VERBOSE(str(boost::format("Body %s: body %s already grabbed\n")%GetName()%pbody->GetName()));
                continue;
            }

            GrabbedPtr pgrabbed(new Grabbed(pbody,pBodyLinkToGrabWith));
            pgrabbed->_troot = pgrabbedinfo->relative_transform_;
            if( !!self_collision_checker_ && self_collision_checker_ != GetEnv()->GetCollisionChecker() ) {
                // collision checking will not be automatically updated with environment calls, so need to do this manually
                self_collision_checker_->InitKinBody(pbody);
            }
            pgrabbed->ProcessCollidingLinks(pgrabbedinfo->robot_links_to_ignore_set_);
            Transform tlink = pBodyLinkToGrabWith->GetTransform();
            Transform tbody = tlink * pgrabbed->_troot;
            pbody->SetTransform(tbody);
            // set velocity
            std::pair<Vector, Vector> velocity = pBodyLinkToGrabWith->GetVelocity();
            velocity.first += velocity.second.cross(tbody.trans - tlink.trans);
            pbody->SetVelocity(velocity.first, velocity.second);
            grabbed_bodies_.push_back(pgrabbed);
            _AttachBody(pbody);
        }
        _PostprocessChangedParameters(Prop_RobotGrabbed);
    }
}

void KinBody::GetIgnoredLinksOfGrabbed(KinBodyConstPtr body, std::list<KinBody::LinkConstPtr>& ignorelinks) const
{
    ignorelinks.clear();
    FOREACHC(itgrabbed, grabbed_bodies_) {
        GrabbedConstPtr pgrabbed = std::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
        KinBodyPtr grabbedbody = pgrabbed->grabbed_body_.lock();
        if( grabbedbody == body ) {
            FOREACHC(itbodylink, links_vector_) {
                if( find(pgrabbed->non_colliding_links_.begin(), pgrabbed->non_colliding_links_.end(), *itbodylink) == pgrabbed->non_colliding_links_.end() ) {
                    ignorelinks.push_back(*itbodylink);
                }
            }
            return;
        }
    }
    RAVELOG_WARN(str(boost::format("body %s is not currently grabbed")%body->GetName()));
}

void KinBody::_UpdateGrabbedBodies()
{
    vector<UserDataPtr>::iterator itgrabbed = grabbed_bodies_.begin();
    while(itgrabbed != grabbed_bodies_.end() ) {
        GrabbedPtr pgrabbed = std::dynamic_pointer_cast<Grabbed>(*itgrabbed);
        KinBodyPtr pbody = pgrabbed->grabbed_body_.lock();
        if( !!pbody ) {
            Transform t = pgrabbed->link_robot_->GetTransform();
            pbody->SetTransform(t * pgrabbed->_troot);
            // set the correct velocity
            std::pair<Vector, Vector> velocity = pgrabbed->link_robot_->GetVelocity();
            velocity.first += velocity.second.cross(t.rotate(pgrabbed->_troot.trans));
            pbody->SetVelocity(velocity.first, velocity.second);
            ++itgrabbed;
        }
        else {
            RAVELOG_DEBUG(str(boost::format("erasing invaliding grabbed body from %s")%GetName()));
            itgrabbed = grabbed_bodies_.erase(itgrabbed);
        }
    }
}

} // end namespace OpenRAVE
