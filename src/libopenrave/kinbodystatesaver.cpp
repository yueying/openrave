﻿// -*- coding: utf-8 -*-
// Copyright (C) 2006-2019 Rosen Diankov (rosen.diankov@gmail.com)
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

namespace OpenRAVE {

KinBody::KinBodyStateSaver::KinBodyStateSaver(KinBodyPtr pbody, int options) : kinbody_(pbody), options_(options), is_restore_on_destructor_(true)
{
    if( options_ & Save_LinkTransformation ) {
        kinbody_->GetLinkTransformations(link_transforms_vector_, _vdoflastsetvalues);
    }
    if( options_ & Save_LinkEnable ) {
        _vEnabledLinks.resize(kinbody_->GetLinks().size());
        for(size_t i = 0; i < _vEnabledLinks.size(); ++i) {
            _vEnabledLinks[i] = kinbody_->GetLinks().at(i)->IsEnabled();
        }
    }
    if( options_ & Save_LinkVelocities ) {
        kinbody_->GetLinkVelocities(_vLinkVelocities);
    }
    if( options_ & Save_JointMaxVelocityAndAcceleration ) {
        kinbody_->GetDOFVelocityLimits(_vMaxVelocities);
        kinbody_->GetDOFAccelerationLimits(_vMaxAccelerations);
        kinbody_->GetDOFJerkLimits(_vMaxJerks);
    }
    if( options_ & Save_JointWeights ) {
        kinbody_->GetDOFWeights(_vDOFWeights);
    }
    if( options_ & Save_JointLimits ) {
        kinbody_->GetDOFLimits(_vDOFLimits[0], _vDOFLimits[1]);
    }
    if( options_ & Save_GrabbedBodies ) {
        _vGrabbedBodies = kinbody_->_vGrabbedBodies;
    }

}

KinBody::KinBodyStateSaver::~KinBodyStateSaver()
{
    if( is_restore_on_destructor_ && !!kinbody_ && kinbody_->GetEnvironmentId() != 0 ) {
        _RestoreKinBody(kinbody_);
    }
}

void KinBody::KinBodyStateSaver::Restore(std::shared_ptr<KinBody> body)
{
    _RestoreKinBody(!body ? kinbody_ : body);
}

void KinBody::KinBodyStateSaver::Release()
{
    kinbody_.reset();
}

void KinBody::KinBodyStateSaver::SetRestoreOnDestructor(bool restore)
{
    is_restore_on_destructor_ = restore;
}

void KinBody::KinBodyStateSaver::_RestoreKinBody(std::shared_ptr<KinBody> pbody)
{
    if( !pbody ) {
        return;
    }
    if( pbody->GetEnvironmentId() == 0 ) {
        RAVELOG_WARN_FORMAT("env=%d, body %s not added to environment, skipping restore", pbody->GetEnv()->GetId()%pbody->GetName());
        return;
    }
    if( options_ & Save_JointLimits ) {
        pbody->SetDOFLimits(_vDOFLimits[0], _vDOFLimits[1]);
    }
    // restoring grabbed bodies has to happen first before link transforms can be restored since _UpdateGrabbedBodies can be called with the old grabbed bodies.
    if( options_ & Save_GrabbedBodies ) {
        // have to release all grabbed first
        pbody->ReleaseAllGrabbed();
        OPENRAVE_ASSERT_OP(pbody->_vGrabbedBodies.size(),==,0);
        FOREACH(itgrabbed, _vGrabbedBodies) {
            GrabbedPtr pgrabbed = std::dynamic_pointer_cast<Grabbed>(*itgrabbed);
            KinBodyPtr pbodygrab = pgrabbed->_pgrabbedbody.lock();
            if( !!pbodygrab ) {
                if( pbody->GetEnv() == kinbody_->GetEnv() ) {
                    pbody->_AttachBody(pbodygrab);
                    pbody->_vGrabbedBodies.push_back(*itgrabbed);
                }
                else {
                    // pgrabbed points to a different environment, so have to re-initialize
                    KinBodyPtr pnewbody = pbody->GetEnv()->GetBodyFromEnvironmentId(pbodygrab->GetEnvironmentId());
                    if( pbodygrab->GetKinematicsGeometryHash() != pnewbody->GetKinematicsGeometryHash() ) {
                        RAVELOG_WARN(str(boost::format("body %s is not similar across environments")%pbodygrab->GetName()));
                    }
                    else {
                        GrabbedPtr pnewgrabbed(new Grabbed(pnewbody,pbody->GetLinks().at(KinBody::LinkPtr(pgrabbed->_plinkrobot)->GetIndex())));
                        pnewgrabbed->_troot = pgrabbed->_troot;
                        pnewgrabbed->_listNonCollidingLinks.clear();
                        FOREACHC(itlinkref, pgrabbed->_listNonCollidingLinks) {
                            pnewgrabbed->_listNonCollidingLinks.push_back(pbody->GetLinks().at((*itlinkref)->GetIndex()));
                        }
                        pbody->_AttachBody(pnewbody);
                        pbody->_vGrabbedBodies.push_back(pnewgrabbed);
                    }
                }
            }
        }

        // if not calling SetLinkTransformations, then manually call _UpdateGrabbedBodies
        if( !(options_ & Save_LinkTransformation ) ) {
            pbody->_UpdateGrabbedBodies();
        }
    }
    if( options_ & Save_LinkTransformation ) {
        pbody->SetLinkTransformations(link_transforms_vector_, _vdoflastsetvalues);
//        if( IS_DEBUGLEVEL(Level_Warn) ) {
//            stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
//            ss << "restoring kinbody " << pbody->GetName() << " to values=[";
//            std::vector<dReal> values;
//            pbody->GetDOFValues(values);
//            FOREACH(it,values) {
//                ss << *it << ", ";
//            }
//            ss << "]";
//            RAVELOG_WARN(ss.str());
//        }
    }
    if( options_ & Save_LinkEnable ) {
        // should first enable before calling the parameter callbacks
        bool bchanged = false;
        for(size_t i = 0; i < _vEnabledLinks.size(); ++i) {
            if( pbody->GetLinks().at(i)->IsEnabled() != !!_vEnabledLinks[i] ) {
                pbody->GetLinks().at(i)->info_.is_enabled_ = !!_vEnabledLinks[i];
                bchanged = true;
            }
        }
        if( bchanged ) {
            pbody->non_adjacent_link_cache_ &= ~AO_Enabled;
            pbody->_PostprocessChangedParameters(Prop_LinkEnable);
        }
    }
    if( options_ & Save_JointMaxVelocityAndAcceleration ) {
        pbody->SetDOFVelocityLimits(_vMaxVelocities);
        pbody->SetDOFAccelerationLimits(_vMaxAccelerations);
        pbody->SetDOFJerkLimits(_vMaxJerks);
    }
    if( options_ & Save_LinkVelocities ) {
        pbody->SetLinkVelocities(_vLinkVelocities);
    }
    if( options_ & Save_JointWeights ) {
        pbody->SetDOFWeights(_vDOFWeights);
    }
}


KinBody::KinBodyStateSaverRef::KinBodyStateSaverRef(KinBody& body, int options) : _body(body), _options(options), _bRestoreOnDestructor(true), _bReleased(false)
{
    if( _options & Save_LinkTransformation ) {
        body.GetLinkTransformations(_vLinkTransforms, _vdoflastsetvalues);
    }
    if( _options & Save_LinkEnable ) {
        _vEnabledLinks.resize(body.GetLinks().size());
        for(size_t i = 0; i < _vEnabledLinks.size(); ++i) {
            _vEnabledLinks[i] = body.GetLinks().at(i)->IsEnabled();
        }
    }
    if( _options & Save_LinkVelocities ) {
        body.GetLinkVelocities(_vLinkVelocities);
    }
    if( _options & Save_JointMaxVelocityAndAcceleration ) {
        body.GetDOFVelocityLimits(_vMaxVelocities);
        body.GetDOFAccelerationLimits(_vMaxAccelerations);
        body.GetDOFJerkLimits(_vMaxJerks);
    }
    if( _options & Save_JointWeights ) {
        body.GetDOFWeights(_vDOFWeights);
    }
    if( _options & Save_JointLimits ) {
        body.GetDOFLimits(_vDOFLimits[0], _vDOFLimits[1]);
    }
    if( _options & Save_GrabbedBodies ) {
        _vGrabbedBodies = body._vGrabbedBodies;
    }
}

KinBody::KinBodyStateSaverRef::~KinBodyStateSaverRef()
{
    if( _bRestoreOnDestructor && !_bReleased && _body.GetEnvironmentId() != 0 ) {
        _RestoreKinBody(_body);
    }
}

void KinBody::KinBodyStateSaverRef::Restore()
{
    if( !_bReleased ) {
        _RestoreKinBody(_body);
    }
}

void KinBody::KinBodyStateSaverRef::Restore(KinBody& body)
{
    _RestoreKinBody(body);
}

void KinBody::KinBodyStateSaverRef::Release()
{
    _bReleased = true;
}

void KinBody::KinBodyStateSaverRef::SetRestoreOnDestructor(bool restore)
{
    _bRestoreOnDestructor = restore;
}

void KinBody::KinBodyStateSaverRef::_RestoreKinBody(KinBody& body)
{
    if( body.GetEnvironmentId() == 0 ) {
        RAVELOG_WARN(str(boost::format("body %s not added to environment, skipping restore")%body.GetName()));
        return;
    }
    if( _options & Save_JointLimits ) {
        body.SetDOFLimits(_vDOFLimits[0], _vDOFLimits[1]);
    }
    // restoring grabbed bodies has to happen first before link transforms can be restored since _UpdateGrabbedBodies can be called with the old grabbed bodies.
    if( _options & Save_GrabbedBodies ) {
        // have to release all grabbed first
        body.ReleaseAllGrabbed();
        OPENRAVE_ASSERT_OP(body._vGrabbedBodies.size(),==,0);
        FOREACH(itgrabbed, _vGrabbedBodies) {
            GrabbedPtr pgrabbed = std::dynamic_pointer_cast<Grabbed>(*itgrabbed);
            KinBodyPtr pbodygrab = pgrabbed->_pgrabbedbody.lock();
            if( !!pbodygrab ) {
                if( body.GetEnv() == body.GetEnv() ) {
                    body._AttachBody(pbodygrab);
                    body._vGrabbedBodies.push_back(*itgrabbed);
                }
                else {
                    // pgrabbed points to a different environment, so have to re-initialize
                    KinBodyPtr pnewbody = body.GetEnv()->GetBodyFromEnvironmentId(pbodygrab->GetEnvironmentId());
                    if( pbodygrab->GetKinematicsGeometryHash() != pnewbody->GetKinematicsGeometryHash() ) {
                        RAVELOG_WARN(str(boost::format("body %s is not similar across environments")%pbodygrab->GetName()));
                    }
                    else {
                        GrabbedPtr pnewgrabbed(new Grabbed(pnewbody,body.GetLinks().at(KinBody::LinkPtr(pgrabbed->_plinkrobot)->GetIndex())));
                        pnewgrabbed->_troot = pgrabbed->_troot;
                        pnewgrabbed->_listNonCollidingLinks.clear();
                        FOREACHC(itlinkref, pgrabbed->_listNonCollidingLinks) {
                            pnewgrabbed->_listNonCollidingLinks.push_back(body.GetLinks().at((*itlinkref)->GetIndex()));
                        }
                        body._AttachBody(pnewbody);
                        body._vGrabbedBodies.push_back(pnewgrabbed);
                    }
                }
            }
        }

        // if not calling SetLinkTransformations, then manually call _UpdateGrabbedBodies
        if( !(_options & Save_LinkTransformation ) ) {
            body._UpdateGrabbedBodies();
        }
    }
    if( _options & Save_LinkTransformation ) {
        body.SetLinkTransformations(_vLinkTransforms, _vdoflastsetvalues);
//        if( IS_DEBUGLEVEL(Level_Warn) ) {
//            stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
//            ss << "restoring kinbody " << body.GetName() << " to values=[";
//            std::vector<dReal> values;
//            body.GetDOFValues(values);
//            FOREACH(it,values) {
//                ss << *it << ", ";
//            }
//            ss << "]";
//            RAVELOG_WARN(ss.str());
//        }
    }
    if( _options & Save_LinkEnable ) {
        // should first enable before calling the parameter callbacks
        bool bchanged = false;
        for(size_t i = 0; i < _vEnabledLinks.size(); ++i) {
            if( body.GetLinks().at(i)->IsEnabled() != !!_vEnabledLinks[i] ) {
                body.GetLinks().at(i)->info_.is_enabled_ = !!_vEnabledLinks[i];
                bchanged = true;
            }
        }
        if( bchanged ) {
            body.non_adjacent_link_cache_ &= ~AO_Enabled;
            body._PostprocessChangedParameters(Prop_LinkEnable);
        }
    }
    if( _options & Save_JointMaxVelocityAndAcceleration ) {
        body.SetDOFVelocityLimits(_vMaxVelocities);
        body.SetDOFAccelerationLimits(_vMaxAccelerations);
        body.SetDOFJerkLimits(_vMaxJerks);
    }
    if( _options & Save_LinkVelocities ) {
        body.SetLinkVelocities(_vLinkVelocities);
    }
    if( _options & Save_JointWeights ) {
        body.SetDOFWeights(_vDOFWeights);
    }
}

} // end namespace OpenRAVE
