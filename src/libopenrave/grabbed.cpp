// Copyright (C) 2006-2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#include <openrave/grabbed.h>
#include "libopenrave.h"

namespace OpenRAVE
{
	void Grabbed::ProcessCollidingLinks(const std::set<int>& setRobotLinksToIgnore)
	{
		_setRobotLinksToIgnore = setRobotLinksToIgnore;
		_listNonCollidingLinks.clear();
		_mapLinkIsNonColliding.clear();
		KinBodyPtr pgrabbedbody(_pgrabbedbody);
		KinBodyPtr pbody = RaveInterfaceCast<KinBody>(_plinkrobot->GetParent());
		EnvironmentBasePtr penv = pbody->GetEnv();
		CollisionCheckerBasePtr pchecker = pbody->GetSelfCollisionChecker();
		if (!pchecker) {
			pchecker = penv->GetCollisionChecker();
		}
		CollisionOptionsStateSaver colsaver(pchecker, 0); // have to reset the collision options
		{
			// have to enable all the links in order to compute accurate _mapLinkIsNonColliding info
			KinBody::KinBodyStateSaver grabbedbodysaver(pgrabbedbody, KinBody::Save_LinkEnable);
			pgrabbedbody->Enable(true);
			KinBody::KinBodyStateSaver robotsaver(pbody, KinBody::Save_LinkEnable);
			pbody->Enable(true);

			//uint64_t starttime = utils::GetMicroTime();

			// check collision with all links to see which are valid
			int numchecked = 0;
			FOREACHC(itlink, pbody->GetLinks()) {
				int noncolliding = 0;
				if (find(_vattachedlinks.begin(), _vattachedlinks.end(), *itlink) == _vattachedlinks.end()) {
					if (setRobotLinksToIgnore.find((*itlink)->GetIndex()) == setRobotLinksToIgnore.end()) {
						++numchecked;
						//uint64_t localstarttime = utils::GetMicroTime();
						if (!pchecker->CheckCollision(KinBody::LinkConstPtr(*itlink), pgrabbedbody)) {
							noncolliding = 1;
						}
						//RAVELOG_DEBUG_FORMAT("check %s col %s %s %fs", pchecker->GetXMLId()%(*itlink)->GetName()%pgrabbedbody->GetName()%(1e-6*(utils::GetMicroTime()-localstarttime)));
					}
				}
				_mapLinkIsNonColliding[*itlink] = noncolliding;
			}

			//uint64_t starttime1 = utils::GetMicroTime();

			std::vector<KinBody::LinkPtr > vbodyattachedlinks;
			FOREACHC(itgrabbed, pbody->_vGrabbedBodies) {
				std::shared_ptr<Grabbed const> pgrabbed = std::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
				bool bsamelink = find(_vattachedlinks.begin(), _vattachedlinks.end(), pgrabbed->_plinkrobot) != _vattachedlinks.end();
				KinBodyPtr pothergrabbedbody = pgrabbed->_pgrabbedbody.lock();
				if (!pothergrabbedbody) {
					RAVELOG_WARN_FORMAT("grabbed body on %s has already been released. ignoring.", pbody->GetName());
					continue;
				}
				if (bsamelink) {
					pothergrabbedbody->GetLinks().at(0)->GetRigidlyAttachedLinks(vbodyattachedlinks);
				}
				if (pothergrabbedbody != pgrabbedbody) {
					KinBody::KinBodyStateSaver othergrabbedbodysaver(pothergrabbedbody, KinBody::Save_LinkEnable);
					pothergrabbedbody->Enable(true);
					FOREACHC(itlink, pothergrabbedbody->GetLinks()) {
						int noncolliding = 0;
						if (bsamelink && find(vbodyattachedlinks.begin(), vbodyattachedlinks.end(), *itlink) != vbodyattachedlinks.end()) {
						}
						else if (!pchecker->CheckCollision(KinBody::LinkConstPtr(*itlink), pgrabbedbody)) {
							noncolliding = 1;
						}
						_mapLinkIsNonColliding[*itlink] = noncolliding;
					}
				}
			}

			//uint64_t starttime2 = utils::GetMicroTime();
			//RAVELOG_DEBUG_FORMAT("env=%d, process links %d %fs %fs", probot->GetEnv()->GetId()%numchecked%(1e-6*(starttime1-starttime))%(1e-6*(starttime2-starttime)));
		}

		if (pgrabbedbody->IsEnabled()) {
			FOREACH(itnoncolliding, _mapLinkIsNonColliding) {
				if (itnoncolliding->second && itnoncolliding->first->IsEnabled()) {
					//RAVELOG_VERBOSE(str(boost::format("non-colliding link %s for grabbed body %s")%(*itlink)->GetName()%pgrabbedbody->GetName()));
					_listNonCollidingLinks.push_back(itnoncolliding->first);
				}
			}
		}
	}

	void Grabbed::AddMoreIgnoreLinks(const std::set<int>& setRobotLinksToIgnore)
	{
		KinBodyPtr pbody = RaveInterfaceCast<KinBody>(_plinkrobot->GetParent());
		FOREACHC(itignoreindex, setRobotLinksToIgnore) {
			_setRobotLinksToIgnore.insert(*itignoreindex);
			KinBody::LinkPtr plink = pbody->GetLinks().at(*itignoreindex);
			_mapLinkIsNonColliding[plink] = 0;
			_listNonCollidingLinks.remove(plink);
		}
	}

	/// return -1 for unknown, 0 for no, 1 for yes
	int Grabbed::WasLinkNonColliding(KinBody::LinkConstPtr plink) const
	{
		std::map<KinBody::LinkConstPtr, int>::const_iterator it = _mapLinkIsNonColliding.find(plink);
		if (it != _mapLinkIsNonColliding.end()) {
			return it->second;
		}
		return -1;
	}

	void Grabbed::UpdateCollidingLinks()
	{
		KinBodyPtr pbody = RaveInterfaceCast<KinBody>(_plinkrobot->GetParent());
		if (!pbody) {
			return;
		}
		EnvironmentBasePtr penv = pbody->GetEnv();
		KinBodyConstPtr pgrabbedbody(_pgrabbedbody);
		if (!pgrabbedbody || !pgrabbedbody->IsEnabled()) {
			_listNonCollidingLinks.clear();
			return;
		}

		CollisionOptionsStateSaverPtr colsaver;
		CollisionCheckerBasePtr pchecker = pbody->GetSelfCollisionChecker();
		if (!pchecker) {
			pchecker = penv->GetCollisionChecker();
		}

		std::map<KinBody::LinkConstPtr, int>::iterator itnoncolliding;
		std::vector<KinBody::LinkPtr > vbodyattachedlinks;
		FOREACHC(itgrabbed, pbody->_vGrabbedBodies) {
			std::shared_ptr<Grabbed const> pgrabbed = std::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
			bool bsamelink = find(_vattachedlinks.begin(), _vattachedlinks.end(), pgrabbed->_plinkrobot) != _vattachedlinks.end();
			KinBodyPtr pothergrabbedbody = pgrabbed->_pgrabbedbody.lock();
			if (!pothergrabbedbody) {
				RAVELOG_WARN_FORMAT("grabbed body on %s has already been destroyed, ignoring.", pbody->GetName());
				continue;
			}
			if (pothergrabbedbody != pgrabbedbody && pothergrabbedbody->GetLinks().size() > 0) {
				if (bsamelink) {
					pothergrabbedbody->GetLinks().at(0)->GetRigidlyAttachedLinks(vbodyattachedlinks);
				}
				KinBody::KinBodyStateSaverPtr othergrabbedbodysaver;
				FOREACHC(itgrabbedlink, pothergrabbedbody->GetLinks()) {
					itnoncolliding = _mapLinkIsNonColliding.find(*itgrabbedlink);
					if (itnoncolliding == _mapLinkIsNonColliding.end()) {
						if (bsamelink && find(vbodyattachedlinks.begin(), vbodyattachedlinks.end(), *itgrabbedlink) != vbodyattachedlinks.end()) {
							_mapLinkIsNonColliding[*itgrabbedlink] = 0;
						}
						else {
							// new body?
							if (!colsaver) {
								// have to reset the collision options
								colsaver.reset(new CollisionOptionsStateSaver(pchecker, 0));
							}
							if (!othergrabbedbodysaver) {
								othergrabbedbodysaver.reset(new KinBody::KinBodyStateSaver(pothergrabbedbody, KinBody::Save_LinkEnable));
								pothergrabbedbody->Enable(true);
							}
							_mapLinkIsNonColliding[*itgrabbedlink] = !pchecker->CheckCollision(KinBody::LinkConstPtr(*itgrabbedlink), pgrabbedbody);
						}
					}
				}
			}
		}

		std::set<KinBodyConstPtr> _setgrabbed;
		FOREACHC(itgrabbed, pbody->_vGrabbedBodies) {
			std::shared_ptr<Grabbed const> pgrabbed = std::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
			KinBodyConstPtr pothergrabbedbody = pgrabbed->_pgrabbedbody.lock();
			if (!!pothergrabbedbody) {
				_setgrabbed.insert(pothergrabbedbody);
			}
			else {
				RAVELOG_WARN_FORMAT("grabbed body on %s has already been destroyed, ignoring.", pbody->GetName());
			}
		}

		_listNonCollidingLinks.clear();
		itnoncolliding = _mapLinkIsNonColliding.begin();
		while (itnoncolliding != _mapLinkIsNonColliding.end()) {
			KinBodyPtr noncollidingparent = itnoncolliding->first->GetParent(true);
			if (!noncollidingparent) {
				_mapLinkIsNonColliding.erase(itnoncolliding++);
				continue;
			}
			if (noncollidingparent != pbody) {
				// check if body is currently being grabbed
				if (_setgrabbed.find(noncollidingparent) == _setgrabbed.end()) {
					_mapLinkIsNonColliding.erase(itnoncolliding++);
					continue;
				}
			}

			if (itnoncolliding->second && itnoncolliding->first->IsEnabled()) {
				_listNonCollidingLinks.push_back(itnoncolliding->first);
			}
			++itnoncolliding;
		}
	}
}