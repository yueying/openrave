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
	void Grabbed::ProcessCollidingLinks(const std::set<int>& robot_links_to_ignore)
	{
		robot_links_to_ignore_ = robot_links_to_ignore;
		non_colliding_links_.clear();
		link_is_non_colliding_.clear();
		KinBodyPtr grabbed_body(grabbed_body_);
		KinBodyPtr body = RaveInterfaceCast<KinBody>(link_robot_->GetParent());
		EnvironmentBasePtr env = body->GetEnv();
		CollisionCheckerBasePtr checker = body->GetSelfCollisionChecker();
		if (!checker) 
		{
			checker = env->GetCollisionChecker();
		}
		CollisionOptionsStateSaver colsaver(checker, 0); // have to reset the collision options
		{
			// have to enable all the links in order to compute accurate link_is_non_colliding_ info
			KinBody::KinBodyStateSaver grabbedbodysaver(grabbed_body, KinBody::Save_LinkEnable);
			grabbed_body->Enable(true);
			KinBody::KinBodyStateSaver robotsaver(body, KinBody::Save_LinkEnable);
			body->Enable(true);

			//uint64_t starttime = utils::GetMicroTime();

			// check collision with all links to see which are valid
			int num_checked = 0;
			for(auto& itlink: body->GetLinks()) 
			{
				int non_colliding = 0;
				if (std::find(attached_links_.begin(), attached_links_.end(), itlink) == attached_links_.end()) 
				{
					if (robot_links_to_ignore.find(itlink->GetIndex()) == robot_links_to_ignore.end()) 
					{
						++num_checked;
						//uint64_t localstarttime = utils::GetMicroTime();
						if (!checker->CheckCollision(KinBody::LinkConstPtr(itlink), grabbed_body)) 
						{
							non_colliding = 1;
						}
						//RAVELOG_DEBUG_FORMAT("check %s col %s %s %fs", checker->GetXMLId()%(*itlink)->GetName()%grabbed_body->GetName()%(1e-6*(utils::GetMicroTime()-localstarttime)));
					}
				}
				link_is_non_colliding_[itlink] = non_colliding;
			}

			//uint64_t starttime1 = utils::GetMicroTime();

			std::vector<KinBody::LinkPtr > body_attached_links;
			for(auto& itgrabbed: body->grabbed_bodies_) 
			{
				std::shared_ptr<Grabbed const> grabbed = std::dynamic_pointer_cast<Grabbed const>(itgrabbed);
				bool is_same_link = std::find(attached_links_.begin(), attached_links_.end(), grabbed->link_robot_) 
					!= attached_links_.end();
				KinBodyPtr other_grabbed_body = grabbed->grabbed_body_.lock();
				if (!other_grabbed_body)
				{
					RAVELOG_WARN_FORMAT("grabbed body on %s has already been released. ignoring.", body->GetName());
					continue;
				}
				if (is_same_link) 
				{
					other_grabbed_body->GetLinks().at(0)->GetRigidlyAttachedLinks(body_attached_links);
				}
				if (other_grabbed_body != grabbed_body)
				{
					KinBody::KinBodyStateSaver othergrabbedbodysaver(other_grabbed_body, KinBody::Save_LinkEnable);
					other_grabbed_body->Enable(true);
					for(auto& itlink: other_grabbed_body->GetLinks())
					{
						int noncolliding = 0;
						if (is_same_link && std::find(body_attached_links.begin(), body_attached_links.end(), itlink)
							!= body_attached_links.end()) 
						{
						}
						else if (!checker->CheckCollision(KinBody::LinkConstPtr(itlink), grabbed_body))
						{
							noncolliding = 1;
						}
						link_is_non_colliding_[itlink] = noncolliding;
					}
				}
			}

			//uint64_t starttime2 = utils::GetMicroTime();
			//RAVELOG_DEBUG_FORMAT("env=%d, process links %d %fs %fs", probot->GetEnv()->GetId()%num_checked%(1e-6*(starttime1-starttime))%(1e-6*(starttime2-starttime)));
		}

		if (grabbed_body->IsEnabled()) 
		{
			for(auto& itnoncolliding: link_is_non_colliding_)
			{
				if (itnoncolliding.second && itnoncolliding.first->IsEnabled())
				{
					//RAVELOG_VERBOSE(str(boost::format("non-colliding link %s for grabbed body %s")%(*itlink)->GetName()%grabbed_body->GetName()));
					non_colliding_links_.push_back(itnoncolliding.first);
				}
			}
		}
	}

	void Grabbed::AddMoreIgnoreLinks(const std::set<int>& robot_links_to_ignore)
	{
		KinBodyPtr body = RaveInterfaceCast<KinBody>(link_robot_->GetParent());
		for(auto& itignoreindex: robot_links_to_ignore) 
		{
			robot_links_to_ignore_.insert(itignoreindex);
			KinBody::LinkPtr link = body->GetLinks().at(itignoreindex);
			link_is_non_colliding_[link] = 0;
			non_colliding_links_.remove(link);
		}
	}

	/// return -1 for unknown, 0 for no, 1 for yes
	int Grabbed::WasLinkNonColliding(KinBody::LinkConstPtr link) const
	{
		std::map<KinBody::LinkConstPtr, int>::const_iterator it = link_is_non_colliding_.find(link);
		if (it != link_is_non_colliding_.end())
		{
			return it->second;
		}
		return -1;
	}

	void Grabbed::UpdateCollidingLinks()
	{
		KinBodyPtr body = RaveInterfaceCast<KinBody>(link_robot_->GetParent());
		if (!body) 
		{
			return;
		}
		EnvironmentBasePtr env = body->GetEnv();
		KinBodyConstPtr grabbed_body(grabbed_body_);
		if (!grabbed_body || !grabbed_body->IsEnabled()) 
		{
			non_colliding_links_.clear();
			return;
		}

		CollisionOptionsStateSaverPtr colsaver;
		CollisionCheckerBasePtr checker = body->GetSelfCollisionChecker();
		if (!checker) 
		{
			checker = env->GetCollisionChecker();
		}

		std::map<KinBody::LinkConstPtr, int>::iterator itnoncolliding;
		std::vector<KinBody::LinkPtr > body_attached_links;
		for(auto& itgrabbed: body->grabbed_bodies_) 
		{
			std::shared_ptr<Grabbed const> grabbed = std::dynamic_pointer_cast<Grabbed const>(itgrabbed);
			bool is_same_link = std::find(attached_links_.begin(), attached_links_.end(), grabbed->link_robot_)
				!= attached_links_.end();
			KinBodyPtr other_grabbed_body = grabbed->grabbed_body_.lock();
			if (!other_grabbed_body) 
			{
				RAVELOG_WARN_FORMAT("grabbed body on %s has already been destroyed, ignoring.", body->GetName());
				continue;
			}
			if (other_grabbed_body != grabbed_body && other_grabbed_body->GetLinks().size() > 0)
			{
				if (is_same_link) 
				{
					other_grabbed_body->GetLinks().at(0)->GetRigidlyAttachedLinks(body_attached_links);
				}
				KinBody::KinBodyStateSaverPtr othergrabbedbodysaver;
				for(auto& itgrabbedlink: other_grabbed_body->GetLinks())
				{
					itnoncolliding = link_is_non_colliding_.find(itgrabbedlink);
					if (itnoncolliding == link_is_non_colliding_.end())
					{
						if (is_same_link && std::find(body_attached_links.begin(), body_attached_links.end(), itgrabbedlink) 
							!= body_attached_links.end()) 
						{
							link_is_non_colliding_[itgrabbedlink] = 0;
						}
						else 
						{
							// new body?
							if (!colsaver) 
							{
								// have to reset the collision options
								colsaver.reset(new CollisionOptionsStateSaver(checker, 0));
							}
							if (!othergrabbedbodysaver) 
							{
								othergrabbedbodysaver.reset(new KinBody::KinBodyStateSaver(other_grabbed_body, KinBody::Save_LinkEnable));
								other_grabbed_body->Enable(true);
							}
							link_is_non_colliding_[itgrabbedlink] = !checker->CheckCollision(KinBody::LinkConstPtr(itgrabbedlink), grabbed_body);
						}
					}
				}
			}
		}

		std::set<KinBodyConstPtr> grabbed_set;
		for(auto& itgrabbed: body->grabbed_bodies_)
		{
			std::shared_ptr<Grabbed const> grabbed = std::dynamic_pointer_cast<Grabbed const>(itgrabbed);
			KinBodyConstPtr other_grabbed_body = grabbed->grabbed_body_.lock();
			if (!!other_grabbed_body)
			{
				grabbed_set.insert(other_grabbed_body);
			}
			else 
			{
				RAVELOG_WARN_FORMAT("grabbed body on %s has already been destroyed, ignoring.", body->GetName());
			}
		}

		non_colliding_links_.clear();
		itnoncolliding = link_is_non_colliding_.begin();
		while (itnoncolliding != link_is_non_colliding_.end()) 
		{
			KinBodyPtr noncollidingparent = itnoncolliding->first->GetParent(true);
			if (!noncollidingparent)
			{
				link_is_non_colliding_.erase(itnoncolliding++);
				continue;
			}
			if (noncollidingparent != body)
			{
				// check if body is currently being grabbed
				if (grabbed_set.find(noncollidingparent) == grabbed_set.end())
				{
					link_is_non_colliding_.erase(itnoncolliding++);
					continue;
				}
			}

			if (itnoncolliding->second && itnoncolliding->first->IsEnabled())
			{
				non_colliding_links_.push_back(itnoncolliding->first);
			}
			++itnoncolliding;
		}
	}
}