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
#ifndef OPENRAVE_GRABBED_H_
#define OPENRAVE_GRABBED_H_

#include <openrave/numerical.h>
#include <openrave/user_data.h>
#include <openrave/kinbody.h>

namespace OpenRAVE
{
	/// \brief The information of a currently grabbed body.
	class Grabbed : public UserData, public std::enable_shared_from_this<Grabbed>
	{
	public:
		Grabbed(KinBodyPtr grabbed_body, KinBody::LinkPtr link_robot) 
			: grabbed_body_(grabbed_body), link_robot_(link_robot) 
		{
			enable_callback_ = grabbed_body->RegisterChangeCallback(KinBody::Prop_LinkEnable,
				boost::bind(&Grabbed::UpdateCollidingLinks, this));
			link_robot_->GetRigidlyAttachedLinks(attached_links_);
		}
		virtual ~Grabbed()
		{
		}

		KinBodyWeakPtr grabbed_body_;         //!< the grabbed body
		KinBody::LinkPtr link_robot_;         //!< robot link that is grabbing the body
		std::list<KinBody::LinkConstPtr> non_colliding_links_;         //!< links that are not colliding with the grabbed body at the time of Grab
		Transform _troot;         //!< root transform (of first link of body) relative to link_robot's transform. In other words, pbody->GetTransform() == link_robot->GetTransform()*troot
		std::set<int> robot_links_to_ignore_; //!< original links of the robot to force ignoring

		/// \brief check collision with all links to see which are valid.
		///
		/// Use the robot's self-collision checker if possible
		/// resets all cached data and re-evaluates the collisions
		/// \param setRobotLinksToIgnore indices of the robot links to always ignore, in other words remove from non-colliding list
		void ProcessCollidingLinks(const std::set<int>& setRobotLinksToIgnore);

		inline const std::vector<KinBody::LinkPtr>& GetRigidlyAttachedLinks() const 
		{
			return attached_links_;
		}

		void AddMoreIgnoreLinks(const std::set<int>& setRobotLinksToIgnore);

		/// return -1 for unknown, 0 for no, 1 for yes
		int WasLinkNonColliding(KinBody::LinkConstPtr plink) const;

		/// \brief updates the non-colliding info while reusing the cache data from _ProcessCollidingLinks
		///
		/// note that Regrab here is *very* dangerous since the robot could be a in a bad self-colliding state with the body. therefore, update the non-colliding state based on link_is_non_colliding_
		void UpdateCollidingLinks();

	private:
		std::vector<KinBody::LinkPtr> attached_links_;
		UserDataPtr enable_callback_; //!< callback for grabbed body when it is enabled/disabled

		std::map<KinBody::LinkConstPtr, int> link_is_non_colliding_; // the collision state for each link at the time the body was grabbed.
	};

	typedef std::shared_ptr<Grabbed> GrabbedPtr;
	typedef std::shared_ptr<Grabbed const> GrabbedConstPtr;
}

#endif // OPENRAVE_GRABBED_H_