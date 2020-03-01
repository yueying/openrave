﻿// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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

#include <openrave/openrave.h>

namespace OpenRAVE
{

	class CustomSamplerCallbackData : public std::enable_shared_from_this<CustomSamplerCallbackData>, public UserData
	{
	public:
		CustomSamplerCallbackData(const SpaceSamplerBase::StatusCallbackFn& callbackfn, SpaceSamplerBasePtr sampler)
			: _callbackfn(callbackfn), _samplerweak(sampler) 
		{
		}
		virtual ~CustomSamplerCallbackData() 
		{
			SpaceSamplerBasePtr sampler = _samplerweak.lock();
			if (!!sampler) 
			{
				sampler->__listRegisteredCallbacks.erase(_iterator);
			}
		}

		SpaceSamplerBase::StatusCallbackFn _callbackfn;
		SpaceSamplerBaseWeakPtr _samplerweak;
		std::list<UserDataWeakPtr>::iterator _iterator;
	};

	typedef std::shared_ptr<CustomSamplerCallbackData> CustomSamplerCallbackDataPtr;
}