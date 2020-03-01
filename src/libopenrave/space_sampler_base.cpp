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
#include <openrave/custom_sampler_callback_data.h>

namespace OpenRAVE
{

	UserDataPtr SpaceSamplerBase::RegisterStatusCallback(const StatusCallbackFn& callbackfn)
	{
		CustomSamplerCallbackDataPtr pdata(new CustomSamplerCallbackData(callbackfn, shared_sampler()));
		pdata->_iterator = __listRegisteredCallbacks.insert(__listRegisteredCallbacks.end(), pdata);
		return pdata;
	}

	int SpaceSamplerBase::_CallStatusFunctions(int sampleiteration)
	{
		int ret = 0;
		for(auto it: __listRegisteredCallbacks) 
		{
			CustomSamplerCallbackDataPtr pitdata = std::dynamic_pointer_cast<CustomSamplerCallbackData>(it.lock());
			if (!!pitdata) 
			{
				ret |= pitdata->_callbackfn(sampleiteration);
			}
		}
		return ret;
	}

}