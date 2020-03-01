// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
/** \file   libopenrave.h
    \brief  Defines the private headers for libopenrave
 */

#ifndef RAVE_LIBOPENRAVE_H
#define RAVE_LIBOPENRAVE_H

#include <openrave/openrave.h> // should be included first in order to get boost throwing openrave exceptions
#include <openrave/utils.h>

//#include <boost/math/special_functions/round.hpp>

// include boost for vc++ only (to get typeof working)
#ifdef _MSC_VER
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/set.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(auto it = (v).begin(), __itend__=(v).end(); it != __itend__; ++(it))
#define FOREACH_NOINC(it, v) for(auto it = (v).begin(), __itend__=(v).end(); it != __itend__; )

#define FOREACHC(it, v) for(auto it = (v).begin(), __itend__=(v).end(); it != __itend__; ++(it))
#define FOREACHC_NOINC(it, v) for(auto it = (v).begin(), __itend__=(v).end(); it != __itend__; )
#define RAVE_REGISTER_BOOST

#else
#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>
#include <algorithm>
#include <complex>

#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(), __itend__=(v).end(); it != __itend__; (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin())it = (v).begin(), __itend__=(v).end(); it != __itend__; )

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif

//template <typename T>
//class openraveconst_iteratorbegin : public T::const_iterator
//{
//public:
//    openraveconst_iteratorbegin(const T & v) : T::const_iterator(v.begin()), _v(v) {
//    }
//    const T & _v;
//};
//
//
//template <typename T>
//class openraveiteratorbegin : public T::iterator
//{
//public:
//    openraveiteratorbegin(const T & v) : T::iterator( const_cast<T&> (v).begin()), _v(v) {
//    }
//    const T & _v;
//};

//#define OPENRAVE_FOREACH(it,v) for( OpenRAVE::openraveiteratorbegin<typeof(v)> (it) (v); (it) != (it)._v.end(); (it)++ )
//#define OPENRAVE_FOREACHC(it,v) for( OpenRAVE::openraveconst_iteratorbegin<typeof(v)> (it) (v); (it) != (it)._v.end(); (it)++ )

#include <stdint.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>

#ifdef USE_CRLIBM
#include <crlibm.h> // robust/accurate math
#endif

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); ++(it))

#ifdef _WIN32
#elif defined(__APPLE_CC__)
#define _strnicmp strncasecmp
#define _stricmp strcasecmp
#else
#define _strnicmp strncasecmp
#define _stricmp strcasecmp

#endif

#include <boost/bind.hpp>
#include <boost/version.hpp>

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem/operations.hpp>

#if !defined(BOOST_FILESYSTEM_VERSION) || BOOST_FILESYSTEM_VERSION <= 2
namespace boost {
namespace filesystem {
inline path absolute(const path& p)
{
    return complete(p, initial_path());
}

inline path absolute(const path& p, const path& base)
{
    return complete(p, base);
}
}
}
#endif

#endif

namespace OpenRAVE {

static const dReal g_fEpsilonLinear = RavePow(g_fEpsilon,0.9);
static const dReal g_fEpsilonJointLimit = RavePow(g_fEpsilon,0.8);
static const dReal g_fEpsilonEvalJointLimit = RavePow(g_fEpsilon,0.7);

template <typename T>
class TransformSaver
{
public:
    TransformSaver(T plink) : _plink(plink) {
        _t = _plink->GetTransform();
    }
    virtual ~TransformSaver() {
        _plink->SetTransform(_t);
    }
    const Transform& GetTransform() {
        return _t;
    }
private:
    T _plink;
    Transform _t;
};

class LinkEnableSaver
{
public:
    LinkEnableSaver(KinBody::LinkPtr plink) : _plink(plink) {
        _bIsEnabled = _plink->IsEnabled();
    }
    virtual ~LinkEnableSaver()
    {
        _plink->Enable(_bIsEnabled);
    }

private:
    KinBody::LinkPtr _plink;
    bool _bIsEnabled;
};

class CallOnDestruction
{
public:
    CallOnDestruction(const std::function<void()>& fn) : _fn(fn) {
    }
    ~CallOnDestruction() {
        _fn();
    }
private:
    std::function<void()> _fn;
};

#define SERIALIZATION_PRECISION 4
template<typename T>
inline T SerializationValue(T f)
{
    return ( f > -1e-4f && f < 1e-4f ) ? static_cast<T>(0) : f; //boost::math::round(10000*f)*0.0001;
}

inline void SerializeRound(std::ostream& o, float f)
{
    o << SerializationValue(f) << " ";
}

inline void SerializeRound(std::ostream& o, double f)
{
    o << SerializationValue(f) << " ";
}

template <class T>
inline void SerializeRound(std::ostream& o, const RaveVector<T>& v)
{
    o << SerializationValue(v.x) << " " << SerializationValue(v.y) << " " << SerializationValue(v.z) << " " << SerializationValue(v.w) << " ";
}

template <class T>
inline void SerializeRound3(std::ostream& o, const RaveVector<T>& v)
{
    o << SerializationValue(v.x) << " " << SerializationValue(v.y) << " " << SerializationValue(v.z) << " ";
}

template <class T>
inline void SerializeRound(std::ostream& o, const RaveTransform<T>& t)
{
    // because we're serializing a quaternion, have to fix what side of the hypershpere it is on
    Vector v = t.rot;
    for(int i = 0; i < 4; ++i) {
        if( v[i] < g_fEpsilon ) {
            v = -v;
            break;
        }
        else if( v[i] > g_fEpsilon ) {
            break;
        }
    }
    SerializeRound(o,v);
    SerializeRound(o,t.trans);
}

template <class T>
inline void SerializeRound(std::ostream& o, const RaveTransformMatrix<T>& t)
{
    o << SerializationValue(t.m[0]) << " " << SerializationValue(t.m[4]) << " " << SerializationValue(t.m[8]) << " "
      << SerializationValue(t.m[1]) << " " << SerializationValue(t.m[5]) << " " << SerializationValue(t.m[9]) << " "
      << SerializationValue(t.m[2]) << " " << SerializationValue(t.m[6]) << " " << SerializationValue(t.m[10]) << " ";
    SerializeRound(o,t.trans);
}

inline int CountCircularBranches(dReal angle)
{
    if( angle > PI ) {
        return static_cast<int>((angle+PI)/(2*PI));
    }
    else if( angle < -PI ) {
        return static_cast<int>((angle-PI)/(2*PI));
    }
    return 0;
}

/// returns a value=angle+2*PI*N such that value is closest to testvalue
inline dReal GetClosestValueAlongCircle(dReal angle, dReal testvalue)
{
    int n = static_cast<int>((testvalue-angle)/PI);
    if( n >= 1 ) {
        return angle + static_cast<dReal>((n+1)/2)*2*PI;
    }
    else if( n <= -1 ) {
        return angle + static_cast<dReal>((n-1)/2)*2*PI;
    }
    return angle;
}

inline dReal TransformDistanceFast(const Transform& t1, const Transform& t2, dReal frotweight=1, dReal ftransweight=1)
{
    dReal e1 = (t1.rot-t2.rot).lengthsqr4();
    dReal e2 = (t1.rot+t2.rot).lengthsqr4();
    dReal e = e1 < e2 ? e1 : e2;
    return RaveSqrt((t1.trans-t2.trans).lengthsqr3() + frotweight*e);
}

inline dReal TransformDistance2(const Transform& t1, const Transform& t2, dReal frotweight=1, dReal ftransweight=1)
{
    //dReal facos = RaveAcos(min(dReal(1),RaveFabs(dot4(t1.rot,t2.rot))));
    dReal fcos1 = (t1.rot-t2.rot).lengthsqr4();
    dReal fcos2 = (t1.rot+t2.rot).lengthsqr4();
    dReal fcos = fcos1 < fcos2 ? fcos1 : fcos2;
    return (t1.trans-t2.trans).lengthsqr3() + frotweight*fcos; //*fcos;
}

int SetDOFValuesIndicesParameters(KinBodyPtr pbody, const std::vector<dReal>& values, const std::vector<int>& vindices, int options);
int SetDOFVelocitiesIndicesParameters(KinBodyPtr pbody, const std::vector<dReal>& velocities, const std::vector<int>& vindices, int options);
int CallSetStateValuesFns(const std::vector< std::pair<PlannerBase::PlannerParameters::SetStateValuesFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, const std::vector<dReal>& v, int options);

void CallGetStateFns(const std::vector< std::pair<PlannerBase::PlannerParameters::GetStateFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v);

void subtractstates(std::vector<dReal>& q1, const std::vector<dReal>& q2);



/// -1 v1 is smaller than v2
// 0 two vectors are equivalent
/// +1 v1 is greater than v2
inline int CompareRealVectors(const std::vector<dReal> & v1, const std::vector<dReal>& v2, dReal epsilon)
{
    if( v1.size() != v2.size() ) {
        return v1.size() < v2.size() ? -1 : 1;
    }
    for(size_t i = 0; i < v1.size(); ++i) {
        if( v1[i] < v2[i]-epsilon ) {
            return -1;
        }
        else if( v1[i] > v2[i]+epsilon ) {
            return 1;
        }
    }
    return 0;
}


namespace LocalXML
{
bool ParseXMLData(BaseXMLReader& reader, const char* buffer, int size);
}

#ifdef _WIN32
inline const char *strcasestr(const char *s, const char *find)
{
    register char c, sc;
    register size_t len;

    if ((c = *find++) != 0) {
        c = tolower((unsigned char)c);
        len = strlen(find);
        do {
            do {
                if ((sc = *s++) == 0) {
                    return (NULL);
                }
            } while ((char)tolower((unsigned char)sc) != c);
        } while (_strnicmp(s, find, len) != 0);
        s--;
    }
    return ((char *) s);
}
#endif

} // end OpenRAVE namespace

// need the prototypes in order to keep them free of the OpenRAVE namespace
namespace OpenRAVEXMLParser
{
class InterfaceXMLReader;
class KinBodyXMLReader;
class LinkXMLReader;
class JointXMLReader;
class ManipulatorXMLReader;
class AttachedSensorXMLReader;
class RobotXMLReader;
class EnvironmentXMLReader;
}
class Environment;

using namespace OpenRAVE;
using namespace std;

#define _(msgid) OpenRAVE::RaveGetLocalizedTextForDomain("openrave", msgid)
#endif
