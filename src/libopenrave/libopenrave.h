﻿// -*- coding: utf-8 -*-
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

#define FOREACH(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(), __itend__=(v).end(); it != __itend__; ++(it))
#define FOREACH_NOINC(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(), __itend__=(v).end(); it != __itend__; )

#define FOREACHC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(), __itend__=(v).end(); it != __itend__; ++(it))
#define FOREACHC_NOINC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(), __itend__=(v).end(); it != __itend__; )
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

#include <openrave/openravejson.h>

#ifdef USE_CRLIBM
#include <crlibm.h> // robust/accurate math
#endif

#ifndef M_PI
#	define M_PI 3.14159265358979323846	// PI
#endif

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); ++(it))

#ifdef _WIN32
#define  strncasecmp _strnicmp
#define  strcasecmp _stricmp
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

#ifdef _MSC_VER
#include <intrin.h>

static inline int __builtin_ctz(unsigned x) {
	unsigned long ret;
	_BitScanForward(&ret, x);
	return (int)ret;
}

static inline int __builtin_ctzll(unsigned long long x) {
	unsigned long ret;
	_BitScanForward64(&ret, x);
	return (int)ret;
}

static inline int __builtin_ctzl(unsigned long x) {
	return sizeof(x) == 8 ? __builtin_ctzll(x) : __builtin_ctz((uint32_t)x);
}

static inline int __builtin_clz(unsigned x) {
	//unsigned long ret;
	//_BitScanReverse(&ret, x);
	//return (int)(31 ^ ret);
	return (int)__lzcnt(x);
}

static inline int __builtin_clzll(unsigned long long x) {
	//unsigned long ret;
	//_BitScanReverse64(&ret, x);
	//return (int)(63 ^ ret);
	return (int)__lzcnt64(x);
}

static inline int __builtin_clzl(unsigned long x) {
	return sizeof(x) == 8 ? __builtin_clzll(x) : __builtin_clz((uint32_t)x);
}

#ifdef __cplusplus
static inline int __builtin_ctzl(unsigned long long x) {
	return __builtin_ctzll(x);
}

static inline int __builtin_clzl(unsigned long long x) {
	return __builtin_clzll(x);
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
    CallOnDestruction(const boost::function<void()>& fn) : _fn(fn) {
    }
    ~CallOnDestruction() {
        _fn();
    }
private:
    boost::function<void()> _fn;
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
inline void SerializeRoundQuaternion(std::ostream& o, const RaveVector<T>& v)
{
    // This function is used only for serializing quaternions. Need to
    // take into account the fact that v and -v represent the same
    // rotation. Convert v to a rotation matrix instead to get a
    // unique representation. Then since the thrid column can be uniquely
    // determined given the first two, serializing only the first two
    // columns is sufficient for the purpose of hash computation.
    RaveTransformMatrix<T> t = matrixFromQuat(v);
    o << SerializationValue(t.m[0]) << " " << SerializationValue(t.m[4]) << " " << SerializationValue(t.m[8]) << " "
      << SerializationValue(t.m[1]) << " " << SerializationValue(t.m[5]) << " " << SerializationValue(t.m[9]) << " ";
}

template <class T>
inline void SerializeRound3(std::ostream& o, const RaveVector<T>& v)
{
    o << SerializationValue(v.x) << " " << SerializationValue(v.y) << " " << SerializationValue(v.z) << " ";
}

template <class T>
inline void SerializeRound(std::ostream& o, const RaveTransform<T>& t)
{
    SerializeRoundQuaternion(o,t.rot);
    SerializeRound3(o,t.trans);
}

template <class T>
inline void SerializeRound(std::ostream& o, const RaveTransformMatrix<T>& t)
{
    // Since the thrid column of the rotation matrix can be uniquely
    // determined given the first two, serializing only the first two
    // columns is sufficient for the purpose of hash computation.
    o << SerializationValue(t.m[0]) << " " << SerializationValue(t.m[4]) << " " << SerializationValue(t.m[8]) << " "
      << SerializationValue(t.m[1]) << " " << SerializationValue(t.m[5]) << " " << SerializationValue(t.m[9]) << " ";
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

inline bool IsZeroWithEpsilon3(const Vector v, dReal fEpsilon)
{
    return RaveFabs(v.x) <= fEpsilon && RaveFabs(v.y) <= fEpsilon && RaveFabs(v.z) <= fEpsilon;
}

inline bool IsZeroWithEpsilon4(const Vector v, dReal fEpsilon)
{
    return RaveFabs(v.x) <= fEpsilon && RaveFabs(v.y) <= fEpsilon && RaveFabs(v.z) <= fEpsilon && RaveFabs(v.w) <= fEpsilon;
}

inline dReal ComputeQuatDistance2(const Vector& quat0, const Vector& quat1)
{
    dReal e1 = (quat0-quat1).lengthsqr4();
    dReal e2 = (quat0+quat1).lengthsqr4();
    dReal e = e1 < e2 ? e1 : e2;
    return e;
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

/// \brief The information of a currently grabbed body.
class Grabbed : public UserData, public std::enable_shared_from_this<Grabbed>
{
public:
    Grabbed(KinBodyPtr pgrabbedbody, KinBody::LinkPtr plinkrobot) : _pgrabbedbody(pgrabbedbody), _plinkrobot(plinkrobot) {
        _enablecallback = pgrabbedbody->RegisterChangeCallback(KinBody::Prop_LinkEnable, boost::bind(&Grabbed::UpdateCollidingLinks, this));
        _plinkrobot->GetRigidlyAttachedLinks(_vattachedlinks);
    }
    virtual ~Grabbed() {
    }
    KinBodyWeakPtr _pgrabbedbody;         ///< the grabbed body
    KinBody::LinkPtr _plinkrobot;         ///< robot link that is grabbing the body
    std::list<KinBody::LinkConstPtr> _listNonCollidingLinks;         ///< links that are not colliding with the grabbed body at the time of Grab
    Transform _troot;         ///< root transform (of first link of body) relative to plinkrobot's transform. In other words, pbody->GetTransform() == plinkrobot->GetTransform()*troot
    std::set<int> _setRobotLinksToIgnore; ///< original links of the robot to force ignoring

    /// \brief check collision with all links to see which are valid.
    ///
    /// Use the robot's self-collision checker if possible
    /// resets all cached data and re-evaluates the collisions
    /// \param setRobotLinksToIgnore indices of the robot links to always ignore, in other words remove from non-colliding list
    void ProcessCollidingLinks(const std::set<int>& setRobotLinksToIgnore);

    inline const std::vector<KinBody::LinkPtr>& GetRigidlyAttachedLinks() const {
        return _vattachedlinks;
    }

    void AddMoreIgnoreLinks(const std::set<int>& setRobotLinksToIgnore);

    /// return -1 for unknown, 0 for no, 1 for yes
    int WasLinkNonColliding(KinBody::LinkConstPtr plink) const;

    /// \brief updates the non-colliding info while reusing the cache data from _ProcessCollidingLinks
    ///
    /// note that Regrab here is *very* dangerous since the robot could be a in a bad self-colliding state with the body. therefore, update the non-colliding state based on _mapLinkIsNonColliding
    void UpdateCollidingLinks();

private:
    std::vector<KinBody::LinkPtr> _vattachedlinks;
    UserDataPtr _enablecallback; ///< callback for grabbed body when it is enabled/disabled

    std::map<KinBody::LinkConstPtr, int> _mapLinkIsNonColliding; // the collision state for each link at the time the body was grabbed.
};

typedef std::shared_ptr<Grabbed> GrabbedPtr;
typedef std::shared_ptr<Grabbed const> GrabbedConstPtr;

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

template <typename IKReal>
inline void polyroots2(const IKReal* rawcoeffs, IKReal* rawroots, int& numroots)
{
    IKReal det = rawcoeffs[1]*rawcoeffs[1]-4*rawcoeffs[0]*rawcoeffs[2];
    if( det < 0 ) {
        numroots=0;
    }
    else if( det == 0 ) {
        rawroots[0] = -0.5*rawcoeffs[1]/rawcoeffs[0];
        numroots = 1;
    }
    else {
        det = RaveSqrt(det);
        rawroots[0] = (-rawcoeffs[1]+det)/(2*rawcoeffs[0]);
        rawroots[1] = (-rawcoeffs[1]-det)/(2*rawcoeffs[0]); //rawcoeffs[2]/(rawcoeffs[0]*rawroots[0]);
        numroots = 2;
    }
}

/// \brief Durand-Kerner polynomial root finding method
template <typename IKReal, int D>
inline void polyroots(const IKReal* rawcoeffs, IKReal* rawroots, int& numroots)
{
    using std::complex;
    BOOST_ASSERT(rawcoeffs[0] != 0);
    const IKReal tol = 128.0*std::numeric_limits<IKReal>::epsilon();
    const IKReal tolsqrt = sqrt(std::numeric_limits<IKReal>::epsilon());
    complex<IKReal> coeffs[D];
    const int maxsteps = 110;
    for(int i = 0; i < D; ++i) {
        coeffs[i] = complex<IKReal>(rawcoeffs[i+1]/rawcoeffs[0]);
    }
    complex<IKReal> roots[D];
    IKReal err[D];
    roots[0] = complex<IKReal>(1,0);
    roots[1] = complex<IKReal>(0.4,0.9); // any complex number not a root of unity works
    err[0] = 1.0;
    err[1] = 1.0;
    for(int i = 2; i < D; ++i) {
        roots[i] = roots[i-1]*roots[1];
        err[i] = 1.0;
    }
    for(int step = 0; step < maxsteps; ++step) {
        bool changed = false;
        for(int i = 0; i < D; ++i) {
            if ( err[i] >= tol ) {
                changed = true;
                // evaluate
                complex<IKReal> x = roots[i] + coeffs[0];
                for(int j = 1; j < D; ++j) {
                    x = roots[i] * x + coeffs[j];
                }
                for(int j = 0; j < D; ++j) {
                    if( i != j ) {
                        if( roots[i] != roots[j] ) {
                            x /= (roots[i] - roots[j]);
                        }
                    }
                }
                roots[i] -= x;
                err[i] = abs(x);
            }
        }
        if( !changed ) {
            break;
        }
    }

    numroots = 0;
    bool visited[D] = {false};
    for(int i = 0; i < D; ++i) {
        if( !visited[i] ) {
            // might be a multiple root, in which case it will have more error than the other roots
            // find any neighboring roots, and take the average
            complex<IKReal> newroot=roots[i];
            int n = 1;
            for(int j = i+1; j < D; ++j) {
                if( abs(roots[i]-roots[j]) < 8*tolsqrt ) {
                    newroot += roots[j];
                    n += 1;
                    visited[j] = true;
                }
            }
            if( n > 1 ) {
                newroot /= n;
            }
            // there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
            if( RaveFabs(imag(newroot)) < tolsqrt ) {
                rawroots[numroots++] = real(newroot);
            }
        }
    }
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


/// \brief Update current info from json value. Create a new one if there is no id matched.
template<typename T>
void UpdateOrCreateInfo(const rapidjson::Value& value, std::vector<std::shared_ptr<T> >& vInfos, dReal fUnitScale, int options)
{
    std::string id = OpenRAVE::orjson::GetStringJsonValueByKey(value, "id");
    bool isDeleted = OpenRAVE::orjson::GetJsonValueByKey<bool>(value, "__deleted__", false);
    typename std::vector<std::shared_ptr<T> >::iterator itExistingInfo = vInfos.end();
    if (!id.empty()) {
        // only try to find old info if id is not empty
        FOREACH(itInfo, vInfos) {
            if ((*itInfo)->_id == id) {
                itExistingInfo = itInfo;
                break;
            }
        }
    }
    // here we allow items with empty id to be created because
    // when we load things from json, some id could be missing on file
    // and for the partial update case, the id should be non-empty
    if (itExistingInfo != vInfos.end()) {
        if (isDeleted) {
            vInfos.erase(itExistingInfo);
            return;
        }
        (*itExistingInfo)->DeserializeJSON(value, fUnitScale, options);
        (*itExistingInfo)->_id = id;
        return;
    }
    if (isDeleted) {
        return;
    }
    std::shared_ptr<T> pNewInfo(new T());
    pNewInfo->DeserializeJSON(value, fUnitScale, options);
    pNewInfo->_id = id;
    vInfos.push_back(pNewInfo);
}

/// \brief Recursively call UpdateFromInfo on children. If children need to be added or removed, require re-init. Returns false if update fails and caller should not continue with other parts of the update.
template<typename InfoPtrType, typename PtrType>
bool UpdateChildrenFromInfo(const std::vector<InfoPtrType>& vInfos, std::vector<PtrType>& vPointers, UpdateFromInfoResult& result)
{
    int index = 0;
    for (typename std::vector<InfoPtrType>::const_iterator itInfo = vInfos.begin(); itInfo != vInfos.end(); ++itInfo, ++index) {
        const InfoPtrType pInfo = *itInfo;
        PtrType pMatchExistingPointer;

        {
            typename std::vector<PtrType>::iterator itExistingSameId = vPointers.end();
            typename std::vector<PtrType>::iterator itExistingSameName = vPointers.end();
            typename std::vector<PtrType>::iterator itExistingSameIdName = vPointers.end();
            typename std::vector<PtrType>::iterator itExistingNoIdName = vPointers.end();

            // search only in the unprocessed part of vPointers
            if( (int)vPointers.size() > index ) {
                for (typename std::vector<PtrType>::iterator itPointer = vPointers.begin() + index; itPointer != vPointers.end(); ++itPointer) {
                    // special case: no id or name, find next existing one that has no id or name
                    if (pInfo->GetId().empty() && pInfo->GetName().empty()) {
                        if ((*itPointer)->GetId().empty() && (*itPointer)->GetName().empty()) {
                            itExistingNoIdName = itPointer;
                            break;
                        }
                        continue;
                    }

                    bool bIdMatch = !(*itPointer)->GetId().empty() && (*itPointer)->GetId() == pInfo->GetId();
                    bool bNameMatch = !(*itPointer)->GetName().empty() && (*itPointer)->GetName() == pInfo->GetName();
                    if( bIdMatch && bNameMatch ) {
                        itExistingSameIdName = itPointer;
                        itExistingSameId = itPointer;
                        itExistingSameName = itPointer;
                        break;
                    }
                    if( bIdMatch && itExistingSameId == vPointers.end() ) {
                        itExistingSameId = itPointer;
                    }
                    if( bNameMatch && itExistingSameName == vPointers.end() ) {
                        itExistingSameName = itPointer;
                    }
                }
            }
            typename std::vector<PtrType>::iterator itExisting = itExistingSameIdName;
            if( itExisting == vPointers.end() ) {
                itExisting = itExistingSameId;
            }
            if( itExisting == vPointers.end() ) {
                itExisting = itExistingSameName;
            }
            if( itExisting == vPointers.end() ) {
                itExisting = itExistingNoIdName;
            }
            if( itExisting != vPointers.end() ) {
                pMatchExistingPointer = *itExisting;
                if (index != itExisting-vPointers.begin()) {
                    // re-arrange vPointers according to the order of infos
                    PtrType pTemp = vPointers[index];
                    vPointers[index] = pMatchExistingPointer;
                    *itExisting = pTemp;
                }
            }
        }
        if (!pMatchExistingPointer) {
            // new element, requires re-init
            result = UFIR_RequireReinitialize;
            return false;
        }

        UpdateFromInfoResult updateFromInfoResult = pMatchExistingPointer->UpdateFromInfo(*pInfo);
        if (updateFromInfoResult == UFIR_NoChange) {
            // no change
            continue;
        }

        if (updateFromInfoResult == UFIR_Success) {
            // something changd
            result = UFIR_Success;
            continue;
        }

        // update failed
        result = updateFromInfoResult;
        return false;
    }

    if (vPointers.size() > vInfos.size()) {
        // have to delete extra, require re-init
        result = UFIR_RequireReinitialize;
        return false;
    }

    return true;
}

template<typename T>
bool AreVectorsDeepEqual(const std::vector<std::shared_ptr<T> >& vFirst, const std::vector<std::shared_ptr<T> >& vSecond) {
    if (vFirst.size() != vSecond.size()) {
        return false;
    }
    for (size_t index = 0; index < vFirst.size(); index++) {
        if (!vFirst[index] || !vSecond[index]) {
            if (!!vFirst[index] || !!vSecond[index]) {
                return false;
            }
        } else {
            if ((*vFirst[index]) != (*vSecond[index])) {
                return false;
            }
        }
    }
    return true;
}

template<typename T, std::size_t N>
bool AreArraysDeepEqual(const boost::array<std::shared_ptr<T>, N>& vFirst, const boost::array<std::shared_ptr<T>, N>& vSecond) {
    if (vFirst.size() != vSecond.size()) {
        return false;
    }
    for (size_t index = 0; index < vFirst.size(); index++) {
        if (!vFirst[index] || !vSecond[index]) {
            if (!!vFirst[index] || !!vSecond[index]) {
                return false;
            }
        } else {
            if ((*vFirst[index]) != (*vSecond[index])) {
                return false;
            }
        }
    }
    return true;
}

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
