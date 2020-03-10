// -*- coding: utf-8 -*-
#ifndef OPENRAVE_FCL_SPACE
#define OPENRAVE_FCL_SPACE

#include <boost/shared_ptr.hpp>
#include <memory> // c++11
#include <vector>
#include <openrave/openrave_macros.h>
#include <openrave/logging.h>
#include <openrave/openrave_exception.h>
#include "plugindefs.h"

namespace fclrave 
{

typedef KinBody::LinkConstPtr LinkConstPtr;
typedef std::pair<LinkConstPtr, LinkConstPtr> LinkPair;
typedef std::weak_ptr<const KinBody> KinBodyConstWeakPtr;
using OpenRAVE::ORE_Assert;
using OpenRAVE::dReal;

// Warning : this is the only place where we use std::shared_ptr (for compatibility with fcl)
typedef std::shared_ptr<fcl::CollisionGeometry<dReal>> CollisionGeometryPtr;
typedef std::shared_ptr<fcl::CollisionObject<dReal>> CollisionObjectPtr;
typedef boost::function<CollisionGeometryPtr (std::vector<fcl::Vector3<dReal>> const &points,
	std::vector<fcl::Triangle> const &triangles) > MeshFactory;
typedef std::vector<fcl::CollisionObject<dReal> *> CollisionGroup;
typedef std::shared_ptr<CollisionGroup> CollisionGroupPtr;
typedef std::pair<Transform, CollisionObjectPtr> TransformCollisionPair;


// Helper functions for conversions from OpenRAVE to FCL

Vector ConvertVectorFromFCL(fcl::Vector3<dReal> const &v)
{
    return Vector(v[0], v[1], v[2]);
}

fcl::Vector3<dReal> ConvertVectorToFCL(Vector const &v)
{
    return fcl::Vector3<dReal>(v.x, v.y, v.z);
}

fcl::Quaternion<dReal> ConvertQuaternionToFCL(Vector const &v)
{
    return fcl::Quaternion<dReal>(v[0], v[1], v[2], v[3]);
}

Vector ConvertQuaternionFromFCL(fcl::Quaternion<dReal> const &v) 
{
    return Vector(v.w(), v.x(), v.y(), v.z());
}

fcl::AABB<dReal> ConvertAABBToFcl(const OpenRAVE::AABB& bv)
{
    return fcl::AABB<dReal>(fcl::AABB<dReal>(ConvertVectorToFCL(bv.pos)), ConvertVectorToFCL(bv.extents));
}


template <class T>
CollisionGeometryPtr ConvertMeshToFCL(std::vector<fcl::Vector3<dReal>> const &points,
	std::vector<fcl::Triangle> const &triangles)
{
    std::shared_ptr< fcl::BVHModel<T> > const model = std::make_shared<fcl::BVHModel<T> >();
    model->beginModel(triangles.size(), points.size());
    model->addSubModel(points, triangles);
    model->endModel();
    return model;
}

/// \brief fcl spaces manages the individual collision objects and sets up callbacks to track their changes.
///
/// It does not know or manage the broadphase manager
class FCLSpace : public std::enable_shared_from_this<FCLSpace>
{
public:
    inline std::weak_ptr<FCLSpace> weak_space()
	{
        return shared_from_this();
    }

    // corresponds to FCLUserData
    class KinBodyInfo : public std::enable_shared_from_this<KinBodyInfo>, public OpenRAVE::UserData
    {
public:
        class LinkInfo
        {
public:
            LinkInfo() : is_from_kinbody_link_(false) 
			{
            }

            LinkInfo(KinBody::LinkPtr plink) 
				: link_(plink), 
				is_from_kinbody_link_(true) 
			{
            }

            virtual ~LinkInfo() 
			{
                Reset();
            }

            void Reset() 
			{
                if( !!linkBV.second )
				{
                    if( !!GetLink() ) 
					{
                        RAVELOG_VERBOSE_FORMAT("env=%d, resetting link %s:%s col=0x%x", 
							GetLink()->GetParent()->GetEnv()->GetId()
							%GetLink()->GetParent()->GetName()
							%GetLink()->GetName()%(uint64_t)linkBV.second.get());
                    }
                    else 
					{
                        RAVELOG_VERBOSE_FORMAT("resetting unknown link col=0x%x", (uint64_t)linkBV.second.get());
                    }
                    linkBV.second->setUserData(nullptr); // reset the user data since someone can hold a ref to the collision object and continue using it
                }
                linkBV.second.reset();

                for(auto& itgeompair: geometries_vector_) 
				{
                    itgeompair.second->setUserData(nullptr);
                    itgeompair.second.reset();
                }
                geometries_vector_.resize(0);
            }

            KinBody::LinkPtr GetLink() 
			{
                return link_.lock();
            }

            KinBody::LinkWeakPtr link_;

            //int last_stamp_; //!< Tracks if the collision geometries are up to date wrt the body update stamp. This is for narrow phase collision
            TransformCollisionPair linkBV; //!< pair of the transformation and collision object corresponding to a bounding OBB for the link
            std::vector<TransformCollisionPair> geometries_vector_; //!< vector of transformations and collision object; one per geometries
            std::string body_link_name_; // for debugging purposes
            bool is_from_kinbody_link_; //!< if true, then from kinbodylink. Otherwise from standalone object that does not have any KinBody associations
        };

        KinBodyInfo() :
			last_stamp_(0),
			link_update_stamp_(0),
			geometry_update_stamp_(0),
			attached_bodies_update_stamp_(0), 
			active_dof_update_stamp_(0)
        {
        }

        virtual ~KinBodyInfo()
		{
            Reset();
        }

        void Reset()
        {
            for(auto& itlink: vlinks)
			{
                itlink->Reset();
            }
            vlinks.resize(0);
            _geometrycallback.reset();
            _geometrygroupcallback.reset();
            _linkenablecallback.reset();
        }

        KinBodyPtr GetBody()
        {
            return _pbody.lock();
        }

        KinBodyWeakPtr _pbody;
        int last_stamp_;  //!< KinBody::GetUpdateStamp() when last synchronized ("is transform up to date")
        int link_update_stamp_; //!< update stamp for link enable state (increases every time link enables change)
        int geometry_update_stamp_; //!< update stamp for geometry update state (increases every time geometry enables change)
        int attached_bodies_update_stamp_; //!< update stamp for when attached bodies change of this body
        int active_dof_update_stamp_; //!< update stamp for when active dofs change of this body

        std::vector< std::shared_ptr<LinkInfo> > vlinks; //!< info for every link of the kinbody

        OpenRAVE::UserDataPtr _bodyAttachedCallback; //!< handle for the callback called when a body is attached or detached
        OpenRAVE::UserDataPtr _activeDOFsCallback; //!< handle for the callback called when a the activeDOFs have changed
        std::list<OpenRAVE::UserDataPtr> _linkEnabledCallbacks;

        OpenRAVE::UserDataPtr _geometrycallback; //!< handle for the callback called when the current geometry of the kinbody changed ( Prop_LinkGeometry )
        OpenRAVE::UserDataPtr _geometrygroupcallback; //!< handle for the callback called when some geometry group of one of the links of this kinbody changed ( Prop_LinkGeometryGroup )
        OpenRAVE::UserDataPtr _linkenablecallback; //!< handle for the callback called when some link enable status of this kinbody has changed so that the envManager is updated ( Prop_LinkEnable )
        OpenRAVE::UserDataPtr _bodyremovedcallback; //!< handle for the callback called when the kinbody is removed from the environment, used in self-collision checkers ( Prop_BodyRemoved )

        std::string _geometrygroup; //!< name of the geometry group tracked by this kinbody info ; if empty, tracks the current geometries
    };

    typedef std::shared_ptr<KinBodyInfo> KinBodyInfoPtr;
    typedef std::shared_ptr<KinBodyInfo const> KinBodyInfoConstPtr;
    typedef std::weak_ptr<KinBodyInfo> KinBodyInfoWeakPtr;
    typedef std::shared_ptr<FCLSpace::KinBodyInfo::LinkInfo> LinkInfoPtr;
    typedef boost::function<void (KinBodyInfoPtr)> SynchronizeCallbackFn;

    FCLSpace(EnvironmentBasePtr penv, const std::string& userdatakey)
        : environment_(penv), user_data_key_(userdatakey), is_self_collision_checker_(true)
    {

        // After many test, OBB seems to be the only real option (followed by kIOS which is needed for distance checking)
        SetBVHRepresentation("OBB");
    }

    virtual ~FCLSpace()
    {
        DestroyEnvironment();
    }

    void DestroyEnvironment()
    {
        RAVELOG_VERBOSE_FORMAT("destroying fcl collision environment (env %d) (userdatakey %s)", environment_->GetId()%user_data_key_);
        FOREACH(itbody, _setInitializedBodies)
		{
            KinBodyInfoPtr pinfo = GetInfo(**itbody);
            if( !!pinfo ) {
                pinfo->Reset();
            }
        }
        _currentpinfo.clear();
        _cachedpinfo.clear();
        _setInitializedBodies.clear();
    }

    KinBodyInfoPtr InitKinBody(KinBodyConstPtr kinbody, 
		KinBodyInfoPtr kinbody_info = KinBodyInfoPtr(), 
		bool bSetToCurrentPInfo=true)
    {
        if( !kinbody_info )
		{
            kinbody_info.reset(new KinBodyInfo());
            kinbody_info->_geometrygroup = _geometrygroup;
        }

        RAVELOG_VERBOSE_FORMAT("env=%d, self=%d, init body %s (%d)",
			kinbody->GetEnv()->GetId()%is_self_collision_checker_%kinbody->GetName()%kinbody->GetEnvironmentId());
        kinbody_info->Reset();
        kinbody_info->_pbody = std::const_pointer_cast<KinBody>(kinbody);
        // make sure that synchronization do occur !
        kinbody_info->last_stamp_ = kinbody->GetUpdateStamp() - 1;

        kinbody_info->vlinks.reserve(kinbody->GetLinks().size());
        FOREACHC(itlink, kinbody->GetLinks()) {
            const KinBody::LinkPtr& plink = *itlink;
            std::shared_ptr<KinBodyInfo::LinkInfo> linkinfo(new KinBodyInfo::LinkInfo(plink));


            typedef boost::range_detail::any_iterator<KinBody::GeometryInfo, boost::forward_traversal_tag, 
				KinBody::GeometryInfo const&, std::ptrdiff_t> GeometryInfoIterator;
            fcl::AABB<double> enclosingBV;

            // Glue code for a unified access to geometries
            if(kinbody_info->_geometrygroup.size() > 0 && plink->GetGroupNumGeometries(kinbody_info->_geometrygroup) >= 0)
			{
                const std::vector<KinBody::GeometryInfoPtr>& vgeometryinfos 
					= plink->GetGeometriesFromGroup(kinbody_info->_geometrygroup);
                FOREACHC(itgeominfo, vgeometryinfos) 
				{
                    const KinBody::GeometryInfoPtr& pgeominfo = *itgeominfo;
                    if( !pgeominfo ) 
					{
                        int igeominfo = itgeominfo - vgeometryinfos.begin();
                        throw OpenRAVE::OpenRAVEException(str(boost::format("Failed to access geometry info %d for link %s:%s with geometrygroup %s")%igeominfo%plink->GetParent()->GetName()%plink->GetName()%kinbody_info->_geometrygroup), OpenRAVE::ORE_InvalidState);
                    }
                    const CollisionGeometryPtr pfclgeom = _CreateFCLGeomFromGeometryInfo(mesh_factory_, *pgeominfo);

                    if( !pfclgeom ) 
					{
                        continue;
                    }

                    // We do not set the transformation here and leave it to _Synchronize
                    CollisionObjectPtr pfclcoll = std::make_shared<fcl::CollisionObject<dReal>>(pfclgeom);
                    pfclcoll->setUserData(linkinfo.get());
                    linkinfo->geometries_vector_.push_back(TransformCollisionPair(pgeominfo->transform_, pfclcoll));

                    KinBody::Link::Geometry _tmpgeometry(std::shared_ptr<KinBody::Link>(), *pgeominfo);
                    if( itgeominfo == vgeometryinfos.begin() ) {
                        enclosingBV = ConvertAABBToFcl(_tmpgeometry.ComputeAABB(Transform()));
                    }
                    else {
                        enclosingBV += ConvertAABBToFcl(_tmpgeometry.ComputeAABB(Transform()));
                    }
                }
            }
            else {
                const std::vector<KinBody::Link::GeometryPtr> & vgeometries = plink->GetGeometries();
                FOREACHC(itgeom, vgeometries) {
                    const KinBody::GeometryInfo& geominfo = (*itgeom)->GetInfo();
                    const CollisionGeometryPtr pfclgeom = _CreateFCLGeomFromGeometryInfo(mesh_factory_, geominfo);

                    if( !pfclgeom ) {
                        continue;
                    }

                    // We do not set the transformation here and leave it to _Synchronize
                    CollisionObjectPtr pfclcoll = std::make_shared<fcl::CollisionObject<dReal>>(pfclgeom);
                    pfclcoll->setUserData(linkinfo.get());

                    linkinfo->geometries_vector_.push_back(TransformCollisionPair(geominfo.transform_, pfclcoll));

                    KinBody::Link::Geometry _tmpgeometry(std::shared_ptr<KinBody::Link>(), geominfo);
                    if( itgeom == vgeometries.begin() ) {
                        enclosingBV = ConvertAABBToFcl(_tmpgeometry.ComputeAABB(Transform()));
                    }
                    else {
                        enclosingBV += ConvertAABBToFcl(_tmpgeometry.ComputeAABB(Transform()));
                    }
                }
            }

            if( linkinfo->geometries_vector_.size() == 0 ) {
                RAVELOG_DEBUG_FORMAT("Initializing link %s/%s with 0 geometries (env %d) (userdatakey %s)",kinbody->GetName()%plink->GetName()%environment_->GetId()%user_data_key_);
            }
            else {
                CollisionGeometryPtr pfclgeomBV = std::make_shared<fcl::Box<dReal>>(enclosingBV.max_ - enclosingBV.min_);
                CollisionObjectPtr pfclcollBV = std::make_shared<fcl::CollisionObject<dReal>>(pfclgeomBV);
                Transform trans(Vector(1,0,0,0),ConvertVectorFromFCL(0.5 * (enclosingBV.min_ + enclosingBV.max_)));
                pfclcollBV->setUserData(linkinfo.get());
                linkinfo->linkBV = std::make_pair(trans, pfclcollBV);
            }

            //link->last_stamp_ = kinbody_info->last_stamp_;
            linkinfo->body_link_name_ = kinbody->GetName() + "/" + plink->GetName();
            kinbody_info->vlinks.push_back(linkinfo);
#ifdef FCLRAVE_COLLISION_OBJECTS_STATISTICS
            RAVELOG_DEBUG_FORMAT("FCLSPACECOLLISIONOBJECT|%s|%s", linkinfo->linkBV.second.get()%linkinfo->body_link_name_);
#endif
        }

        kinbody_info->_geometrycallback = kinbody->RegisterChangeCallback(KinBody::Prop_LinkGeometry,
			boost::bind(&FCLSpace::_ResetCurrentGeometryCallback,boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()),std::weak_ptr<KinBodyInfo>(kinbody_info)));
        kinbody_info->_geometrygroupcallback = kinbody->RegisterChangeCallback(KinBody::Prop_LinkGeometryGroup,
			boost::bind(&FCLSpace::_ResetGeometryGroupsCallback,boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()),std::weak_ptr<KinBodyInfo>(kinbody_info)));
        kinbody_info->_linkenablecallback = kinbody->RegisterChangeCallback(KinBody::Prop_LinkEnable, 
			boost::bind(&FCLSpace::_ResetLinkEnableCallback, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), std::weak_ptr<KinBodyInfo>(kinbody_info)));
        kinbody_info->_activeDOFsCallback = kinbody->RegisterChangeCallback(KinBody::Prop_RobotActiveDOFs,
			boost::bind(&FCLSpace::_ResetActiveDOFsCallback, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), std::weak_ptr<KinBodyInfo>(kinbody_info)));

        kinbody_info->_bodyAttachedCallback = kinbody->RegisterChangeCallback(KinBody::Prop_BodyAttached, 
			boost::bind(&FCLSpace::_ResetAttachedBodyCallback, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), std::weak_ptr<KinBodyInfo>(kinbody_info)));
        kinbody_info->_bodyremovedcallback = kinbody->RegisterChangeCallback(KinBody::Prop_BodyRemoved, 
			boost::bind(&FCLSpace::RemoveUserData, boost::bind(&OpenRAVE::utils::sptr_from<FCLSpace>, weak_space()), boost::bind(&OpenRAVE::utils::sptr_from<const KinBody>, std::weak_ptr<const KinBody>(kinbody))));

        BOOST_ASSERT(kinbody->GetEnvironmentId() != 0);
        if( bSetToCurrentPInfo ) {
            _currentpinfo[kinbody->GetEnvironmentId()] = kinbody_info;
        }
        //_cachedpinfo[kinbody->GetEnvironmentId()] what to do with the cache?
        _setInitializedBodies.insert(kinbody);

        //Do I really need to synchronize anything at that point ?
        _Synchronize(*kinbody_info, *kinbody);

        return kinbody_info;
    }

    bool HasNamedGeometry(const KinBody &body, const std::string& groupname)
	{
        // The empty string corresponds to current geometries so all kinbodies have it
        if( groupname.size() == 0 ) {
            return true;
        }
        FOREACHC(itlink, body.GetLinks()) {
            if( (*itlink)->GetGroupNumGeometries(groupname) >= 0 ) {
                return true;
            }
        }
        return false;
    }


    void SetGeometryGroup(const std::string& groupname)
    {
        // should always do this since bodies can have different geometry groups set
        _geometrygroup = groupname;
        FOREACHC(itbody, _setInitializedBodies) {
            SetBodyGeometryGroup(*itbody, groupname);
        }
    }

    const std::string& GetGeometryGroup() const
    {
        return _geometrygroup;
    }


    bool SetBodyGeometryGroup(KinBodyConstPtr pbody, const std::string& groupname) {
        if (!HasNamedGeometry(*pbody, groupname)) {
            return false;
        }

        // Save the already existing KinBodyInfoPtr for the old geometry group
        KinBodyInfoPtr poldinfo = GetInfo(*pbody);
        if( poldinfo->_geometrygroup == groupname ) {
            return true;
        }

        poldinfo->geometry_update_stamp_ += 1;
        _cachedpinfo[(pbody)->GetEnvironmentId()][poldinfo->_geometrygroup] = poldinfo;

        BOOST_ASSERT(pbody->GetEnvironmentId() != 0);

        KinBodyInfoPtr pinfo = _cachedpinfo[pbody->GetEnvironmentId()][groupname];
        if(!pinfo) {
            RAVELOG_VERBOSE_FORMAT("FCLSpace : creating geometry %s for kinbody %s (id = %d) (env = %d)", groupname%pbody->GetName()%pbody->GetEnvironmentId()%environment_->GetId());
            pinfo.reset(new KinBodyInfo);
            pinfo->_geometrygroup = groupname;
            InitKinBody(pbody, pinfo);
        }
        else {
            RAVELOG_VERBOSE_FORMAT("env=%d, switching to geometry %s for kinbody %s (id = %d)", environment_->GetId()%groupname%pbody->GetName()%pbody->GetEnvironmentId());
            // Set the current info to use the KinBodyInfoPtr associated to groupname
            _currentpinfo[pbody->GetEnvironmentId()] = pinfo;

            // Revoke the information inside the cache so that a potentially outdated object does not survive
            _cachedpinfo[(pbody)->GetEnvironmentId()].erase(groupname);
        }

        return true;
    }

    const std::string& GetBodyGeometryGroup(const KinBody &body) const {
        static const std::string empty;
        KinBodyInfoPtr pinfo = GetInfo(body);
        if( !!pinfo ) {
            return pinfo->_geometrygroup;
        } else {
            return empty;
        }
    }

    // Set the current bvhRepresentation and reinitializes all the KinbodyInfo if needed
    void SetBVHRepresentation(std::string const &type)
    {
        if( type == bvh_representation_ ) 
		{
            return;
        }

        if (type == "AABB")
		{
            bvh_representation_ = type;
            mesh_factory_ = &ConvertMeshToFCL<fcl::AABB<dReal>>;
        } else if (type == "OBB") 
		{
            bvh_representation_ = type;
            mesh_factory_ = &ConvertMeshToFCL<fcl::OBB<dReal>>;
        } else if (type == "RSS")
		{
            bvh_representation_ = type;
            mesh_factory_ = &ConvertMeshToFCL<fcl::RSS<dReal>>;
        } else if (type == "OBBRSS")
		{
            bvh_representation_ = type;
            mesh_factory_ = &ConvertMeshToFCL<fcl::OBBRSS<dReal>>;
        } else if (type == "kDOP16") 
		{
            bvh_representation_ = type;
            mesh_factory_ = &ConvertMeshToFCL< fcl::KDOP<dReal,16> >;
        } else if (type == "kDOP18") 
		{
            bvh_representation_ = type;
            mesh_factory_ = &ConvertMeshToFCL< fcl::KDOP<dReal,18> >;
        } else if (type == "kDOP24") 
		{
            bvh_representation_ = type;
            mesh_factory_ = &ConvertMeshToFCL< fcl::KDOP<dReal,24> >;
        } else if (type == "kIOS")
		{
            bvh_representation_ = type;
            mesh_factory_ = &ConvertMeshToFCL<fcl::kIOS<dReal>>;
        } else
		{
            RAVELOG_WARN(str(boost::format("Unknown BVH representation '%s', keeping '%s' representation") % type % bvh_representation_));
            return;
        }

        // reinitialize all the KinBodyInfo

        FOREACH(itbody, _setInitializedBodies) {
            KinBodyInfoPtr pinfo = GetInfo(**itbody);
            pinfo->geometry_update_stamp_++;
            InitKinBody(*itbody, pinfo);
        }
        _cachedpinfo.clear();
    }

    std::string const& GetBVHRepresentation() const {
        return bvh_representation_;
    }


    void Synchronize()
    {
        // We synchronize only the initialized bodies, which differs from oderave
        FOREACH(itbody, _setInitializedBodies) {
            Synchronize(**itbody);
        }
    }

    void Synchronize(const KinBody &body)
    {
        KinBodyInfoPtr pinfo = GetInfo(body);
        if( !pinfo ) 
		{
            return;
        }
        BOOST_ASSERT( pinfo->GetBody().get() == &body);
        _Synchronize(*pinfo, body);
    }

    void SynchronizeWithAttached(const KinBody &body)
    {
        if( body.HasAttached() ) 
		{
            std::set<KinBodyPtr> attached_kinbody_temp_set;
            body.GetAttached(attached_kinbody_temp_set);
            for(auto& itbody: attached_kinbody_temp_set) 
			{
                Synchronize(*itbody);
            }
        }
        else 
		{
            Synchronize(body);
        }
    }

    KinBodyInfoPtr GetInfo(const KinBody &body) const
    {
        int envId = body.GetEnvironmentId();
        if ( envId == 0 ) {
            RAVELOG_WARN_FORMAT("env=%d, body %s has invalid environment id 0", body.GetEnv()->GetId()%body.GetName());
            return KinBodyInfoPtr();
        }

        std::map< int, KinBodyInfoPtr >::const_iterator it = _currentpinfo.find(envId);
        if( it == _currentpinfo.end() ) {
            return KinBodyInfoPtr();
        }
        return it->second;
    }


    void RemoveUserData(KinBodyConstPtr pbody) {
        if( !!pbody ) {
            RAVELOG_VERBOSE(str(boost::format("FCL User data removed from env %d (userdatakey %s) : %s") % environment_->GetId() % user_data_key_ % pbody->GetName()));
            _setInitializedBodies.erase(pbody);
            KinBodyInfoPtr pinfo = GetInfo(*pbody);
            if( !!pinfo ) {
                pinfo->Reset();
            }
            BOOST_ASSERT(pbody->GetEnvironmentId() != 0);

            _currentpinfo.erase(pbody->GetEnvironmentId());
            _cachedpinfo.erase(pbody->GetEnvironmentId());
        }
    }


    const std::set<KinBodyConstPtr>& GetEnvBodies() const {
        return _setInitializedBodies;
    }

    inline CollisionObjectPtr GetLinkBV(const KinBody::Link &link) {
        return GetLinkBV(*link.GetParent(), link.GetIndex());
    }

    inline CollisionObjectPtr GetLinkBV(const KinBody &body, int index) {
        KinBodyInfoPtr pinfo = GetInfo(body);
        if( !!pinfo ) {
            return GetLinkBV(*pinfo, index);
        } else {
            RAVELOG_WARN(str(boost::format("KinBody %s is not initialized in fclspace %s, env %d")%body.GetName()%user_data_key_%environment_->GetId()));
            return CollisionObjectPtr();
        }
    }

    inline CollisionObjectPtr GetLinkBV(const KinBodyInfo &info, int index) {
        return info.vlinks.at(index)->linkBV.second;
    }


    inline LinkInfoPtr GetLinkInfo(const KinBody::Link &link) {
        return GetInfo(*link.GetParent())->vlinks.at(link.GetIndex());
    }

    inline void SetIsSelfCollisionChecker(bool bIsSelfCollisionChecker)
    {
        is_self_collision_checker_ = bIsSelfCollisionChecker;
    }

    inline bool IsSelfCollisionChecker() const
    {
        return is_self_collision_checker_;
    }

    inline const MeshFactory& GetMeshFactory() const {
        return mesh_factory_;
    }

private:
    static void _AddGeomInfoToBVHSubmodel(fcl::BVHModel<fcl::OBB<dReal>>& model, KinBody::GeometryInfo const &info)
    {
        const OpenRAVE::TriMesh& mesh = info.mesh_collision_;
        if (mesh.vertices.empty() || mesh.indices.empty()) {
            return;
        }

        OPENRAVE_ASSERT_OP(mesh.indices.size() % 3, ==, 0);
        size_t const num_points = mesh.vertices.size();
        size_t const num_triangles = mesh.indices.size() / 3;

        std::vector<fcl::Vector3<dReal>> fcl_points(num_points);
        for (size_t ipoint = 0; ipoint < num_points; ++ipoint) {
            Vector v = info.transform_*mesh.vertices[ipoint];
            fcl_points[ipoint] = fcl::Vector3<dReal>(v.x, v.y, v.z);
        }

        std::vector<fcl::Triangle> fcl_triangles(num_triangles);
        for (size_t itri = 0; itri < num_triangles; ++itri) {
            int const *const tri_indices = &mesh.indices[3 * itri];
            fcl_triangles[itri] = fcl::Triangle(tri_indices[0], tri_indices[1], tri_indices[2]);
        }
        model.addSubModel(fcl_points, fcl_triangles);
    }

    static TransformCollisionPair _CreateTransformCollisionPairFromOBB(fcl::OBB<dReal> const &bv) {
        CollisionGeometryPtr pbvGeom = make_shared<fcl::Box<dReal>>(bv.extent[0]*2.0f, bv.extent[1]*2.0f, bv.extent[2]*2.0f);
        CollisionObjectPtr pbvColl = std::make_shared<fcl::CollisionObject<dReal>>(pbvGeom);
        fcl::Quaternion<dReal> fclBvRot;
        fclBvRot = (bv.axis);
        Vector bvRotation = ConvertQuaternionFromFCL(fclBvRot);
        Vector bvTranslation = ConvertVectorFromFCL(bv.center());

        return std::make_pair(Transform(bvRotation, bvTranslation), pbvColl);
    }

    // what about the tests on non-zero size (eg. box extents) ?
    static CollisionGeometryPtr _CreateFCLGeomFromGeometryInfo(const MeshFactory &mesh_factory,
		const KinBody::GeometryInfo &info)
    {
        switch(info.type_) {

        case OpenRAVE::GT_None:
            return CollisionGeometryPtr();

        case OpenRAVE::GT_Box:
            return make_shared<fcl::Box<dReal>>(info.gemo_outer_extents_data_.x*2.0f,info.gemo_outer_extents_data_.y*2.0f,info.gemo_outer_extents_data_.z*2.0f);

        case OpenRAVE::GT_Sphere:
            return make_shared<fcl::Sphere<dReal>>(info.gemo_outer_extents_data_.x);

        case OpenRAVE::GT_Cylinder:
            return make_shared<fcl::Cylinder<dReal>>(info.gemo_outer_extents_data_.x, info.gemo_outer_extents_data_.y);

        case OpenRAVE::GT_Container:
        case OpenRAVE::GT_TriMesh:
        case OpenRAVE::GT_Cage:
        {
            const OpenRAVE::TriMesh& mesh = info.mesh_collision_;
            if (mesh.vertices.empty() || mesh.indices.empty()) {
                return CollisionGeometryPtr();
            }

            OPENRAVE_ASSERT_OP(mesh.indices.size() % 3, ==, 0);
            size_t const num_points = mesh.vertices.size();
            size_t const num_triangles = mesh.indices.size() / 3;

            std::vector<fcl::Vector3<dReal>> fcl_points(num_points);
            for (size_t ipoint = 0; ipoint < num_points; ++ipoint) {
                Vector v = mesh.vertices[ipoint];
                fcl_points[ipoint] = fcl::Vector3<dReal>(v.x, v.y, v.z);
            }

            std::vector<fcl::Triangle> fcl_triangles(num_triangles);
            for (size_t itri = 0; itri < num_triangles; ++itri) {
                int const *const tri_indices = &mesh.indices[3 * itri];
                fcl_triangles[itri] = fcl::Triangle(tri_indices[0], tri_indices[1], tri_indices[2]);
            }

            return mesh_factory(fcl_points, fcl_triangles);
        }

        default:
            RAVELOG_WARN(str(boost::format("FCL doesn't support geom type %d")%info.type_));
            return CollisionGeometryPtr();
        }
    }

    /// \brief pass in info.GetBody() as a reference to avoid dereferencing the weak pointer in KinBodyInfo
    void _Synchronize(KinBodyInfo& info, const KinBody& body)
    {
        //KinBodyPtr pbody = info.GetBody();
        if( info.last_stamp_ != body.GetUpdateStamp()) 
		{
            std::vector<Transform> vtrans;
            body.GetLinkTransformations(vtrans);
            info.last_stamp_ = body.GetUpdateStamp();
            BOOST_ASSERT( body.GetLinks().size() == info.vlinks.size() );
            BOOST_ASSERT( vtrans.size() == info.vlinks.size() );
            for(size_t i = 0; i < vtrans.size(); ++i) 
			{
                CollisionObjectPtr pcoll = info.vlinks[i]->linkBV.second;
                if( !pcoll ) 
				{
                    continue;
                }
                Transform pose = vtrans[i] * info.vlinks[i]->linkBV.first;
                fcl::Vector3<dReal> newPosition = ConvertVectorToFCL(pose.trans);
                fcl::Quaternion<dReal> newOrientation = ConvertQuaternionToFCL(pose.rot);

                pcoll->setTranslation(newPosition);
                pcoll->setQuatRotation(newOrientation);
                // Do not forget to recompute the AABB otherwise getAABB won't give an up to date AABB
                pcoll->computeAABB();

                //info.vlinks[i]->last_stamp_ = info.last_stamp_;
                FOREACHC(itgeomcoll, info.vlinks[i]->geometries_vector_)
				{
                    CollisionObjectPtr pcoll = (*itgeomcoll).second;
                    Transform pose = vtrans[i] * (*itgeomcoll).first;
                    fcl::Vector3<dReal> newPosition = ConvertVectorToFCL(pose.trans);
                    fcl::Quaternion<dReal> newOrientation = ConvertQuaternionToFCL(pose.rot);

                    pcoll->setTranslation(newPosition);
                    pcoll->setQuatRotation(newOrientation);
                    // Do not forget to recompute the AABB otherwise getAABB won't give an up to date AABB
                    pcoll->computeAABB();
                }
            }

            // Does this have any use ?
            // if( !!_synccallback ) {
            //     _synccallback(pinfo);
            // }
        }
    }

    /// \brief controls whether the kinbody info is removed during the destructor
    class KinBodyInfoRemover
    {
public:
        KinBodyInfoRemover(const boost::function<void()>& fn) : _fn(fn) {
            _bDoRemove = true;
        }
        ~KinBodyInfoRemover() {
            if( _bDoRemove ) {
                _fn();
            }
        }

        void ResetRemove() {
            _bDoRemove = false;
        }

private:
        boost::function<void()> _fn;
        bool _bDoRemove;
    };
    void _ResetCurrentGeometryCallback(std::weak_ptr<KinBodyInfo> _pinfo)
    {
        KinBodyInfoPtr pinfo = _pinfo.lock();
        KinBodyPtr pbody = pinfo->GetBody();
        KinBodyInfoPtr pcurrentinfo = _currentpinfo[pbody->GetEnvironmentId()];

        if( !!pinfo && pinfo == pcurrentinfo ) {//pinfo->_geometrygroup.size() == 0 ) {
            // pinfo is current set to the current one, so should InitKinBody into _currentpinfo
            //RAVELOG_VERBOSE_FORMAT("env=%d, resetting current geometry for kinbody %s geometry_update_stamp_=%d, (key %s, self=%d)", environment_->GetId()%pbody->GetName()%pinfo->geometry_update_stamp_%user_data_key_%is_self_collision_checker_);
            pinfo->geometry_update_stamp_++;
            KinBodyInfoRemover remover(boost::bind(&FCLSpace::RemoveUserData, this, pbody)); // protect
            InitKinBody(pbody, pinfo, false);
            remover.ResetRemove(); // succeeded
        }
        //_cachedpinfo[pbody->GetEnvironmentId()].erase(std::string());
    }

    void _ResetGeometryGroupsCallback(std::weak_ptr<KinBodyInfo> _pinfo)
    {
        KinBodyInfoPtr pinfo = _pinfo.lock();
        KinBodyPtr pbody = pinfo->GetBody();

        //KinBodyInfoPtr pcurrentinfo = _currentpinfo[pbody->GetEnvironmentId()];

        if( !!pinfo ) {// && pinfo->_geometrygroup.size() > 0 ) {
            //RAVELOG_VERBOSE_FORMAT("env=%d, resetting geometry groups for kinbody %s, geometry_update_stamp_=%d (key %s, self=%d)", environment_->GetId()%pbody->GetName()%pinfo->geometry_update_stamp_%user_data_key_%is_self_collision_checker_);
            pinfo->geometry_update_stamp_++;
            KinBodyInfoRemover remover(boost::bind(&FCLSpace::RemoveUserData, this, pbody)); // protect
            InitKinBody(pbody, pinfo, false);
            remover.ResetRemove(); // succeeded
        }
//        KinBodyInfoPtr pinfoCurrentGeometry = _cachedpinfo[kinbody->GetEnvironmentId()][std::string()];
//        _cachedpinfo.erase(kinbody->GetEnvironmentId());
//        if( !!pinfoCurrentGeometry ) {
//            _cachedpinfo[kinbody->GetEnvironmentId()][std::string()] = pinfoCurrentGeometry;
//        }
    }

    void _ResetLinkEnableCallback(std::weak_ptr<KinBodyInfo> _pinfo) {
        KinBodyInfoPtr pinfo = _pinfo.lock();
        if( !!pinfo ) {
            pinfo->link_update_stamp_++;
        }
    }

    void _ResetActiveDOFsCallback(std::weak_ptr<KinBodyInfo> _pinfo) {
        KinBodyInfoPtr pinfo = _pinfo.lock();
        if( !!pinfo ) {
            pinfo->active_dof_update_stamp_++;
        }
    }

    void _ResetAttachedBodyCallback(std::weak_ptr<KinBodyInfo> _pinfo) {
        KinBodyInfoPtr pinfo = _pinfo.lock();
        if( !!pinfo ) {
            pinfo->attached_bodies_update_stamp_++;
        }
    }


    EnvironmentBasePtr environment_;
    std::string user_data_key_;
    std::string _geometrygroup;
    //SynchronizeCallbackFn _synccallback;

    std::string bvh_representation_;
    MeshFactory mesh_factory_;

    std::set<KinBodyConstPtr> _setInitializedBodies; //!< Set of the kinbody initialized in this space
    std::map< int, std::map< std::string, KinBodyInfoPtr > > _cachedpinfo; //!< Associates to each body id and geometry group name the corresponding kinbody info if already initialized and not currently set as user data
    std::map< int, KinBodyInfoPtr> _currentpinfo; //!< maps kinbody environment id to the kinbodyinfo struct constaining fcl objects. The key being environment id makes it easier to compare objects without getting a handle to their pointers. Whenever a KinBodyInfoPtr goes into this map, it is removed from _cachedpinfo

    bool is_self_collision_checker_; // Currently not used
};


}

#endif
