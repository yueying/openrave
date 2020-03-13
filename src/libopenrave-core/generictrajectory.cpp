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
#include "ravep.h"
#include <boost/lambda/lambda.hpp>
#include <boost/lexical_cast.hpp>
#include <openrave/xmlreaders.h>

namespace OpenRAVE {

// To distinguish between binary and XML trajectory files
static const uint16_t MAGIC_NUMBER = 0x62ff;
static const uint16_t VERSION_NUMBER = 0x0002;  // Version number for serialization

static const dReal g_fEpsilonLinear = RavePow(g_fEpsilon,0.9);
static const dReal g_fEpsilonQuadratic = RavePow(g_fEpsilon,0.45); // should be 0.6...perhaps this is related to parabolic smoother epsilons?

/* Helper functions for binary trajectory file writing */
inline void WriteBinaryUInt16(std::ostream& f, uint16_t value)
{
    f.write((const char*) &value, sizeof(value));
}

inline void WriteBinaryUInt32(std::ostream& f, uint32_t value)
{
    f.write((const char*) &value, sizeof(value));
}

inline void WriteBinaryInt(std::ostream& f, int value)
{
    f.write((const char*) &value, sizeof(value));
}

inline void WriteBinaryString(std::ostream& f, const std::string& s)
{
    BOOST_ASSERT(s.length() <= std::numeric_limits<uint16_t>::max());
    const uint16_t length = (uint16_t) s.length();
    WriteBinaryUInt16(f, length);
    if (length > 0)
    {
        f.write(s.c_str(), length);
    }
}

inline void WriteBinaryVector(std::ostream&f, const std::vector<dReal>& v)
{
    // Indicate number of data points
    const uint32_t numDataPoints = v.size();
    WriteBinaryUInt32(f, numDataPoints);

    // Write vector memory block to binary file
    const uint64_t vectorLengthBytes = numDataPoints*sizeof(dReal);
    f.write((const char*) &v[0], vectorLengthBytes);
}

/* Helper functions for binary trajectory file reading */
inline bool ReadBinaryUInt16(std::istream& f, uint16_t& value)
{
    f.read((char*) &value, sizeof(value));
    return !!f;
}

inline bool ReadBinaryUInt32(std::istream& f, uint32_t& value)
{
    f.read((char*) &value, sizeof(value));
    return !!f;
}

inline bool ReadBinaryInt(std::istream& f, int& value)
{
    f.read((char*) &value, sizeof(value));
    return !!f;
}

inline bool ReadBinaryString(std::istream& f, std::string& s)
{
    uint16_t length = 0;
    ReadBinaryUInt16(f, length);
    if (length > 0)
    {
        s.resize(length);
        f.read(&s[0], length);
    }
    else
    {
        s.clear();
    }
    return !!f;
}

inline bool ReadBinaryVector(std::istream& f, std::vector<dReal>& v)
{
    // Get number of data points
    uint32_t numDataPoints = 0;
    ReadBinaryUInt32(f, numDataPoints);
    v.resize(numDataPoints);

    // Load binary directly to vector
    const uint64_t vectorLengthBytes = numDataPoints*sizeof(dReal);
    f.read((char*) &v[0], vectorLengthBytes);

    return !!f;
}

class GenericTrajectory : public TrajectoryBase
{
    std::map<std::string,int> order_map_;
public:
    GenericTrajectory(EnvironmentBasePtr penv, std::istream& sinput)
		: TrajectoryBase(penv),
		time_offset_(-1)
    {
        order_map_["deltatime"] = 0;
        order_map_["joint_snaps"] = 1;
        order_map_["affine_snaps"] = 2;
        order_map_["joint_jerks"] = 3;
        order_map_["affine_jerks"] = 4;
        order_map_["joint_accelerations"] = 5;
        order_map_["affine_accelerations"] = 6;
        order_map_["joint_velocities"] = 7;
        order_map_["affine_velocities"] = 8;
        order_map_["joint_values"] = 9;
        order_map_["affine_transform"] = 10;
        order_map_["joint_torques"] = 11;
        is_init_ = false;
        is_sampling_verified_ = false;
    }

    bool SortGroups(const ConfigurationSpecification::Group& g1, const ConfigurationSpecification::Group& g2)
    {
        size_t index1 = g1.name.find_first_of(' ');
        if( index1 == std::string::npos ) 
		{
            index1 = g1.name.size();
        }
        size_t index2 = g2.name.find_first_of(' ');
        if( index2 == std::string::npos ) 
		{
            index2 = g2.name.size();
        }
        std::map<string,int>::iterator it1 = order_map_.find(g1.name.substr(0,index1));
        std::map<string,int>::iterator it2 = order_map_.find(g2.name.substr(0,index2));
        if( it1 == order_map_.end() ) 
		{
            return it2 == order_map_.end();
        }
        if( it2 == order_map_.end())
		{
            return true;
        }
        return it1->second < it2->second;
    }

    void Init(const ConfigurationSpecification& spec)
    {
        if( is_init_  && config_specification_ == spec )
		{
            // already init
        }
        else 
		{
            //BOOST_ASSERT(spec.GetDOF()>0 && spec.IsValid()); // when deserializing, can sometimes get invalid spec, but that's ok
            is_init_ = false;
            group_interpolators_vector_.resize(0);
            group_validators_vector_.resize(0);
            derivative_offsets_vector_.resize(0);
            deriv_deriv_offsets_vector_.resize(0);
            deriv_deriv_deriv_offsets_vector_.resize(0);
            intergral_offsets_vector_.resize(0);
            config_specification_ = spec; // what if this pointer is the same?
            // order the groups based on computation order
            std::stable_sort(config_specification_.groups_vector_.begin(),
				config_specification_.groups_vector_.end(),
				boost::bind(&GenericTrajectory::SortGroups,this,_1,_2));
            time_offset_ = -1;
            for(auto& itgroup:config_specification_.groups_vector_)
			{
                if( itgroup.name == "deltatime" ) 
				{
                    time_offset_ = itgroup.offset;
                }
            }
            _InitializeGroupFunctions();
        }
        trajectory_data_vector_.resize(0);
        _vaccumtime.resize(0);
        delta_inv_time_vector_.resize(0);
        _bChanged = true;
        is_sampling_verified_ = false;
        is_init_ = true;
    }

    void Insert(size_t index, const std::vector<dReal>& data, bool bOverwrite)
    {
        BOOST_ASSERT(is_init_);
        if( data.size() == 0 ) 
		{
            return;
        }
        BOOST_ASSERT(config_specification_.GetDOF()>0);
        OPENRAVE_ASSERT_FORMAT((data.size()%config_specification_.GetDOF()) == 0, "%d does not divide dof %d", data.size()%config_specification_.GetDOF(), ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP(index*config_specification_.GetDOF(),<=,trajectory_data_vector_.size());
        if( bOverwrite && index*config_specification_.GetDOF() < trajectory_data_vector_.size() ) {
            size_t copysize = min(data.size(),trajectory_data_vector_.size()-index*config_specification_.GetDOF());
            std::copy(data.begin(),data.begin()+copysize,trajectory_data_vector_.begin()+index*config_specification_.GetDOF());
            if( copysize < data.size() ) {
                trajectory_data_vector_.insert(trajectory_data_vector_.end(),data.begin()+copysize,data.end());
            }
        }
        else {
            trajectory_data_vector_.insert(trajectory_data_vector_.begin()+index*config_specification_.GetDOF(),data.begin(),data.end());
        }
        _bChanged = true;
    }

    void Insert(size_t index, const std::vector<dReal>& data, const ConfigurationSpecification& spec, bool bOverwrite)
    {
        BOOST_ASSERT(is_init_);
        if( data.size() == 0 ) {
            return;
        }
        BOOST_ASSERT(spec.GetDOF()>0);
        OPENRAVE_ASSERT_FORMAT((data.size()%spec.GetDOF()) == 0, "%d does not divide dof %d", data.size()%spec.GetDOF(), ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP(index*config_specification_.GetDOF(),<=,trajectory_data_vector_.size());
        if( config_specification_ == spec ) {
            Insert(index,data,bOverwrite);
        }
        else {
            std::vector< std::vector<ConfigurationSpecification::Group>::const_iterator > vconvertgroups(config_specification_.groups_vector_.size());
            for(size_t i = 0; i < vconvertgroups.size(); ++i) {
                vconvertgroups[i] = spec.FindCompatibleGroup(config_specification_.groups_vector_[i]);
            }
            size_t numpoints = data.size()/spec.GetDOF();
            size_t sourceindex = 0;
            std::vector<dReal>::iterator ittargetdata;
            std::vector<dReal>::const_iterator itsourcedata;
            if( bOverwrite && index*config_specification_.GetDOF() < trajectory_data_vector_.size() ) {
                size_t copyelements = min(numpoints,trajectory_data_vector_.size()/config_specification_.GetDOF()-index);
                ittargetdata = trajectory_data_vector_.begin()+index*config_specification_.GetDOF();
                itsourcedata = data.begin();
                _ConvertData(ittargetdata,itsourcedata,vconvertgroups,spec,copyelements,false);
                sourceindex = copyelements*spec.GetDOF();
                index += copyelements;
            }
            if( sourceindex < data.size() ) {
                size_t numelements = (data.size()-sourceindex)/spec.GetDOF();
                std::vector<dReal> vtemp(numelements*config_specification_.GetDOF());
                ittargetdata = vtemp.begin();
                itsourcedata = data.begin()+sourceindex;
                _ConvertData(ittargetdata,itsourcedata,vconvertgroups,spec,numelements,true);
                trajectory_data_vector_.insert(trajectory_data_vector_.begin()+index*config_specification_.GetDOF(),vtemp.begin(),vtemp.end());
            }
            _bChanged = true;
        }
    }

    void Remove(size_t startindex, size_t endindex)
    {
        BOOST_ASSERT(is_init_);
        if( startindex == endindex ) {
            return;
        }
        BOOST_ASSERT(startindex*config_specification_.GetDOF() <= trajectory_data_vector_.size() && endindex*config_specification_.GetDOF() <= trajectory_data_vector_.size());
        OPENRAVE_ASSERT_OP(startindex,<,endindex);
        trajectory_data_vector_.erase(trajectory_data_vector_.begin()+startindex*config_specification_.GetDOF(),trajectory_data_vector_.begin()+endindex*config_specification_.GetDOF());
        _bChanged = true;
    }

    void Sample(std::vector<dReal>& data, dReal time) const
    {
        BOOST_ASSERT(is_init_);
        BOOST_ASSERT(_timeoffset>=0);
        BOOST_ASSERT(time >= 0);
        _ComputeInternal();
        OPENRAVE_ASSERT_OP_FORMAT0((int)trajectory_data_vector_.size(),>=,config_specification_.GetDOF(), 
			"trajectory needs at least one point to sample from", ORE_InvalidArguments);
        if( IS_DEBUGLEVEL(Level_Verbose) || (RaveGetDebugLevel() & Level_VerifyPlans) )
		{
            _VerifySampling();
        }
        data.resize(0);
        data.resize(config_specification_.GetDOF(),0);
        if( time >= GetDuration() ) 
		{
            std::copy(trajectory_data_vector_.end()-config_specification_.GetDOF(),trajectory_data_vector_.end(),data.begin());
        }
        else 
		{
            std::vector<dReal>::iterator it = std::lower_bound(_vaccumtime.begin(),_vaccumtime.end(),time);
            if( it == _vaccumtime.begin() ) {
                std::copy(trajectory_data_vector_.begin(),trajectory_data_vector_.begin()+config_specification_.GetDOF(),data.begin());
                data.at(time_offset_) = time;
            }
            else {
                size_t index = it-_vaccumtime.begin();
                dReal deltatime = time-_vaccumtime.at(index-1);
                dReal waypointdeltatime = trajectory_data_vector_.at(config_specification_.GetDOF()*index + time_offset_);
                // unfortunately due to floating-point error deltatime might not be in the range [0, waypointdeltatime], so double check!
                if( deltatime < 0 ) {
                    // most likely small epsilon
                    deltatime = 0;
                }
                else if( deltatime > waypointdeltatime ) {
                    deltatime = waypointdeltatime;
                }
                for(size_t i = 0; i < group_interpolators_vector_.size(); ++i) {
                    if( !!group_interpolators_vector_[i] ) {
                        group_interpolators_vector_[i](index-1,deltatime,data);
                    }
                }
                // should return the sample time relative to the last endpoint so it is easier to re-insert in the trajectory
                data.at(time_offset_) = deltatime;
            }
        }
    }

    void Sample(std::vector<dReal>& data, dReal time, 
		const ConfigurationSpecification& spec, bool reintializeData) const
    {
        BOOST_ASSERT(is_init_);
        OPENRAVE_ASSERT_OP(time_offset_,>=,0);
        OPENRAVE_ASSERT_OP(time, >=, -g_fEpsilon);
        _ComputeInternal();
        OPENRAVE_ASSERT_OP_FORMAT0((int)trajectory_data_vector_.size(),>=,config_specification_.GetDOF(), "trajectory needs at least one point to sample from", ORE_InvalidArguments);
        if( IS_DEBUGLEVEL(Level_Verbose) || (RaveGetDebugLevel() & Level_VerifyPlans) ) {
            _VerifySampling();
        }
        if( reintializeData ) {
            data.resize(0);
        }
        data.resize(spec.GetDOF(),0);
        if( time >= GetDuration() ) {
            ConfigurationSpecification::ConvertData(data.begin(),spec,trajectory_data_vector_.end()-config_specification_.GetDOF(),config_specification_,1,GetEnv());
        }
        else {
            std::vector<dReal>::iterator it = std::lower_bound(_vaccumtime.begin(),_vaccumtime.end(),time);
            if( it == _vaccumtime.begin() ) {
                ConfigurationSpecification::ConvertData(data.begin(),spec,trajectory_data_vector_.begin(),config_specification_,1,GetEnv());
            }
            else {
                // could be faster
                vector<dReal> vinternaldata(config_specification_.GetDOF(),0);
                size_t index = it-_vaccumtime.begin();
                dReal deltatime = time-_vaccumtime.at(index-1);
                dReal waypointdeltatime = trajectory_data_vector_.at(config_specification_.GetDOF()*index + time_offset_);
                // unfortunately due to floating-point error deltatime might not be in the range [0, waypointdeltatime], so double check!
                if( deltatime < 0 ) {
                    // most likely small epsilon
                    deltatime = 0;
                }
                else if( deltatime > waypointdeltatime ) {
                    deltatime = waypointdeltatime;
                }
                for(size_t i = 0; i < group_interpolators_vector_.size(); ++i) {
                    if( !!group_interpolators_vector_[i] ) {
                        group_interpolators_vector_[i](index-1,deltatime,vinternaldata);
                    }
                }
                ConfigurationSpecification::ConvertData(data.begin(),spec,vinternaldata.begin(),config_specification_,1,GetEnv());
            }
        }
    }

    const ConfigurationSpecification& GetConfigurationSpecification() const
    {
        return config_specification_;
    }

    size_t GetNumWaypoints() const
    {
        BOOST_ASSERT(is_init_);
        return trajectory_data_vector_.size()/config_specification_.GetDOF();
    }

    void GetWaypoints(size_t startindex, size_t endindex, std::vector<dReal>& data) const
    {
        BOOST_ASSERT(is_init_);
        BOOST_ASSERT(startindex<=endindex && startindex*config_specification_.GetDOF() <= trajectory_data_vector_.size() && endindex*config_specification_.GetDOF() <= trajectory_data_vector_.size());
        data.resize((endindex-startindex)*config_specification_.GetDOF(),0);
        std::copy(trajectory_data_vector_.begin()+startindex*config_specification_.GetDOF(),trajectory_data_vector_.begin()+endindex*config_specification_.GetDOF(),data.begin());
    }

    void GetWaypoints(size_t startindex, size_t endindex, std::vector<dReal>& data, const ConfigurationSpecification& spec) const
    {
        BOOST_ASSERT(is_init_);
        BOOST_ASSERT(startindex<=endindex && startindex*config_specification_.GetDOF() <= trajectory_data_vector_.size() && endindex*config_specification_.GetDOF() <= trajectory_data_vector_.size());
        data.resize(spec.GetDOF()*(endindex-startindex),0);
        if( startindex < endindex ) {
            ConfigurationSpecification::ConvertData(data.begin(),spec,trajectory_data_vector_.begin()+startindex*config_specification_.GetDOF(),config_specification_,endindex-startindex,GetEnv());
        }
    }

    size_t GetFirstWaypointIndexAfterTime(dReal time) const
    {
        BOOST_ASSERT(is_init_);
        BOOST_ASSERT(time_offset_>=0);
        _ComputeInternal();
        if( _vaccumtime.size() == 0 ) {
            return 0;
        }
        if( time < _vaccumtime.at(0) ) {
            return 0;
        }
        if( time >= _vaccumtime.at(_vaccumtime.size()-1) ) {
            return GetNumWaypoints();
        }
        std::vector<dReal>::const_iterator itaccum = std::lower_bound(_vaccumtime.begin(), _vaccumtime.end(), time);
        return itaccum-_vaccumtime.begin();
    }

    dReal GetDuration() const
    {
        BOOST_ASSERT(is_init_);
        _ComputeInternal();
        return _vaccumtime.size() > 0 ? _vaccumtime.back() : 0;
    }

    // New feature: Store trajectory file in binary
    void serialize(std::ostream& O, int options) const override
    {
        if( options & 0x8000 ) {
            TrajectoryBase::serialize(O, options);
        }
        else {
            // NOTE: Ignore 'options' argument for now

            // Write binary file header
            WriteBinaryUInt16(O, MAGIC_NUMBER);
            WriteBinaryUInt16(O, VERSION_NUMBER);

            /* Store meta-data */

            // Indicate size of meta data
            const ConfigurationSpecification& spec = this->GetConfigurationSpecification();
            const uint16_t numGroups = spec.groups_vector_.size();
            WriteBinaryUInt16(O, numGroups);

            FOREACHC(itgroup, spec.groups_vector_)
            {
                WriteBinaryString(O, itgroup->name);   // Writes group name
                WriteBinaryInt(O, itgroup->offset);    // Writes offset
                WriteBinaryInt(O, itgroup->dof);       // Writes dof
                WriteBinaryString(O, itgroup->interpolation);  // Writes interpolation
            }

            /* Store data waypoints */
            WriteBinaryVector(O, this->trajectory_data_vector_);

            WriteBinaryString(O, GetDescription());

            // Readable interfaces, added on VERSION_NUMBER=0x0002
            std::stringstream ss;
            const uint16_t numReadableInterfaces = GetReadableInterfaces().size();
            WriteBinaryUInt16(O, numReadableInterfaces);
            FOREACHC(itReadableInterface, GetReadableInterfaces()) {
                WriteBinaryString(O, itReadableInterface->first);  // xmlid

                xmlreaders::StreamXMLWriterPtr writer(new xmlreaders::StreamXMLWriter(std::string()));
                std::dynamic_pointer_cast<XMLReadable>(itReadableInterface->second)->Serialize(writer, options);
                ss.clear();
                ss.str(std::string());
                writer->Serialize(ss);
                WriteBinaryString(O, ss.str());
            }
        }
    }

    void deserialize(std::istream& I) override
    {
        // Check whether binary or XML file
        stringstream::streampos pos = I.tellg();  // Save old position
        uint16_t binaryFileHeader = 0;
        if( !ReadBinaryUInt16(I, binaryFileHeader) ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot read first 2 bytes for deserializing traj, stream might be empty "),ORE_InvalidArguments);
        }

        // Read binary trajectory files
        if (binaryFileHeader == MAGIC_NUMBER)
        {
            uint16_t versionNumber = 0;
            ReadBinaryUInt16(I, versionNumber);

            // currently supported versions: 0x0001, 0x0002
            if (versionNumber > VERSION_NUMBER || versionNumber < 0x0001)
            {
                throw OPENRAVE_EXCEPTION_FORMAT(_("unsupported trajectory format version %d "),versionNumber,ORE_InvalidArguments);
            }

            /* Read metadata */

            // Read number of groups
            uint16_t numGroups = 0;
            ReadBinaryUInt16(I, numGroups);

            is_init_ = false;
            config_specification_.groups_vector_.resize(numGroups);
            FOREACH(itgroup, config_specification_.groups_vector_)
            {
                ReadBinaryString(I, itgroup->name);             // Read group name
                ReadBinaryInt(I, itgroup->offset);              // Read offset
                ReadBinaryInt(I, itgroup->dof);                 // Read dof
                ReadBinaryString(I, itgroup->interpolation);    // Read interpolation
            }
            this->Init(config_specification_);

            /* Read trajectory data */
            ReadBinaryVector(I, this->trajectory_data_vector_);
            ReadBinaryString(I, description_);

            // clear out existing readable interfaces
            {
                const READERSMAP& readableInterfaces = GetReadableInterfaces();
                for (READERSMAP::const_iterator itReadableInterface = readableInterfaces.begin(); itReadableInterface != readableInterfaces.end(); ++itReadableInterface ) {
                    SetReadableInterface(itReadableInterface->first, XMLReadablePtr());
                }
            }
            // versions >= 0x0002 have readable interfaces
            if (versionNumber >= 0x0002)
            {
                // read readable interfaces
                uint16_t numReadableInterfaces = 0;
                ReadBinaryUInt16(I, numReadableInterfaces);
                std::string xmlid;
                std::string serializedReadableInterface;
                for (size_t readableInterfaceIndex = 0; readableInterfaceIndex < numReadableInterfaces; ++readableInterfaceIndex)
                {
                    ReadBinaryString(I, xmlid);
                    ReadBinaryString(I, serializedReadableInterface);

                    XMLReadablePtr readableInterface(new xmlreaders::StringXMLReadable(xmlid, serializedReadableInterface));
                    SetReadableInterface(xmlid, readableInterface);
                }
            }
        }
        else {
            // try XML deserialization
            I.seekg((size_t) pos);                  // Reset to initial positoin
            TrajectoryBase::deserialize(I);
        }
    }

    void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        InterfaceBase::Clone(preference,cloningoptions);
        TrajectoryBaseConstPtr r = RaveInterfaceConstCast<TrajectoryBase>(preference);
        Init(r->GetConfigurationSpecification());
        r->GetWaypoints(0,r->GetNumWaypoints(),trajectory_data_vector_);
        _bChanged = true;
    }

    void Swap(TrajectoryBasePtr rawtraj)
    {
        OPENRAVE_ASSERT_OP(GetXMLId(),==,rawtraj->GetXMLId());
        std::shared_ptr<GenericTrajectory> traj = std::dynamic_pointer_cast<GenericTrajectory>(rawtraj);
        config_specification_.Swap(traj->config_specification_);
        derivative_offsets_vector_.swap(traj->derivative_offsets_vector_);
        deriv_deriv_offsets_vector_.swap(traj->deriv_deriv_offsets_vector_);
        deriv_deriv_deriv_offsets_vector_.swap(traj->deriv_deriv_deriv_offsets_vector_);
        intergral_offsets_vector_.swap(traj->intergral_offsets_vector_);
        std::swap(time_offset_, traj->time_offset_);
        std::swap(is_init_, traj->is_init_);
        std::swap(trajectory_data_vector_, traj->trajectory_data_vector_);
        std::swap(_vaccumtime, traj->_vaccumtime);
        std::swap(delta_inv_time_vector_, traj->delta_inv_time_vector_);
        std::swap(_bChanged, traj->_bChanged);
        std::swap(is_sampling_verified_, traj->is_sampling_verified_);
        _InitializeGroupFunctions();
    }

protected:
    void _ConvertData(std::vector<dReal>::iterator ittargetdata, std::vector<dReal>::const_iterator itsourcedata, const std::vector< std::vector<ConfigurationSpecification::Group>::const_iterator >& vconvertgroups, const ConfigurationSpecification& spec, size_t numelements, bool filluninitialized)
    {
        for(size_t igroup = 0; igroup < vconvertgroups.size(); ++igroup) {
            if( vconvertgroups[igroup] != spec.groups_vector_.end() ) {
                ConfigurationSpecification::ConvertGroupData(ittargetdata+config_specification_.groups_vector_[igroup].offset, config_specification_.GetDOF(), config_specification_.groups_vector_[igroup], itsourcedata+vconvertgroups[igroup]->offset, spec.GetDOF(), *vconvertgroups[igroup],numelements,GetEnv(),filluninitialized);
            }
            else if( filluninitialized ) {
                vector<dReal> vdefaultvalues(config_specification_.groups_vector_[igroup].dof,0);
                const string& groupname = config_specification_.groups_vector_[igroup].name;
                if( groupname.size() >= 16 && groupname.substr(0,16) == "affine_transform" ) {
                    stringstream ss(groupname.substr(16));
                    string robotname;
                    int affinedofs=0;
                    ss >> robotname >> affinedofs;
                    if( !!ss ) {
                        BOOST_ASSERT((int)vdefaultvalues.size()==RaveGetAffineDOF(affinedofs));
                        RaveGetAffineDOFValuesFromTransform(vdefaultvalues.begin(),Transform(),affinedofs);
                    }
                }
                else if( groupname.size() >= 13 && groupname.substr(0,13) == "outputSignals") {
                    std::fill(vdefaultvalues.begin(), vdefaultvalues.end(), -1);
                }
                int offset = config_specification_.groups_vector_[igroup].offset;
                for(size_t ielement = 0; ielement < numelements; ++ielement, offset += config_specification_.GetDOF()) {
                    for(int j = 0; j < config_specification_.groups_vector_[igroup].dof; ++j) {
                        *(ittargetdata+offset+j) = vdefaultvalues[j];
                    }
                }
            }
        }
    }

    void _ComputeInternal() const
    {
        if( !_bChanged ) {
            return;
        }
        if( time_offset_ < 0 ) {
            _vaccumtime.resize(0);
            delta_inv_time_vector_.resize(0);
        }
        else {
            _vaccumtime.resize(GetNumWaypoints());
            delta_inv_time_vector_.resize(_vaccumtime.size());
            if( _vaccumtime.size() == 0 ) {
                return;
            }
            _vaccumtime.at(0) = trajectory_data_vector_.at(time_offset_);
            delta_inv_time_vector_.at(0) = 1/trajectory_data_vector_.at(time_offset_);
            for(size_t i = 1; i < _vaccumtime.size(); ++i) {
                dReal deltatime = trajectory_data_vector_[config_specification_.GetDOF()*i+time_offset_];
                if( deltatime < 0 ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("deltatime (%.15e) is < 0 at point %d/%d", deltatime%i%_vaccumtime.size(), ORE_InvalidState);
                }
                delta_inv_time_vector_[i] = 1/deltatime;
                _vaccumtime[i] = _vaccumtime[i-1] + deltatime;
            }
        }
        _bChanged = false;
        is_sampling_verified_ = false;
    }

    /// \brief assumes _ComputeInternal has finished
    void _VerifySampling() const
    {
        BOOST_ASSERT(!_bChanged);
        BOOST_ASSERT(is_init_);
        if( is_sampling_verified_ ) 
		{
            return;
        }
        for(size_t i = 0; i < group_interpolators_vector_.size(); ++i) {
            if( config_specification_.groups_vector_.at(i).offset != time_offset_ ) {
                if( !group_interpolators_vector_[i] ) {
                    RAVELOG_WARN(str(boost::format("unknown interpolation method '%s' for group '%s'")%config_specification_.groups_vector_.at(i).interpolation%config_specification_.groups_vector_.at(i).name));
                }
            }
        }

        for(size_t i = 0; i < config_specification_.groups_vector_.size(); ++i) {
            const string& interpolation = config_specification_.groups_vector_[i].interpolation;
            const string& name = config_specification_.groups_vector_[i].name;
            for(int j = 0; j < config_specification_.groups_vector_[i].dof; ++j) {
                if( derivative_offsets_vector_.at(config_specification_.groups_vector_[i].offset+j) < -2 && intergral_offsets_vector_.at(config_specification_.groups_vector_[i].offset+j) < -2 ) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("%s interpolation group '%s' needs derivatives/integrals for sampling"),interpolation%name,ORE_InvalidArguments);
                }
            }
        }

        if( IS_DEBUGLEVEL(Level_Debug) || (RaveGetDebugLevel() & Level_VerifyPlans) ) {
            // go through all the points
            for(size_t ipoint = 0; ipoint+1 < _vaccumtime.size(); ++ipoint) {
                dReal deltatime = _vaccumtime[ipoint+1] - _vaccumtime[ipoint];
                for(size_t i = 0; i < group_validators_vector_.size(); ++i) {
                    if( !!group_validators_vector_[i] ) {
                        group_validators_vector_[i](ipoint,deltatime);
                    }
                }
            }
        }
        is_sampling_verified_ = true;
    }

    /// \brief called in order to initialize group_interpolators_vector_ 
	///  and group_validators_vector_, derivative_offsets_vector_, intergral_offsets_vector_
    void _InitializeGroupFunctions()
    {
        // first set sizes to 0
        group_interpolators_vector_.resize(0);
        group_validators_vector_.resize(0);
        derivative_offsets_vector_.resize(0);
        deriv_deriv_offsets_vector_.resize(0);
        deriv_deriv_deriv_offsets_vector_.resize(0);
        intergral_offsets_vector_.resize(0);
        group_interpolators_vector_.resize(config_specification_.groups_vector_.size());
        group_validators_vector_.resize(config_specification_.groups_vector_.size());
        derivative_offsets_vector_.resize(config_specification_.GetDOF(),-1);
        deriv_deriv_offsets_vector_.resize(config_specification_.GetDOF(),-1);
        deriv_deriv_deriv_offsets_vector_.resize(config_specification_.GetDOF(),-1);
        intergral_offsets_vector_.resize(config_specification_.GetDOF(),-1);
        for(size_t i = 0; i < config_specification_.groups_vector_.size(); ++i)
		{
            const std::string& interpolation = config_specification_.groups_vector_[i].interpolation;
            int need_neighboring_info_num = 0;
            if( interpolation == "previous" ) 
			{
                group_interpolators_vector_[i] = boost::bind(&GenericTrajectory::_InterpolatePrevious,
					this,boost::ref(config_specification_.groups_vector_[i]),_1,_2,_3);
            }
            else if( interpolation == "next" ) 
			{
                group_interpolators_vector_[i] = boost::bind(&GenericTrajectory::_InterpolateNext,
					this,boost::ref(config_specification_.groups_vector_[i]),_1,_2,_3);
            }
            else if( interpolation == "linear" ) 
			{
                if( config_specification_.groups_vector_[i].name.size() >= 14 
					&& config_specification_.groups_vector_[i].name.substr(0,14) == "ikparam_values" ) 
				{
                    std::stringstream ss(config_specification_.groups_vector_[i].name.substr(14));
                    int niktype=0;
                    ss >> niktype;
                    group_interpolators_vector_[i] = boost::bind(&GenericTrajectory::_InterpolateLinearIk,
						this,boost::ref(config_specification_.groups_vector_[i]),_1,_2,_3,
						static_cast<IkParameterizationType>(niktype));
                    // TODO add validation for ikparam until
                }
                else 
				{
                    group_interpolators_vector_[i] = boost::bind(&GenericTrajectory::_InterpolateLinear,
						this,boost::ref(config_specification_.groups_vector_[i]),_1,_2,_3);
                    group_validators_vector_[i] = boost::bind(&GenericTrajectory::_ValidateLinear,
						this,boost::ref(config_specification_.groups_vector_[i]),_1,_2);
                }
                need_neighboring_info_num = 2;
            }
            else if( interpolation == "quadratic" ) 
			{
                if( config_specification_.groups_vector_[i].name.size() >= 14 
					&& config_specification_.groups_vector_[i].name.substr(0,14) == "ikparam_values" )
				{
                    std::stringstream ss(config_specification_.groups_vector_[i].name.substr(14));
                    int niktype=0;
                    ss >> niktype;
                    group_interpolators_vector_[i] = boost::bind(&GenericTrajectory::_InterpolateQuadraticIk,
						this,boost::ref(config_specification_.groups_vector_[i]),_1,_2,_3,
						static_cast<IkParameterizationType>(niktype));
                    // TODO add validation for ikparam until
                }
                else 
				{
                    group_interpolators_vector_[i] = boost::bind(&GenericTrajectory::_InterpolateQuadratic,this,
						boost::ref(config_specification_.groups_vector_[i]),_1,_2,_3);
                    group_validators_vector_[i] = boost::bind(&GenericTrajectory::_ValidateQuadratic,this,
						boost::ref(config_specification_.groups_vector_[i]),_1,_2);
                }
                need_neighboring_info_num = 3;
            }
            else if( interpolation == "cubic" ) 
			{
                group_interpolators_vector_[i] = boost::bind(&GenericTrajectory::_InterpolateCubic,this,
					boost::ref(config_specification_.groups_vector_[i]),_1,_2,_3);
                group_validators_vector_[i] = boost::bind(&GenericTrajectory::_ValidateCubic,this,
					boost::ref(config_specification_.groups_vector_[i]),_1,_2);
                need_neighboring_info_num = 3;
            }
            else if( interpolation == "quartic" ) 
			{
                group_interpolators_vector_[i] = boost::bind(&GenericTrajectory::_InterpolateQuartic,this,
					boost::ref(config_specification_.groups_vector_[i]),_1,_2,_3);
                group_validators_vector_[i] = boost::bind(&GenericTrajectory::_ValidateQuartic,this,
					boost::ref(config_specification_.groups_vector_[i]),_1,_2);
                need_neighboring_info_num = 3;
            }
            else if( interpolation == "quintic" )
			{
                group_interpolators_vector_[i] = boost::bind(&GenericTrajectory::_InterpolateQuintic,
					this,boost::ref(config_specification_.groups_vector_[i]),_1,_2,_3);
                group_validators_vector_[i] = boost::bind(&GenericTrajectory::_ValidateQuintic,
					this,boost::ref(config_specification_.groups_vector_[i]),_1,_2);
                need_neighboring_info_num = 3;
            }
            else if( interpolation == "sextic" ) 
			{
                group_interpolators_vector_[i] = boost::bind(&GenericTrajectory::_InterpolateSextic,
					this,boost::ref(config_specification_.groups_vector_[i]),_1,_2,_3);
                group_validators_vector_[i] = boost::bind(&GenericTrajectory::_ValidateSextic,
					this,boost::ref(config_specification_.groups_vector_[i]),_1,_2);
                need_neighboring_info_num = 3;
            }
            else if( interpolation == "" )
			{
                // if there is no interpolation, default to "next". deltatime is such a group, but that is overwritten
                group_interpolators_vector_[i] = boost::bind(&GenericTrajectory::_InterpolateNext,
					this,boost::ref(config_specification_.groups_vector_[i]),_1,_2,_3);
            }


            if( need_neighboring_info_num )
			{
                std::vector<ConfigurationSpecification::Group>::const_iterator itderiv 
					= config_specification_.FindTimeDerivativeGroup(config_specification_.groups_vector_[i]);

                // only correct derivative if interpolation is the expected one compared to config_specification_.groups_vector_[i].interpolation
                // this is necessary in order to prevent using wrong information. For example, sometimes position and velocity can both be linear, which means they are decoupled from their interpolation
                if( itderiv != config_specification_.groups_vector_.end() ) 
				{
                    if( itderiv->interpolation.size() == 0 
						|| itderiv->interpolation != ConfigurationSpecification::GetInterpolationDerivative(config_specification_.groups_vector_[i].interpolation) ) 
					{
                        // not correct interpolation, so remove from being a real derivative
                        itderiv = config_specification_.groups_vector_.end();
                    }
                }

                if( itderiv == config_specification_.groups_vector_.end() ) 
				{
                    // don't throw an error here since it is unknown if the trajectory will be sampled
                    for(int j = 0; j < config_specification_.groups_vector_[i].dof; ++j) 
					{
                        derivative_offsets_vector_[config_specification_.groups_vector_[i].offset+j]
							= -need_neighboring_info_num;
                    }
                }
                else 
				{
                    for(int j = 0; j < config_specification_.groups_vector_[i].dof; ++j)
					{
                        derivative_offsets_vector_[config_specification_.groups_vector_[i].offset+j]
							= itderiv->offset+j;
                    }
                    std::vector<ConfigurationSpecification::Group>::const_iterator itdd = config_specification_.FindTimeDerivativeGroup(*itderiv);
                    if( itdd != config_specification_.groups_vector_.end() ) 
					{
                        if( itdd->interpolation.size() == 0 || itdd->interpolation != ConfigurationSpecification::GetInterpolationDerivative(itderiv->interpolation) ) {
                            // not correct interpolation, so remove from being a real derivative
                            itdd = config_specification_.groups_vector_.end();
                        }
                    }

                    if( itdd == config_specification_.groups_vector_.end() ) 
					{
                        // don't throw an error here since it is unknown if the trajectory will be sampled
                        for(int j = 0; j < config_specification_.groups_vector_[i].dof; ++j) 
						{
                            deriv_deriv_offsets_vector_[config_specification_.groups_vector_[i].offset+j] 
								= -need_neighboring_info_num;
                        }
                    }
                    else 
					{
                        for(int j = 0; j < config_specification_.groups_vector_[i].dof; ++j) 
						{
                            deriv_deriv_offsets_vector_[config_specification_.groups_vector_[i].offset+j] 
								= itdd->offset+j;
                        }
                        std::vector<ConfigurationSpecification::Group>::const_iterator itddd 
							= config_specification_.FindTimeDerivativeGroup(*itdd);
                        if( itddd != config_specification_.groups_vector_.end() )
						{
                            if( itddd->interpolation.size() == 0 
								|| itddd->interpolation != ConfigurationSpecification::GetInterpolationDerivative(itdd->interpolation) ) {
                                // not correct interpolation, so remove from being a real derivative
                                itddd = config_specification_.groups_vector_.end();
                            }
                        }

                        if( itddd == config_specification_.groups_vector_.end() ) 
						{
                            // don't throw an error here since it is unknown if the trajectory will be sampled
                            for(int j = 0; j < config_specification_.groups_vector_[i].dof; ++j) 
							{
                                deriv_deriv_deriv_offsets_vector_[config_specification_.groups_vector_[i].offset+j] 
									= -need_neighboring_info_num;
                            }
                        }
                        else 
						{
                            for(int j = 0; j < config_specification_.groups_vector_[i].dof; ++j) 
							{
                                deriv_deriv_deriv_offsets_vector_[config_specification_.groups_vector_[i].offset+j] 
									= itddd->offset+j;
                            }
                        }
                    }
                }
                std::vector<ConfigurationSpecification::Group>::const_iterator itintegral 
					= config_specification_.FindTimeIntegralGroup(config_specification_.groups_vector_[i]);
                // TODO check interpolation param for consistency
                if( itintegral == config_specification_.groups_vector_.end() )
				{
                    // don't throw an error here since it is unknown if the trajectory will be sampled
                    for(int j = 0; j < config_specification_.groups_vector_[i].dof; ++j)
					{
                        intergral_offsets_vector_[config_specification_.groups_vector_[i].offset+j] 
							= -need_neighboring_info_num;
                    }
                }
                else 
				{
                    for(int j = 0; j < config_specification_.groups_vector_[i].dof; ++j)
					{
                        intergral_offsets_vector_[config_specification_.groups_vector_[i].offset+j] 
							= itintegral->offset+j;
                    }
                }
            }
        }
    }

    void _InterpolatePrevious(const ConfigurationSpecification::Group& g, 
		size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        size_t offset = ipoint*config_specification_.GetDOF()+g.offset;
        if( (ipoint+1)*config_specification_.GetDOF() < trajectory_data_vector_.size() )
		{
            // if point is so close the previous, then choose the next
            dReal f = delta_inv_time_vector_.at(ipoint+1)*deltatime;
            if( f > 1-g_fEpsilon ) 
			{
                offset += config_specification_.GetDOF();
            }
        }
        std::copy(trajectory_data_vector_.begin()+offset,trajectory_data_vector_.begin()+offset+g.dof,data.begin()+g.offset);
    }

    void _InterpolateNext(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        if( (ipoint+1)*config_specification_.GetDOF() < trajectory_data_vector_.size() ) 
		{
            ipoint += 1;
        }
        size_t offset = ipoint*config_specification_.GetDOF() + g.offset;
        if( deltatime <= g_fEpsilon && ipoint > 0 )
		{
            // if point is so close the previous, then choose the previous
            offset -= config_specification_.GetDOF();
        }
        std::copy(trajectory_data_vector_.begin()+offset,trajectory_data_vector_.begin()+offset+g.dof,data.begin()+g.offset);
    }

    void _InterpolateLinear(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        size_t offset = ipoint*config_specification_.GetDOF();
        int derivoffset = derivative_offsets_vector_[g.offset];
        if( derivoffset < 0 ) 
		{
            // expected derivative offset, interpolation can be wrong for circular joints
            dReal f = delta_inv_time_vector_.at(ipoint+1)*deltatime;
            for(int i = 0; i < g.dof; ++i)
			{
                data[g.offset+i] = trajectory_data_vector_[offset+g.offset+i]*(1-f) 
					+ f*trajectory_data_vector_[config_specification_.GetDOF()+offset+g.offset+i];
            }
        }
        else 
		{
            for(int i = 0; i < g.dof; ++i) 
			{
                dReal deriv0 = trajectory_data_vector_[config_specification_.GetDOF()+offset+derivoffset+i];
                data[g.offset+i] = trajectory_data_vector_[offset+g.offset+i] + deltatime*deriv0;
            }
        }
    }

    void _InterpolateLinearIk(const ConfigurationSpecification::Group& g, 
		size_t ipoint, dReal deltatime, std::vector<dReal>& data, IkParameterizationType iktype)
    {
        _InterpolateLinear(g,ipoint,deltatime,data);
        if( deltatime > g_fEpsilon ) 
		{
            size_t offset = ipoint*config_specification_.GetDOF();
            dReal f = delta_inv_time_vector_.at(ipoint+1)*deltatime;
            switch(iktype) 
			{
            case IKP_Rotation3D:
            case IKP_Transform6D:
			{
                Vector q0, q1;
                q0.Set4(&trajectory_data_vector_[offset+g.offset]);
                q1.Set4(&trajectory_data_vector_[config_specification_.GetDOF()+offset+g.offset]);
                Vector q = quatSlerp(q0,q1,f);
                data[g.offset+0] = q[0];
                data[g.offset+1] = q[1];
                data[g.offset+2] = q[2];
                data[g.offset+3] = q[3];
                break;
            }
            case IKP_TranslationDirection5D:
			{
                Vector dir0(trajectory_data_vector_[offset+g.offset+0],
					trajectory_data_vector_[offset+g.offset+1],
					trajectory_data_vector_[offset+g.offset+2]);
                Vector dir1(trajectory_data_vector_[config_specification_.GetDOF()+offset+g.offset+0],
					trajectory_data_vector_[config_specification_.GetDOF()+offset+g.offset+1],
					trajectory_data_vector_[config_specification_.GetDOF()+offset+g.offset+2]);
                Vector axisangle = dir0.cross(dir1);
                dReal fsinangle = RaveSqrt(axisangle.lengthsqr3());
                if( fsinangle > g_fEpsilon )
				{
                    axisangle *= f*RaveAsin(min(dReal(1),fsinangle))/fsinangle;
                    Vector newdir = quatRotate(quatFromAxisAngle(axisangle),dir0);
                    data[g.offset+0] = newdir[0];
                    data[g.offset+1] = newdir[1];
                    data[g.offset+2] = newdir[2];
                }
                break;
            }
            default:
                break;
            }
        }
    }

    void _InterpolateQuadratic(const ConfigurationSpecification::Group& g, 
		size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        size_t offset = ipoint*config_specification_.GetDOF();
        if( deltatime > g_fEpsilon ) 
		{
            int derivoffset = derivative_offsets_vector_[g.offset];
            if( derivoffset >= 0 )
			{
                for(int i = 0; i < g.dof; ++i) 
				{
                    // coeff*t^2 + deriv0*t + pos0
                    dReal deriv0 = trajectory_data_vector_[offset+derivoffset+i];
                    dReal deriv1 = trajectory_data_vector_[config_specification_.GetDOF()+offset+derivoffset+i];
                    dReal coeff = 0.5*delta_inv_time_vector_.at(ipoint+1)*(deriv1-deriv0);
                    data[g.offset+i] = trajectory_data_vector_[offset+g.offset+i] + deltatime*(deriv0 + deltatime*coeff);
                }
            }
            else
			{
                dReal ideltatime = delta_inv_time_vector_.at(ipoint+1);
                dReal ideltatime2 = ideltatime*ideltatime;
                int integraloffset = intergral_offsets_vector_[g.offset];
                for(int i = 0; i < g.dof; ++i) 
				{
                    // c2*t**2 + c1*t + v0
                    // c2*deltatime**2 + c1*deltatime + v0 = v1
                    // integral: c2/3*deltatime**3 + c1/2*deltatime**2 + v0*deltatime = p1-p0
                    // mult by (3/deltatime): c2*deltatime**2 + 3/2*c1*deltatime + 3*v0 = 3*(p1-p0)/deltatime
                    // subtract by original: 0.5*c1*deltatime + 2*v0 - 3*(p1-p0)/deltatime + v1 = 0
                    // c1*deltatime = 6*(p1-p0)/deltatime - 4*v0 - 2*v1
                    dReal integral0 = trajectory_data_vector_[offset+integraloffset+i];
                    dReal integral1 = trajectory_data_vector_[config_specification_.GetDOF()+offset+integraloffset+i];
                    dReal value0 = trajectory_data_vector_[offset+g.offset+i];
                    dReal value1 = trajectory_data_vector_[config_specification_.GetDOF()+offset+g.offset+i];
                    dReal c1TimesDelta = 6*(integral1-integral0)*ideltatime - 4*value0 - 2*value1;
                    dReal c1 = c1TimesDelta*ideltatime;
                    dReal c2 = (value1 - value0 - c1TimesDelta)*ideltatime2;
                    data[g.offset+i] = value0 + deltatime * (c1 + deltatime*c2);
                }
            }
        }
        else 
		{
            for(int i = 0; i < g.dof; ++i) 
			{
                data[g.offset+i] = trajectory_data_vector_[offset+g.offset+i];
            }
        }
    }

    void _InterpolateQuadraticIk(const ConfigurationSpecification::Group& g, 
		size_t ipoint, dReal deltatime, std::vector<dReal>& data, IkParameterizationType iktype)
    {
        _InterpolateQuadratic(g, ipoint, deltatime, data);
        if( deltatime > g_fEpsilon ) {
            int derivoffset = derivative_offsets_vector_[g.offset];
            size_t offset = ipoint*config_specification_.GetDOF();
            Vector q0, q0vel, q1, q1vel;
            switch(iktype) {
            case IKP_Rotation3D:
            case IKP_Transform6D: {
                q0.Set4(&trajectory_data_vector_[offset+g.offset]);
                q0vel.Set4(&trajectory_data_vector_[offset+derivoffset]);
                q1.Set4(&trajectory_data_vector_[config_specification_.GetDOF()+offset+g.offset]);
                q1vel.Set4(&trajectory_data_vector_[config_specification_.GetDOF()+offset+derivoffset]);
                Vector angularvelocity0 = quatMultiply(q0vel,quatInverse(q0))*2;
                Vector angularvelocity1 = quatMultiply(q1vel,quatInverse(q1))*2;
                Vector coeff = (angularvelocity1-angularvelocity0)*(0.5*delta_inv_time_vector_.at(ipoint+1));
                Vector vtotaldelta = angularvelocity0*deltatime + coeff*(deltatime*deltatime);
                Vector q = quatMultiply(quatFromAxisAngle(Vector(vtotaldelta.y,vtotaldelta.z,vtotaldelta.w)),q0);
                data[g.offset+0] = q[0];
                data[g.offset+1] = q[1];
                data[g.offset+2] = q[2];
                data[g.offset+3] = q[3];
                break;
            }
            case IKP_TranslationDirection5D: {
                Vector dir0, dir1, angularvelocity0, angularvelocity1;
                dir0.Set3(&trajectory_data_vector_[offset+g.offset]);
                dir1.Set3(&trajectory_data_vector_[config_specification_.GetDOF()+offset+g.offset]);
                Vector axisangle = dir0.cross(dir1);
                if( axisangle.lengthsqr3() > g_fEpsilon ) {
                    angularvelocity0.Set3(&trajectory_data_vector_[offset+derivoffset]);
                    angularvelocity1.Set3(&trajectory_data_vector_[config_specification_.GetDOF()+offset+derivoffset]);
                    Vector coeff = (angularvelocity1-angularvelocity0)*(0.5*delta_inv_time_vector_.at(ipoint+1));
                    Vector vtotaldelta = angularvelocity0*deltatime + coeff*(deltatime*deltatime);
                    Vector newdir = quatRotate(quatFromAxisAngle(vtotaldelta),dir0);
                    data[g.offset+0] = newdir[0];
                    data[g.offset+1] = newdir[1];
                    data[g.offset+2] = newdir[2];
                }
                break;
            }
            default:
                break;
            }
        }
    }

    void _InterpolateCubic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        // p = c3*t**3 + c2*t**2 + c1*t + c0
        // c3 = (v1*dt + v0*dt - 2*px)/(dt**3)
        // c2 = (3*px - 2*v0*dt - v1*dt)/(dt**2)
        // c1 = v0
        // c0 = p0
        size_t offset = ipoint*config_specification_.GetDOF();
        if( deltatime > g_fEpsilon ) {
            int derivoffset = derivative_offsets_vector_[g.offset];
            if( derivoffset >= 0 ) {
                dReal ideltatime = delta_inv_time_vector_.at(ipoint+1);
                dReal ideltatime2 = ideltatime*ideltatime;
                dReal ideltatime3 = ideltatime2*ideltatime;
                for(int i = 0; i < g.dof; ++i) {
                    // coeff*t^2 + deriv0*t + pos0
                    dReal deriv0 = trajectory_data_vector_[offset+derivoffset+i];
                    dReal deriv1 = trajectory_data_vector_[config_specification_.GetDOF()+offset+derivoffset+i];
                    dReal px = trajectory_data_vector_.at(config_specification_.GetDOF()+offset+g.offset+i) - trajectory_data_vector_[offset+g.offset+i];
                    dReal c3 = (deriv1+deriv0)*ideltatime2 - 2*px*ideltatime3;
                    dReal c2 = 3*px*ideltatime2 - (2*deriv0+deriv1)*ideltatime;
                    data[g.offset+i] = trajectory_data_vector_[offset+g.offset+i] + deltatime*(deriv0 + deltatime*(c2 + deltatime*c3));
                }
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cubic interpolation does not have all data"),ORE_InvalidArguments);
            }
        }
        else {
            for(int i = 0; i < g.dof; ++i) {
                data[g.offset+i] = trajectory_data_vector_[offset+g.offset+i];
            }
        }
    }

    void _InterpolateQuartic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        // p = c4*t**4 + c3*t**3 + c2*t**2 + c1*t + c0
        //
        // v1 = 4*c4*dt**3 + 3*c3*dt**2 + a0*dt + v0
        // a1 = 12*c4*dt**2 + 6*c3*dt + a0
        //
        // c4 = (-2*(v1-v0) + (a0 + a1)*dt)/(4*dt**3)
        // c3 = ((v1-v0)*3 - (2*a0+a1)*dt)/(3*dt**2)
        // c2 = a0/2
        // c1 = v0
        // c0 = p0
        size_t offset = ipoint*config_specification_.GetDOF();
        if( deltatime > g_fEpsilon ) {
            int derivoffset = derivative_offsets_vector_[g.offset];
            int ddoffset = deriv_deriv_offsets_vector_[g.offset];
            if( derivoffset >= 0 && ddoffset >= 0 ) {
                dReal ideltatime = delta_inv_time_vector_.at(ipoint+1);
                dReal ideltatime2 = ideltatime*ideltatime;
                dReal ideltatime3 = ideltatime2*ideltatime;
                for(int i = 0; i < g.dof; ++i) {
                    dReal deriv0 = trajectory_data_vector_[offset+derivoffset+i];
                    dReal deriv1 = trajectory_data_vector_[config_specification_.GetDOF()+offset+derivoffset+i];
                    dReal dd0 = trajectory_data_vector_[offset+ddoffset+i];
                    dReal dd1 = trajectory_data_vector_[config_specification_.GetDOF()+offset+ddoffset+i];
                    dReal c4 = -0.5*(deriv1-deriv0)*ideltatime3 + (dd0 + dd1)*ideltatime2*0.25;
                    dReal c3 = (deriv1-deriv0)*ideltatime2 - (2*dd0+dd1)*ideltatime/3.0;
                    data[g.offset+i] = trajectory_data_vector_[offset+g.offset+i] + deltatime*(deriv0 + deltatime*(0.5*dd0 + deltatime*(c3 + deltatime*c4)));
                }
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cubic interpolation does not have all data"),ORE_InvalidArguments);
            }
        }
        else {
            for(int i = 0; i < g.dof; ++i) {
                data[g.offset+i] = trajectory_data_vector_[offset+g.offset+i];
            }
        }
    }

    void _InterpolateQuintic(const ConfigurationSpecification::Group& g, 
		size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        // p0, p1, v0, v1, a0, a1, dt, t, c5, c4, c3 = symbols('p0, p1, v0, v1, a0, a1, dt, t, c5, c4, c3')
        // p = c5*t**5 + c4*t**4 + c3*t**3 + c2*t**2 + c1*t + c0
        //
        // c5*dt**5 + c4*dt**4 + c3*dt**3 + a0/2*t**2 + v0*t + p0 - p1 = 0
        // 5*c5*t**4 + 4*c4*dt**3 + 3*c3*dt**2 + a0*dt + v0 - v1 = 0
        // 20*c5*t**3 + 12*c4*dt**2 + 6*c3*dt + a0 - a1 = 0
        //
        // A = Matrix(3,3,[dt**5, dt**4, dt**3, 5*dt**4, 4*dt**3, 3*dt**2, 20*dt**3, 12*dt**2, 6*dt])
        // b = Matrix(3, 1, [p1 -a0/2*dt**2 - v0*dt - p0, v1 -a0*dt - v0, a1 - a0])
        // c5 = -0.5*a0/dt**3 + a1/(2*dt**3) - 3*v0/dt**4 - 3*v1/dt**4 - 6*p0/dt**5 + 6*p1/dt**5
        // c4 = 1.5*a0/dt**2 - a1/dt**2 + 8*v0/dt**3 + 7*v1/dt**3 + 15*p0/dt**4 - 15*p1/dt**4
        // c3 = -1.5*a0/dt + a1/(2*dt) - 6*v0/dt**2 - 4*v1/dt**2 - 10*p0/dt**3 + 10*p1/dt**3
        // c2 = a0/2
        // c1 = v0
        // c0 = p0
        size_t offset = ipoint*config_specification_.GetDOF();
        if( deltatime > g_fEpsilon ) {
            int derivoffset = derivative_offsets_vector_[g.offset];
            int ddoffset = deriv_deriv_offsets_vector_[g.offset];
            if( derivoffset >= 0 && ddoffset >= 0 ) {
                dReal ideltatime = delta_inv_time_vector_.at(ipoint+1);
                dReal ideltatime2 = ideltatime*ideltatime;
                dReal ideltatime3 = ideltatime2*ideltatime;
                dReal ideltatime4 = ideltatime2*ideltatime2;
                dReal ideltatime5 = ideltatime4*ideltatime;
                for(int i = 0; i < g.dof; ++i) {
                    dReal p0 = trajectory_data_vector_[offset+g.offset+i];
                    dReal px = trajectory_data_vector_[config_specification_.GetDOF()+offset+g.offset+i] - p0;
                    dReal deriv0 = trajectory_data_vector_[offset+derivoffset+i];
                    dReal deriv1 = trajectory_data_vector_[config_specification_.GetDOF()+offset+derivoffset+i];
                    dReal dd0 = trajectory_data_vector_[offset+ddoffset+i];
                    dReal dd1 = trajectory_data_vector_[config_specification_.GetDOF()+offset+ddoffset+i];
                    dReal c5 = (-0.5*dd0 + dd1*0.5)*ideltatime3 - (3*deriv0 + 3*deriv1)*ideltatime4 + px*6*ideltatime5;
                    dReal c4 = (1.5*dd0 - dd1)*ideltatime2 + (8*deriv0 + 7*deriv1)*ideltatime3 - px*15*ideltatime4;
                    dReal c3 = (-1.5*dd0 + dd1*0.5)*ideltatime + (-6*deriv0 - 4*deriv1)*ideltatime2 + px*10*ideltatime3;
                    data[g.offset+i] = p0 + deltatime*(deriv0 + deltatime*(0.5*dd0 + deltatime*(c3 + deltatime*(c4 + deltatime*c5))));
                }
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cubic interpolation does not have all data"),ORE_InvalidArguments);
            }
        }
        else {
            for(int i = 0; i < g.dof; ++i) {
                data[g.offset+i] = trajectory_data_vector_[offset+g.offset+i];
            }
        }
    }

    void _InterpolateSextic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        // p = c6*t**6 + c5*t**5 + c4*t**4 + c3*t**3 + c2*t**2 + c1*t + c0
        //
        // v1 = 6*c6*dt**5 + 5*c5*dt**4 + 4*c4*dt**3 + j0/2*dt**2 + a0*dt + v0
        // a1 = 30*c6*dt**4 + 20*c5*dt**3 + 12*c4*dt**2 + j0*dt + a0
        // j1 = 120*c6*dt**3 + 60*c5*dt**2 + 24*c4*dt + j0
        //
        // c6 = -a0/(2*dt**4) - a1/(2*dt**4) - j0/(12*dt**3) + j1/(12*dt**3) - v0/dt**5 + v1/dt**5
        // c5 = 8*a0/(5*dt**3) + 7*a1/(5*dt**3) + 3*j0/(10*dt**2) - j1/(5*dt**2) + 3*v0/dt**4 - 3*v1/dt**4
        // c4 = -3*a0/(2*dt**2) - a1/dt**2 - 3*j0/(8*dt) + j1/(8*dt) - 5*v0/(2*dt**3) + 5*v1/(2*dt**3)
        // c3 = j0/6
        // c2 = a0/2
        // c1 = v0
        // c0 = p0
        size_t offset = ipoint*config_specification_.GetDOF();
        if( deltatime > g_fEpsilon ) {
            int derivoffset = derivative_offsets_vector_[g.offset];
            int ddoffset = deriv_deriv_offsets_vector_[g.offset];
            int dddoffset = deriv_deriv_deriv_offsets_vector_[g.offset];
            if( derivoffset >= 0 && ddoffset >= 0 && dddoffset >= 0 ) {
                dReal ideltatime = delta_inv_time_vector_.at(ipoint+1);
                dReal ideltatime2 = ideltatime*ideltatime;
                dReal ideltatime3 = ideltatime2*ideltatime;
                dReal ideltatime4 = ideltatime2*ideltatime2;
                dReal ideltatime5 = ideltatime4*ideltatime;
                //dReal deltatime2 = deltatime*deltatime;
                //dReal deltatime3 = deltatime2*deltatime;
                //dReal deltatime4 = deltatime2*deltatime2;
                //dReal deltatime5 = deltatime4*deltatime;
                for(int i = 0; i < g.dof; ++i) {
                    dReal p0 = trajectory_data_vector_[offset+g.offset+i];
                    //dReal px = trajectory_data_vector_[config_specification_.GetDOF()+offset+g.offset+i] - p0;
                    dReal deriv0 = trajectory_data_vector_[offset+derivoffset+i];
                    dReal deriv1 = trajectory_data_vector_[config_specification_.GetDOF()+offset+derivoffset+i];
                    dReal dd0 = trajectory_data_vector_[offset+ddoffset+i];
                    dReal dd1 = trajectory_data_vector_[config_specification_.GetDOF()+offset+ddoffset+i];
                    dReal ddd0 = trajectory_data_vector_[offset+dddoffset+i];
                    dReal ddd1 = trajectory_data_vector_[config_specification_.GetDOF()+offset+dddoffset+i];
                    // matrix inverse is slow but at least it will work for now
                    // A=Matrix(3,3,[6*dt**5, 5*dt**4, 4*dt**3, 30*dt**4, 20*dt**3, 12*dt**2, 120*dt**3, 60*dt**2, 24*dt])
                    // A.inv() = [   dt**(-5), -1/(2*dt**4), 1/(12*dt**3)]
                    //           [   -3/dt**4,  7/(5*dt**3), -1/(5*dt**2)]
                    //           [5/(2*dt**3),     -1/dt**2,     1/(8*dt)]
                    // b = Matrix(3,1,[v1 -j0/2*dt**2 - a0*dt - v0, a1 -j0*dt - a0, j1 -j0])
                    //dReal A[9], b[3]; // A*[c6,c5,c4] = b
                    //A[0] = 6*deltatime5;   A[1] = 5*deltatime4;  A[2] = 4*deltatime3;  b[0] = -ddd0*0.5*deltatime2 - dd0*deltatime - deriv0;
                    //A[3] = 30*deltatime4;  A[4] = 20*deltatime3; A[5] = 12*deltatime2; b[1] = -ddd0*deltatime - dd0;
                    //A[6] = 120*deltatime3; A[7] = 60*deltatime2; A[8] = 24*deltatime;  b[2] = -ddd0;
                    //mathextra::inv3(A, deltatime5, NULL, 3);
                    //dReal c6 = A[0]*b[0] + A[1]*b[1] + A[2]*b[2];
                    //dReal c5 = A[3]*b[0] + A[4]*b[1] + A[5]*b[2];
                    //dReal c4 = A[6]*b[0] + A[7]*b[1] + A[8]*b[2];
                    dReal c6 = (-dd0 - dd1)*0.5*ideltatime4 + (-ddd0 + ddd1)/12.0*ideltatime3 + (-deriv0 + deriv1)*ideltatime5;
                    dReal c5 = (1.6*dd0 + 1.4*dd1)*ideltatime3 + (0.3*ddd0 - ddd1*0.2)*ideltatime2 + (3*deriv0 - 3*deriv1)*ideltatime4;
                    dReal c4 = (-1.5*dd0 - dd1)*ideltatime2 + (-0.375*ddd0 + ddd1*0.125)*ideltatime + (-2.5*deriv0 + 2.5*deriv1)*ideltatime3;
                    data[g.offset+i] = p0 + deltatime*(deriv0 + deltatime*(0.5*dd0 + deltatime*(ddd0/6.0 + deltatime*(c4 + deltatime*(c5 + deltatime*c6)))));
                }
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cubic interpolation does not have all data"),ORE_InvalidArguments);
            }
        }
        else {
            for(int i = 0; i < g.dof; ++i) {
                data[g.offset+i] = trajectory_data_vector_[offset+g.offset+i];
            }
        }
    }

    void _ValidateLinear(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime)
    {
        size_t offset = ipoint*config_specification_.GetDOF();
        int derivoffset = derivative_offsets_vector_[g.offset];
        if( derivoffset >= 0 ) {
            for(int i = 0; i < g.dof; ++i) {
                dReal deriv0 = trajectory_data_vector_[config_specification_.GetDOF()+offset+derivoffset+i];
                dReal expected = trajectory_data_vector_[offset+g.offset+i] + deltatime*deriv0;
                dReal error = RaveFabs(trajectory_data_vector_[config_specification_.GetDOF()+offset+g.offset+i] - expected);
                if( RaveFabs(error-2*PI) > g_fEpsilonLinear ) { // TODO, officially track circular joints
                    OPENRAVE_ASSERT_OP_FORMAT(error,<=,g_fEpsilonLinear, "trajectory segment for group %s interpolation %s points %d-%d dof %d is invalid", g.name%g.interpolation%ipoint%(ipoint+1)%i, ORE_InvalidState);
                }
            }
        }
    }

    void _ValidateQuadratic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime)
    {
        if( deltatime > g_fEpsilon ) {
            size_t offset = ipoint*config_specification_.GetDOF();
            int derivoffset = derivative_offsets_vector_[g.offset];
            if( derivoffset >= 0 ) {
                for(int i = 0; i < g.dof; ++i) {
                    // coeff*t^2 + deriv0*t + pos0
                    dReal deriv0 = trajectory_data_vector_[offset+derivoffset+i];
                    dReal coeff = 0.5*delta_inv_time_vector_.at(ipoint+1)*(trajectory_data_vector_[config_specification_.GetDOF()+offset+derivoffset+i]-deriv0);
                    dReal expected = trajectory_data_vector_[offset+g.offset+i] + deltatime*(deriv0 + deltatime*coeff);
                    dReal error = RaveFabs(trajectory_data_vector_.at(config_specification_.GetDOF()+offset+g.offset+i)-expected);
                    if( RaveFabs(error-2*PI) > 1e-5 ) { // TODO, officially track circular joints
                        OPENRAVE_ASSERT_OP_FORMAT(error,<=,1e-4, "trajectory segment for group %s interpolation %s time %f points %d-%d dof %d is invalid", g.name%g.interpolation%deltatime%ipoint%(ipoint+1)%i, ORE_InvalidState);
                    }
                }
            }
            else {
                int integraloffset = intergral_offsets_vector_[g.offset];
                BOOST_ASSERT(integraloffset>=0);
                // cannot verify since there's not enough constraints
            }
        }
    }

    void _ValidateCubic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime)
    {
        // TODO, need 3 groups to verify
    }

    void _ValidateQuartic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime)
    {
    }

    void _ValidateQuintic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime)
    {
    }

    void _ValidateSextic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime)
    {
    }

    ConfigurationSpecification config_specification_;
    std::vector< boost::function<void(size_t,dReal,std::vector<dReal>&)> > group_interpolators_vector_;
    std::vector< boost::function<void(size_t,dReal)> > group_validators_vector_;
    std::vector<int> derivative_offsets_vector_, deriv_deriv_offsets_vector_, deriv_deriv_deriv_offsets_vector_; //!< for every group that relies on other info to compute its position, this will point to the derivative offset. -1 if invalid and not needed, -2 if invalid and needed
    std::vector<int> intergral_offsets_vector_; //!< for every group that relies on other info to compute its position, this will point to the integral offset (ie the position for a velocity group). -1 if invalid and not needed, -2 if invalid and needed
    int time_offset_;

    std::vector<dReal> trajectory_data_vector_;
    mutable std::vector<dReal> _vaccumtime, delta_inv_time_vector_;
    bool is_init_;
    mutable bool _bChanged; //!< if true, then _ComputeInternal() has to be called in order to compute _vaccumtime and delta_inv_time_vector_
    mutable bool is_sampling_verified_; //!< if false, then _VerifySampling() has not be called yet to verify that all points can be sampled.
};

TrajectoryBasePtr CreateGenericTrajectory(EnvironmentBasePtr penv, std::istream& sinput)
{
    return TrajectoryBasePtr(new GenericTrajectory(penv,sinput));
}

}
