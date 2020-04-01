// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov (rosen.diankov@gmail.com)
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

#include <openrave/planningutils.h>
#include <openrave/xml_readers.h>
#include <openrave/dummy_xml_reader.h>
#include <openrave/space_sampler_base.h>

namespace OpenRAVE {

static std::string s_linearsmoother = "linearsmoother"; //"shortcut_linear";

std::istream& operator>>(std::istream& I, PlannerParameters& pp)
{
    if (!!I) {
        stringbuf buf;

        //std::istream::sentry sentry(I); // necessary?!
        stringstream::streampos pos = I.tellg();
        I.seekg(0, ios::end);
        stringstream::streampos endpos = I.tellg();
        I.seekg(pos);

        std::vector<char> vstrbuf;
        vstrbuf.reserve((size_t)(endpos - pos)); // make sure there are at least this many bytes

        const char* pMatchPlannerParameters = "</PlannerParameters>";
        size_t nMatchPlannerParametersSize = 20;
        bool bFoundMatch = false;
        for (char c = I.get(); I; c = I.get()) {
            vstrbuf.push_back(c);
            if (c == pMatchPlannerParameters[nMatchPlannerParametersSize - 1]) {
                // matches at the end, check if previous string matches
                if (vstrbuf.size() >= nMatchPlannerParametersSize) {
					//TODO:give an OS-independent version of strnicmp.
                    if (_strnicmp(&vstrbuf[vstrbuf.size() - nMatchPlannerParametersSize], pMatchPlannerParameters, nMatchPlannerParametersSize) == 0) {
                        // matches, so quit
                        bFoundMatch = true;
                        break;
                    }
                }
            }
        }

        if (!bFoundMatch) {
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("error, failed to find </PlannerParameters> in %s"), buf.str(), ORE_InvalidArguments);
        }

        pp._plannerparametersdepth = 0;
        xmlreaders::ParseXMLData(pp, &vstrbuf[0], vstrbuf.size());
    }

    return I;
}

void SubtractStates(std::vector<dReal>& q1, const std::vector<dReal>& q2)
{
    BOOST_ASSERT(q1.size() == q2.size());
    for (size_t i = 0; i < q1.size(); ++i) {
        q1[i] -= q2[i];
    }
}

int AddStates(std::vector<dReal>& q, const std::vector<dReal>& qdelta, int fromgoal)
{
    BOOST_ASSERT(q.size() == qdelta.size());
    for (size_t i = 0; i < q.size(); ++i) {
        q[i] += qdelta[i];
    }
    return NSS_Reached;
}

int AddStatesWithLimitCheck(std::vector<dReal>& q,
    const std::vector<dReal>& qdelta, int fromgoal, const std::vector<dReal>& vLowerLimits, const std::vector<dReal>& vUpperLimits)
{
    BOOST_ASSERT(q.size() == qdelta.size());
    int status = NSS_Reached;
    for (size_t i = 0; i < q.size(); ++i) {
        q[i] += qdelta[i];
        if (q[i] > vUpperLimits.at(i)) {
            q[i] = vUpperLimits.at(i);
            status |= 0x2;
        } else if (q[i] < vLowerLimits.at(i)) {
            q[i] = vLowerLimits.at(i);
            status |= 0x2;
        }
    }
    return status;
}

PlannerStatus::PlannerStatus()
    : statusCode(0)
{
}

PlannerStatus::PlannerStatus(const std::string& description, const int statusCode, CollisionReportPtr report)
    : description(description)
    , statusCode(statusCode)
    , report(report)
{
    // not sure if this works...
    errorOrigin = str(boost::format("[%s:%d %s] ") % OpenRAVE::RaveGetSourceFilename(__FILE__) % __LINE__ % __FUNCTION__);
}

PlannerStatus::PlannerStatus(const int status_code)
    : PlannerStatus("", status_code)
{
    description.clear();
    if (status_code & PS_HasSolution) 
	{
        description += "planner succeeded. ";
    } else
	{
        description += "planner failed with generic error. ";
    }

    if (status_code & PS_Interrupted) 
	{
        description += "planning was interrupted, but can be resumed by calling PlanPath again. ";
    }
    if (status_code & PS_InterruptedWithSolution) 
	{
        description += "planning was interrupted, but a valid path/solution was returned.\
                        Can call PlanPath again to refine results. ";
    }
    if (status_code & PS_FailedDueToCollision) 
	{
        description += "planner failed due to collision constraints. ";
    }
    if (status_code & PS_FailedDueToInitial) 
	{
        description += "failed due to initial configurations. ";
    }
    if (status_code & PS_FailedDueToGoal) 
	{
        description += "failed due to goal configurations. ";
    }
    if (status_code & PS_FailedDueToKinematics)
	{
        description += "failed due to kinematics constraints. ";
    }
    if (status_code & PS_FailedDueToIK) 
	{
        description += "failed due to inverse kinematics \
                       (could be due to collisions or velocity constraints, but don't know). ";
    }
    if (status_code & PS_FailedDueToVelocityConstraints) 
	{
        description += "failed due to velocity constraints. ";
    }

    if (description.empty()) 
	{
        RAVELOG_WARN_FORMAT(_tr("planner status code (0x%x) is not supported by planner status default constructor"), status_code);
    }
}

PlannerStatus::PlannerStatus(const std::string& description,
	const int status_code, 
	const IkParameterization& ikparam, 
	CollisionReportPtr report)
    : PlannerStatus(description, status_code, report)
{
    this->ikparam = ikparam;
}

PlannerStatus::PlannerStatus(const std::string& description, 
	const int statusCode, 
	const std::vector<dReal>& jointValues, 
	CollisionReportPtr report)
    : PlannerStatus(description, statusCode, report)
{
    this->jointValues = jointValues;
}

PlannerStatus::~PlannerStatus()
{
}

PlannerStatus& PlannerStatus::SetErrorOrigin(const std::string& errorOrigin)
{
    this->errorOrigin = errorOrigin;
    return *this;
}

PlannerStatus& PlannerStatus::SetPlannerParameters(PlannerParametersConstPtr parameters)
{
    this->parameters = parameters;
    return *this;
}

void PlannerStatus::SaveToJson(rapidjson::Value& rPlannerStatus, rapidjson::Document::AllocatorType& alloc) const
{
    rPlannerStatus.SetObject();
    openravejson::SetJsonValueByKey(rPlannerStatus, "errorOrigin", errorOrigin, alloc);
    openravejson::SetJsonValueByKey(rPlannerStatus, "description", description, alloc);
    openravejson::SetJsonValueByKey(rPlannerStatus, "statusCode", statusCode, alloc);
    if (jointValues.size() > 0) {
        openravejson::SetJsonValueByKey(rPlannerStatus, "jointValues", jointValues, alloc);
    }

    if (report != NULL) {
        rapidjson::Value reportjson(rapidjson::kObjectType);
        if (!!report->plink1) {
            openravejson::SetJsonValueByKey(reportjson, "plink1", report->plink1->GetName(), alloc);
        }
        if (!!report->plink2) {
            openravejson::SetJsonValueByKey(reportjson, "plink2", report->plink2->GetName(), alloc);
        }
        rapidjson::Value reportContactsjson(rapidjson::kObjectType);
        for (size_t i = 0; i < report->contacts.size(); ++i) {
            rapidjson::Value reportContactsPosjson(rapidjson::kObjectType);
            openravejson::SetJsonValueByKey(reportContactsPosjson, "x", report->contacts[i].pos.x, alloc);
            openravejson::SetJsonValueByKey(reportContactsPosjson, "y", report->contacts[i].pos.y, alloc);
            openravejson::SetJsonValueByKey(reportContactsPosjson, "z", report->contacts[i].pos.z, alloc);

            rapidjson::Value rname;
            openravejson::SaveJsonValue(rname, std::to_string(i), alloc);
            reportContactsjson.AddMember(rname, reportContactsPosjson, alloc);
        }
        openravejson::SetJsonValueByKey(reportjson, "contacts", reportContactsjson, alloc);
        //Eventually, serialization could be in openravejson.h
        openravejson::SetJsonValueByKey(rPlannerStatus, "collisionReport", reportjson, alloc);
    }

    //Eventually, serialization could be in openravejson.h ?
    if (ikparam.GetType() != IKP_None) {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1); /// have to do this or otherwise precision gets lost
        ss << ikparam;
        openravejson::SetJsonValueByKey(rPlannerStatus, "ikparam", ss.str(), alloc);
    }
}

PlannerParameters::StateSaver::StateSaver(PlannerParametersPtr params)
    : _params(params)
{
    _params->_getstatefn(_values);
    OPENRAVE_ASSERT_OP((int)_values.size(), ==, _params->GetDOF());
}

PlannerParameters::StateSaver::~StateSaver()
{
    _Restore();
}

void PlannerParameters::StateSaver::Restore()
{
    _Restore();
}

void PlannerParameters::StateSaver::_Restore()
{
    int ret = _params->SetStateValues(_values, 0);
    BOOST_ASSERT(ret == 0);
}

PlannerParameters::PlannerParameters()
    : XMLReadable("plannerparameters")
    , step_length_(0.04f)
    , max_iterations_(0)
    , max_planning_time_(0)
    , post_processing_planner_(s_linearsmoother)
    , random_generator_seed_(0)
{
    _diffstatefn = SubtractStates;
    _neighstatefn = AddStates;

    //_sPostProcessingParameters ="<_nmaxiterations>100</_nmaxiterations><_postprocessing planner=\"lineartrajectoryretimer\"></_postprocessing>";
    // should not verify initial path since coming from RRT. actually the linear smoother sometimes introduces small collisions due to the discrete nature of the collision checking, so also want to ignore those
    _sPostProcessingParameters = "<_nmaxiterations>20</_nmaxiterations><_postprocessing planner=\"parabolicsmoother\"><_nmaxiterations>100</_nmaxiterations><verifyinitialpath>0</verifyinitialpath></_postprocessing>";
    xml_parameters_vector_.reserve(20);
    xml_parameters_vector_.push_back("configuration");
    xml_parameters_vector_.push_back("_vinitialconfig");
    xml_parameters_vector_.push_back("_vinitialconfigvelocities");
    xml_parameters_vector_.push_back("_vgoalconfigvelocities");
    xml_parameters_vector_.push_back("_vgoalconfig");
    xml_parameters_vector_.push_back("_vconfiglowerlimit");
    xml_parameters_vector_.push_back("_vconfigupperlimit");
    xml_parameters_vector_.push_back("_vconfigvelocitylimit");
    xml_parameters_vector_.push_back("_vconfigaccelerationlimit");
    xml_parameters_vector_.push_back("_vconfigresolution");
    xml_parameters_vector_.push_back("_nmaxiterations");
    xml_parameters_vector_.push_back("_nmaxplanningtime");
    xml_parameters_vector_.push_back("_fsteplength");
    xml_parameters_vector_.push_back("_postprocessing");
    xml_parameters_vector_.push_back("_nrandomgeneratorseed");
}

PlannerParameters::~PlannerParameters()
{
}

PlannerParameters::PlannerParameters(const PlannerParameters& r)
    : XMLReadable("")
{
    BOOST_ASSERT(0);
}

PlannerParameters& PlannerParameters::operator=(const PlannerParameters& r)
{
    // reset
    _costfn = r._costfn;
    _goalfn = r._goalfn;
    _distmetricfn = r._distmetricfn;
    _checkpathvelocityconstraintsfn = r._checkpathvelocityconstraintsfn;
    _samplefn = r._samplefn;
    _sampleneighfn = r._sampleneighfn;
    _samplegoalfn = r._samplegoalfn;
    _sampleinitialfn = r._sampleinitialfn;
    _setstatevaluesfn = r._setstatevaluesfn;
    _getstatefn = r._getstatefn;
    _diffstatefn = r._diffstatefn;
    _neighstatefn = r._neighstatefn;
    _listInternalSamplers = r._listInternalSamplers;

    initial_config_vector_.resize(0);
    initial_config_velocities_vector_.resize(0);
    goal_config_velocities_vector_.resize(0);
    goal_config_vector_.resize(0);
    _configurationspecification = ConfigurationSpecification();
    config_lower_limit_vector_.resize(0);
    config_upper_limit_vector_.resize(0);
    config_resolution_vector_.resize(0);
    config_velocity_limit_vector_.resize(0);
    config_acceleration_limit_vector_.resize(0);
    post_processing_planner_ = "";
    _sPostProcessingParameters.resize(0);
    _sExtraParameters.resize(0);
    max_iterations_ = 0;
    max_planning_time_ = 0;
    step_length_ = 0.04f;
    random_generator_seed_ = 0;
    _plannerparametersdepth = 0;

    // transfer data
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1); /// have to do this or otherwise precision gets lost and planners' initial conditions can vioalte constraints
    ss << r;
    ss >> *this;
    return *this;
}

void PlannerParameters::copy(std::shared_ptr<PlannerParameters const> r)
{
    *this = *r;
}

int PlannerParameters::SetStateValues(const std::vector<dReal>& values, int options) const
{
    if (!!_setstatevaluesfn) {
        return _setstatevaluesfn(values, options);
    }
    throw OpenRAVEException(_tr("need to set PlannerParameters::_setstatevaluesfn"));
}

bool PlannerParameters::serialize(std::ostream& O, int options) const
{
    O << _configurationspecification << endl;
    O << "<_vinitialconfig>";
    FOREACHC(it, initial_config_vector_)
    {
        O << *it << " ";
    }
    O << "</_vinitialconfig>" << endl;
    O << "<_vinitialconfigvelocities>";
    FOREACHC(it, initial_config_velocities_vector_)
    {
        O << *it << " ";
    }
    O << "</_vinitialconfigvelocities>" << endl;
    O << "<_vgoalconfig>";
    FOREACHC(it, goal_config_vector_)
    {
        O << *it << " ";
    }
    O << "</_vgoalconfig>" << endl;
    O << "<_vgoalconfigvelocities>";
    FOREACHC(it, goal_config_velocities_vector_)
    {
        O << *it << " ";
    }
    O << "</_vgoalconfigvelocities>" << endl;
    O << "<_vconfiglowerlimit>";
    FOREACHC(it, config_lower_limit_vector_)
    {
        O << *it << " ";
    }
    O << "</_vconfiglowerlimit>" << endl;
    O << "<_vconfigupperlimit>";
    FOREACHC(it, config_upper_limit_vector_)
    {
        O << *it << " ";
    }
    O << "</_vconfigupperlimit>" << endl;
    O << "<_vconfigvelocitylimit>";
    FOREACHC(it, config_velocity_limit_vector_)
    {
        O << *it << " ";
    }
    O << "</_vconfigvelocitylimit>" << endl;
    O << "<_vconfigaccelerationlimit>";
    FOREACHC(it, config_acceleration_limit_vector_)
    {
        O << *it << " ";
    }
    O << "</_vconfigaccelerationlimit>" << endl;
    O << "<_vconfigresolution>";
    FOREACHC(it, config_resolution_vector_)
    {
        O << *it << " ";
    }
    O << "</_vconfigresolution>" << endl;

    O << "<_nmaxiterations>" << max_iterations_ << "</_nmaxiterations>" << endl;
    O << "<_nmaxplanningtime>" << max_planning_time_ << "</_nmaxplanningtime>" << endl;
    O << "<_fsteplength>" << step_length_ << "</_fsteplength>" << endl;
    O << "<_nrandomgeneratorseed>" << random_generator_seed_ << "</_nrandomgeneratorseed>" << endl;
    O << "<_postprocessing planner=\"" << post_processing_planner_ << "\">" << _sPostProcessingParameters << "</_postprocessing>" << endl;
    if (!(options & 1)) {
        O << _sExtraParameters << endl;
    }
    return !!O;
}

BaseXMLReader::ProcessElement PlannerParameters::startElement(const std::string& name, const AttributesList& atts)
{
    _ss.str(""); // have to clear the string
    if (!!__pcurreader) {
        if (__pcurreader->startElement(name, atts) == PE_Support) {
            return PE_Support;
        }
        return PE_Ignore;
    }

    if (__processingtag.size() > 0) {
        return PE_Ignore;
    }
    if (name == "plannerparameters") {
        _plannerparametersdepth++;
        return PE_Support;
    }

    if (name == "_postprocessing") {
        _sslocal.reset(new std::stringstream());
        post_processing_planner_ = "";
        _sPostProcessingParameters = "";
        FOREACHC(itatt, atts)
        {
            if (itatt->first == "planner") {
                post_processing_planner_ = itatt->second;
            }
        }
        __pcurreader.reset(new DummyXMLReader(name, GetXMLId(), _sslocal));
        return PE_Support;
    }

    if (find(xml_parameters_vector_.begin(), xml_parameters_vector_.end(), name) == xml_parameters_vector_.end()) {
        _sslocal.reset(new std::stringstream());
        *_sslocal << "<" << name << " ";
        FOREACHC(itatt, atts)
        {
            *_sslocal << itatt->first << "=\"" << itatt->second << "\" ";
        }
        *_sslocal << ">" << endl;
        __pcurreader.reset(new DummyXMLReader(name, GetXMLId(), _sslocal));
        return PE_Support;
    }

    if (name == "configuration") {
        __pcurreader.reset(new ConfigurationSpecification::Reader(_configurationspecification));
        return PE_Support;
    }

    static const std::array<std::string, 14> names = { { "_vinitialconfig", "_vgoalconfig", "_vconfiglowerlimit", "_vconfigupperlimit", "_vconfigvelocitylimit", "_vconfigaccelerationlimit", "_vconfigresolution", "_nmaxiterations", "_nmaxplanningtime", "_fsteplength", "_postprocessing", "_nrandomgeneratorseed", "_vinitialconfigvelocities", "_vgoalconfigvelocities" } };
    if (find(names.begin(), names.end(), name) != names.end()) {
        __processingtag = name;
        return PE_Support;
    }
    return PE_Pass;
}

bool PlannerParameters::endElement(const std::string& name)
{
    if (!!__pcurreader) {
        if (__pcurreader->endElement(name)) {
            std::shared_ptr<DummyXMLReader> pdummy = std::dynamic_pointer_cast<DummyXMLReader>(__pcurreader);
            if (!!pdummy) {
                if (pdummy->GetFieldName() == "_postprocessing") {
                    _sPostProcessingParameters = _sslocal->str();
                    _sslocal.reset();
                } else {
                    *_sslocal << "</" << name << ">" << endl;
                    _sExtraParameters += _sslocal->str();
                    _sslocal.reset();
                }
            }
            __pcurreader.reset();
        }
    } else if (name == "plannerparameters") {
        return --_plannerparametersdepth < 0;
    } else if (__processingtag.size() > 0) {
        if (name == "_vinitialconfig") {
            initial_config_vector_ = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        } else if (name == "_vinitialconfigvelocities") {
            initial_config_velocities_vector_ = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        } else if (name == "_vgoalconfig") {
            goal_config_vector_ = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        } else if (name == "_vgoalconfigvelocities") {
            goal_config_velocities_vector_ = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        } else if (name == "_vconfiglowerlimit") {
            config_lower_limit_vector_ = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        } else if (name == "_vconfigupperlimit") {
            config_upper_limit_vector_ = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        } else if (name == "_vconfigresolution") {
            config_resolution_vector_ = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        } else if (name == "_vconfigvelocitylimit") {
            config_velocity_limit_vector_ = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        } else if (name == "_vconfigaccelerationlimit") {
            config_acceleration_limit_vector_ = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        } else if (name == "_nmaxiterations") {
            _ss >> max_iterations_;
        } else if (name == "_nmaxplanningtime") {
            _ss >> max_planning_time_;
        } else if (name == "_fsteplength") {
            _ss >> step_length_;
        } else if (name == "_nrandomgeneratorseed") {
            _ss >> random_generator_seed_;
        }
        if (name != __processingtag) {
            RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n") % name % __processingtag));
        }
        __processingtag = "";
        return false;
    }

    return false;
}

void PlannerParameters::characters(const std::string& ch)
{
    if (!!__pcurreader) {
        __pcurreader->characters(ch);
    } else {
        _ss.clear();
        _ss << ch;
    }
}

std::ostream& operator<<(std::ostream& O, const PlannerParameters& v)
{
    O << "<" << v.GetXMLId() << ">" << endl;
    v.serialize(O);
    O << "</" << v.GetXMLId() << ">" << endl;
    return O;
}

int SetActiveDOFValuesParameters(RobotBasePtr probot, const std::vector<dReal>& values, int options)
{
    // should setstatefn check limits?
    probot->SetActiveDOFValues(values, KinBody::CLA_CheckLimits);
    return 0;
}

int SetDOFValuesIndicesParameters(KinBodyPtr pbody, const std::vector<dReal>& values, const std::vector<int>& vindices, int options)
{
    // should setstatefn check limits?
    pbody->SetDOFValues(values, KinBody::CLA_CheckLimits, vindices);
    return 0;
}

int SetDOFVelocitiesIndicesParameters(KinBodyPtr pbody, const std::vector<dReal>& velocities, const std::vector<int>& vindices, int options)
{
    // should setstatefn check limits?
    pbody->SetDOFVelocities(velocities, KinBody::CLA_CheckLimits, vindices);
    return 0;
}

void PlannerParameters::SetRobotActiveJoints(RobotBasePtr robot)
{
    // check if any of the links affected by the dofs beside the base link are static
    FOREACHC(itlink, robot->GetLinks())
    {
        if ((*itlink)->IsStatic()) {
            FOREACHC(itdof, robot->GetActiveDOFIndices())
            {
                KinBody::JointPtr pjoint = robot->GetJointFromDOFIndex(*itdof);
                OPENRAVE_ASSERT_FORMAT(!robot->DoesAffect(pjoint->GetJointIndex(), (*itlink)->GetIndex()), "robot %s link %s is static when it is affected by active joint %s", robot->GetName() % (*itlink)->GetName() % pjoint->GetName(), ORE_InvalidState);
            }
        }
    }

    using namespace planningutils;
    _distmetricfn = boost::bind(&SimpleDistanceMetric::Eval, std::shared_ptr<SimpleDistanceMetric>(new SimpleDistanceMetric(robot)), _1, _2);
    if (robot->GetActiveDOF() == (int)robot->GetActiveDOFIndices().size()) {
        // only roobt joint indices, so use a more resiliant function
        _getstatefn = boost::bind(&RobotBase::GetDOFValues, robot, _1, robot->GetActiveDOFIndices());
        _setstatevaluesfn = boost::bind(SetDOFValuesIndicesParameters, robot, _1, robot->GetActiveDOFIndices(), _2);
        _diffstatefn = boost::bind(&RobotBase::SubtractDOFValues, robot, _1, _2, robot->GetActiveDOFIndices());
    } else {
        _getstatefn = boost::bind(&RobotBase::GetActiveDOFValues, robot, _1);
        _setstatevaluesfn = boost::bind(SetActiveDOFValuesParameters, robot, _1, _2);
        _diffstatefn = boost::bind(&RobotBase::SubtractActiveDOFValues, robot, _1, _2);
    }

    SpaceSamplerBasePtr pconfigsampler = RaveCreateSpaceSampler(robot->GetEnv(), str(boost::format("robotconfiguration %s") % robot->GetName()));
    _listInternalSamplers.clear();
    _listInternalSamplers.push_back(pconfigsampler);

    std::shared_ptr<SimpleNeighborhoodSampler> defaultsamplefn(new SimpleNeighborhoodSampler(pconfigsampler, _distmetricfn, _diffstatefn));
    _samplefn = boost::bind(&SimpleNeighborhoodSampler::Sample, defaultsamplefn, _1);
    _sampleneighfn = boost::bind(&SimpleNeighborhoodSampler::Sample, defaultsamplefn, _1,
        _2, _3);

    robot->GetActiveDOFLimits(config_lower_limit_vector_, config_upper_limit_vector_);
    robot->GetActiveDOFVelocityLimits(config_velocity_limit_vector_);
    robot->GetActiveDOFAccelerationLimits(config_acceleration_limit_vector_);
    robot->GetActiveDOFResolutions(config_resolution_vector_);
    robot->GetActiveDOFValues(initial_config_vector_);
    robot->GetActiveDOFVelocities(initial_config_velocities_vector_); // necessary?
    _configurationspecification = robot->GetActiveConfigurationSpecification();

    _neighstatefn = boost::bind(AddStatesWithLimitCheck, _1, _2, _3, boost::ref(config_lower_limit_vector_), boost::ref(config_upper_limit_vector_)); // probably ok... do we need to clamp limits?

    // have to do this last, disable timed constraints for default
    std::list<KinBodyPtr> listCheckCollisions;
    listCheckCollisions.push_back(robot);
    std::shared_ptr<DynamicsCollisionConstraint> pcollision(new DynamicsCollisionConstraint(shared_parameters(), listCheckCollisions, 0xffffffff & ~CFO_CheckTimeBasedConstraints));
    _checkpathvelocityconstraintsfn = boost::bind(&DynamicsCollisionConstraint::Check, pcollision, _1, _2, _3, _4, _5, _6, _7, _8);
}

void _CallDiffStateFns(const std::vector<std::pair<PlannerParameters::DiffStateFn, int>>& vfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v0, const std::vector<dReal>& v1)
{
    if (vfunctions.size() == 1) {
        vfunctions.at(0).first(v0, v1);
    } else {
        OPENRAVE_ASSERT_OP((int)v0.size(), ==, nDOF);
        OPENRAVE_ASSERT_OP((int)v1.size(), ==, nDOF);
        std::vector<dReal> vtemp0, vtemp1;
        vtemp0.reserve(nMaxDOFForGroup);
        vtemp1.reserve(nMaxDOFForGroup);
        std::vector<dReal>::iterator itsource0 = v0.begin();
        std::vector<dReal>::const_iterator itsource1 = v1.begin();
        FOREACHC(itfn, vfunctions)
        {
            vtemp0.resize(itfn->second);
            std::copy(itsource0, itsource0 + itfn->second, vtemp0.begin());
            vtemp1.resize(itfn->second);
            std::copy(itsource1, itsource1 + itfn->second, vtemp1.begin());
            itfn->first(vtemp0, vtemp1);
            // copy result back to itsource0
            std::copy(vtemp0.begin(), vtemp0.end(), itsource0);
            itsource0 += itfn->second;
            itsource1 += itfn->second;
        }
    }
}

/// \brief returns square root of joint distance * weights
/// \param vweights2 squared weights
dReal _EvalJointDOFDistanceMetric(const PlannerParameters::DiffStateFn& difffn, const std::vector<dReal>& c0, const std::vector<dReal>& c1, const std::vector<dReal>& vweights2)
{
    std::vector<dReal> c = c0;
    difffn(c, c1);
    dReal dist = 0;
    for (size_t i = 0; i < c.size(); i++) {
        dist += vweights2.at(i) * c.at(i) * c.at(i);
    }
    return RaveSqrt(dist);
}

dReal _CallDistMetricFns(const std::vector<std::pair<PlannerParameters::DistMetricFn, int>>& vfunctions, int nDOF, int nMaxDOFForGroup, const std::vector<dReal>& v0, const std::vector<dReal>& v1)
{
    if (vfunctions.size() == 1) {
        return vfunctions.at(0).first(v0, v1);
    } else {
        OPENRAVE_ASSERT_OP((int)v0.size(), ==, nDOF);
        OPENRAVE_ASSERT_OP((int)v1.size(), ==, nDOF);
        std::vector<dReal> vtemp0, vtemp1;
        vtemp0.reserve(nMaxDOFForGroup);
        vtemp1.reserve(nMaxDOFForGroup);
        std::vector<dReal>::const_iterator itsource0 = v0.begin(), itsource1 = v1.begin();
        dReal f = 0;
        FOREACHC(itfn, vfunctions)
        {
            vtemp0.resize(itfn->second);
            std::copy(itsource0, itsource0 + itfn->second, vtemp0.begin());
            vtemp1.resize(itfn->second);
            std::copy(itsource1, itsource1 + itfn->second, vtemp1.begin());
            f += itfn->first(vtemp0, vtemp1);
            itsource0 += itfn->second;
            itsource1 += itfn->second;
        }
        return f;
    }
}

bool _CallSampleFns(const std::vector<std::pair<PlannerParameters::SampleFn, int>>& vfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v)
{
    if (vfunctions.size() == 1) {
        return vfunctions.at(0).first(v);
    } else {
        std::vector<dReal> vtemp;
        vtemp.reserve(nMaxDOFForGroup);
        v.resize(nDOF);
        std::vector<dReal>::iterator itdest = v.begin();
        FOREACHC(itfn, vfunctions)
        {
            if (!itfn->first(vtemp)) {
                return false;
            }
            std::copy(vtemp.begin(), vtemp.end(), itdest);
            itdest += itfn->second;
        }
        return true;
    }
}

bool _CallSampleNeighFns(const std::vector<std::pair<PlannerParameters::SampleNeighFn, int>>& vfunctions, const std::vector<std::pair<PlannerParameters::DistMetricFn, int>>& vdistfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v, const std::vector<dReal>& vCurSample, dReal fRadius)
{
    if (vfunctions.size() == 1) {
        return vfunctions.at(0).first(v, vCurSample, fRadius);
    } else {
        OPENRAVE_ASSERT_OP((int)vCurSample.size(), ==, nDOF);
        std::vector<dReal> vtemp0, vtemp1;
        vtemp0.reserve(nMaxDOFForGroup);
        vtemp1.reserve(nMaxDOFForGroup);
        std::vector<dReal>::const_iterator itsample = vCurSample.begin();
        v.resize(nDOF);
        std::vector<dReal>::iterator itdest = v.begin();
        int ifn = 0;
        FOREACHC(itfn, vfunctions)
        {
            vtemp1.resize(itfn->second);
            if (fRadius <= 0) {
                std::copy(itsample, itsample + itfn->second, itdest);
            } else {
                std::copy(itsample, itsample + itfn->second, vtemp1.begin());
                if (!itfn->first(vtemp0, vtemp1, fRadius)) {
                    return false;
                }
                fRadius -= vdistfunctions[ifn].first(vtemp0, vtemp1);
                std::copy(vtemp0.begin(), vtemp0.end(), itdest);
            }
            itdest += itfn->second;
            itsample += itfn->second;
            ++ifn;
        }
        return true;
    }
}

int CallSetStateValuesFns(const std::vector<std::pair<PlannerParameters::SetStateValuesFn, int>>& vfunctions, int nDOF, int nMaxDOFForGroup, const std::vector<dReal>& v, int options)
{
    if (vfunctions.size() == 1) {
        return vfunctions.at(0).first(v, options);
    } else {
        std::vector<dReal> vtemp;
        vtemp.reserve(nMaxDOFForGroup);
        OPENRAVE_ASSERT_OP((int)v.size(), ==, nDOF);
        std::vector<dReal>::const_iterator itsrc = v.begin();
        FOREACHC(itfn, vfunctions)
        {
            vtemp.resize(itfn->second);
            std::copy(itsrc, itsrc + itfn->second, vtemp.begin());
            int ret = itfn->first(vtemp, options);
            if (ret != 0) {
                return ret;
            }
            itsrc += itfn->second;
        }
    }
    return 0;
}

void CallGetStateFns(const std::vector<std::pair<PlannerParameters::GetStateFn, int>>& vfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v)
{
    if (vfunctions.size() == 1) {
        vfunctions.at(0).first(v);
    } else {
        std::vector<dReal> vtemp;
        vtemp.reserve(nMaxDOFForGroup);
        v.resize(nDOF);
        std::vector<dReal>::iterator itdest = v.begin();
        FOREACHC(itfn, vfunctions)
        {
            itfn->first(vtemp);
            std::copy(vtemp.begin(), vtemp.end(), itdest);
            itdest += itfn->second;
        }
    }
}

int _CallNeighStateFns(const std::vector<std::pair<PlannerParameters::NeighStateFn, int>>& vfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v, const std::vector<dReal>& vdelta, int fromgoal)
{
    if (vfunctions.size() == 1) {
        return vfunctions.at(0).first(v, vdelta, fromgoal);
    } else {
        OPENRAVE_ASSERT_OP((int)vdelta.size(), ==, nDOF);
        OPENRAVE_ASSERT_OP((int)v.size(), ==, nDOF);
        std::vector<dReal> vtemp0, vtemp1;
        vtemp0.reserve(nMaxDOFForGroup);
        vtemp1.reserve(nMaxDOFForGroup);
        std::vector<dReal>::const_iterator itdelta = vdelta.begin();
        std::vector<dReal>::iterator itdest = v.begin();
        int ret = NSS_Failed;
        FOREACHC(itfn, vfunctions)
        {
            vtemp0.resize(itfn->second);
            std::copy(itdest, itdest + itfn->second, vtemp0.begin());
            vtemp1.resize(itfn->second);
            std::copy(itdelta, itdelta + itfn->second, vtemp1.begin());
            int status = itfn->first(vtemp0, vtemp1, fromgoal);
            if (status == NSS_Failed) {
                return NSS_Failed;
            }
            ret |= status;
            std::copy(vtemp0.begin(), vtemp0.end(), itdest);
            itdest += itfn->second;
            itdelta += itfn->second;
        }
        return ret;
    }
}

void PlannerParameters::SetConfigurationSpecification(EnvironmentBasePtr penv, const ConfigurationSpecification& spec)
{
    using namespace planningutils;
    spec.Validate();
    std::vector<std::pair<DiffStateFn, int>> diffstatefns(spec.groups_vector_.size());
    std::vector<std::pair<DistMetricFn, int>> distmetricfns(spec.groups_vector_.size());
    std::vector<std::pair<SampleFn, int>> samplefns(spec.groups_vector_.size());
    std::vector<std::pair<SampleNeighFn, int>> sampleneighfns(spec.groups_vector_.size());
    std::vector<std::pair<SetStateValuesFn, int>> setstatevaluesfns(spec.groups_vector_.size());
    std::vector<std::pair<GetStateFn, int>> getstatefns(spec.groups_vector_.size());
    std::vector<std::pair<NeighStateFn, int>> neighstatefns(spec.groups_vector_.size());
    std::vector<dReal> vConfigLowerLimit(spec.GetDOF()), vConfigUpperLimit(spec.GetDOF()), vConfigVelocityLimit(spec.GetDOF()), vConfigAccelerationLimit(spec.GetDOF()), vConfigResolution(spec.GetDOF()), v0, v1;
    std::list<KinBodyPtr> listCheckCollisions;
    string bodyname;
    stringstream ss, ssout;
    // order the groups depending on offset
    int nMaxDOFForGroup = 0;
    std::vector<std::pair<int, int>> vgroupoffsets(spec.groups_vector_.size());
    for (size_t igroup = 0; igroup < spec.groups_vector_.size(); ++igroup) {
        vgroupoffsets[igroup].first = spec.groups_vector_[igroup].offset;
        vgroupoffsets[igroup].second = igroup;
        nMaxDOFForGroup = max(nMaxDOFForGroup, spec.groups_vector_[igroup].dof);
    }

    _listInternalSamplers.clear();

    std::sort(vgroupoffsets.begin(), vgroupoffsets.end());
    for (size_t igroup = 0; igroup < spec.groups_vector_.size(); ++igroup) {
        const ConfigurationSpecification::Group& g = spec.groups_vector_[igroup];
        int isavegroup = vgroupoffsets[igroup].second;
        if (g.name.size() >= 12 && g.name.substr(0, 12) == "joint_values") {
            ss.clear();
            ss.str(g.name.substr(12));
            ss >> bodyname;
            BOOST_ASSERT(!!ss);
            KinBodyPtr pbody = penv->GetKinBody(bodyname);
            OPENRAVE_ASSERT_FORMAT(!!pbody, "body %s not found", bodyname, ORE_InvalidArguments);
            std::vector<int> dofindices((istream_iterator<int>(ss)), istream_iterator<int>());
            if (dofindices.size() == 0) {
                OPENRAVE_ASSERT_OP((int)dofindices.size(), ==, pbody->GetDOF());
            }
            std::vector<dReal> vweights2;
            pbody->GetDOFWeights(vweights2, dofindices);
            FOREACH(itf, vweights2)
            {
                *itf *= *itf;
            }
            diffstatefns[isavegroup].first = boost::bind(&KinBody::SubtractDOFValues, pbody, _1, _2, dofindices);
            diffstatefns[isavegroup].second = g.dof;
            distmetricfns[isavegroup].first = boost::bind(_EvalJointDOFDistanceMetric, diffstatefns[isavegroup].first, _1, _2, vweights2);
            distmetricfns[isavegroup].second = g.dof;

            SpaceSamplerBasePtr pconfigsampler = RaveCreateSpaceSampler(penv, str(boost::format("bodyconfiguration %s") % pbody->GetName()));
            _listInternalSamplers.push_back(pconfigsampler);
            {
                ss.clear();
                ss.str("");
                ss << "SetDOFs ";
                FOREACHC(itindex, dofindices)
                {
                    ss << *itindex << " ";
                }
                if (!pconfigsampler->SendCommand(ssout, ss)) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_tr("failed to set body %s configuration to %s"), pbody->GetName() % ss.str(), ORE_Assert);
                }
            }
            std::shared_ptr<SimpleNeighborhoodSampler> defaultsamplefn(new SimpleNeighborhoodSampler(pconfigsampler, distmetricfns[isavegroup].first, diffstatefns[isavegroup].first));
            samplefns[isavegroup].first = boost::bind(&SimpleNeighborhoodSampler::Sample, defaultsamplefn, _1);
            samplefns[isavegroup].second = g.dof;
            sampleneighfns[isavegroup].first = boost::bind(&SimpleNeighborhoodSampler::Sample, defaultsamplefn, _1, _2, _3);
            sampleneighfns[isavegroup].second = g.dof;
            setstatevaluesfns[isavegroup].first = boost::bind(SetDOFValuesIndicesParameters, pbody, _1, dofindices, _2);
            setstatevaluesfns[isavegroup].second = g.dof;
            getstatefns[isavegroup].first = boost::bind(&KinBody::GetDOFValues, pbody, _1, dofindices);
            getstatefns[isavegroup].second = g.dof;
            neighstatefns[isavegroup].second = g.dof;
            pbody->GetDOFLimits(v0, v1, dofindices);
            neighstatefns[isavegroup].first = boost::bind(AddStatesWithLimitCheck, _1, _2, _3, v0, v1);
            std::copy(v0.begin(), v0.end(), vConfigLowerLimit.begin() + g.offset);
            std::copy(v1.begin(), v1.end(), vConfigUpperLimit.begin() + g.offset);
            pbody->GetDOFVelocityLimits(v0, dofindices);
            std::copy(v0.begin(), v0.end(), vConfigVelocityLimit.begin() + g.offset);
            pbody->GetDOFAccelerationLimits(v0, dofindices);
            std::copy(v0.begin(), v0.end(), vConfigAccelerationLimit.begin() + g.offset);
            pbody->GetDOFResolutions(v0, dofindices);
            std::copy(v0.begin(), v0.end(), vConfigResolution.begin() + g.offset);
            if (find(listCheckCollisions.begin(), listCheckCollisions.end(), pbody) == listCheckCollisions.end()) {
                listCheckCollisions.push_back(pbody);
            }
        }
        //        else if( g.name.size() >= 16 && g.name.substr(0,16) == "joint_velocities" ) {
        //        }
        //        else if( g.name.size() >= 16 && g.name.substr(0,16) == "affine_transform" ) {
        //        }
        //        else if( g.name.size() >= 17 && g.name.substr(0,17) == "affine_velocities" ) {
        //        }
        //        else if( g.name.size() >= 4 && g.name.substr(0,4) == "grab" ) {
        //        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("group %s not supported for for planner parameters configuration"), g.name, ORE_InvalidArguments);
        }
    }
    _diffstatefn = boost::bind(_CallDiffStateFns, diffstatefns, spec.GetDOF(), nMaxDOFForGroup, _1, _2);
    _distmetricfn = boost::bind(_CallDistMetricFns, distmetricfns, spec.GetDOF(), nMaxDOFForGroup, _1, _2);
    _samplefn = boost::bind(_CallSampleFns, samplefns, spec.GetDOF(), nMaxDOFForGroup, _1);
    _sampleneighfn = boost::bind(_CallSampleNeighFns, sampleneighfns, distmetricfns, spec.GetDOF(), nMaxDOFForGroup, _1, _2, _3);
    _setstatevaluesfn = boost::bind(CallSetStateValuesFns, setstatevaluesfns, spec.GetDOF(), nMaxDOFForGroup, _1, _2);
    _getstatefn = boost::bind(CallGetStateFns, getstatefns, spec.GetDOF(), nMaxDOFForGroup, _1);
    _neighstatefn = boost::bind(_CallNeighStateFns, neighstatefns, spec.GetDOF(), nMaxDOFForGroup, _1, _2, _3);
    config_lower_limit_vector_.swap(vConfigLowerLimit);
    config_upper_limit_vector_.swap(vConfigUpperLimit);
    config_velocity_limit_vector_.swap(vConfigVelocityLimit);
    config_acceleration_limit_vector_.swap(vConfigAccelerationLimit);
    config_resolution_vector_.swap(vConfigResolution);
    _configurationspecification = spec;
    _getstatefn(initial_config_vector_);
    // have to do this last, disable timed constraints for default
    std::shared_ptr<DynamicsCollisionConstraint> pcollision(new DynamicsCollisionConstraint(shared_parameters(), listCheckCollisions, 0xffffffff & ~CFO_CheckTimeBasedConstraints));
    _checkpathvelocityconstraintsfn = boost::bind(&DynamicsCollisionConstraint::Check, pcollision, _1,
        _2, _3, _4,
        _5, _6, _7, _8);
}

void PlannerParameters::Validate() const
{
    OPENRAVE_ASSERT_OP(_configurationspecification.GetDOF(), ==, GetDOF());
    OPENRAVE_ASSERT_OP(initial_config_vector_.size() % GetDOF(), ==, 0);
    OPENRAVE_ASSERT_OP(initial_config_velocities_vector_.size() % GetDOF(), ==, 0);
    OPENRAVE_ASSERT_OP(goal_config_velocities_vector_.size() % GetDOF(), ==, 0);
    OPENRAVE_ASSERT_OP(goal_config_vector_.size() % GetDOF(), ==, 0);
    OPENRAVE_ASSERT_OP(config_lower_limit_vector_.size(), ==, (size_t)GetDOF());
    OPENRAVE_ASSERT_OP(config_upper_limit_vector_.size(), ==, (size_t)GetDOF());
    if (config_velocity_limit_vector_.size() > 0) {
        OPENRAVE_ASSERT_OP(config_velocity_limit_vector_.size(), ==, (size_t)GetDOF());
    }
    if (config_acceleration_limit_vector_.size() > 0) {
        OPENRAVE_ASSERT_OP(config_acceleration_limit_vector_.size(), ==, (size_t)GetDOF());
    }
    OPENRAVE_ASSERT_OP(config_resolution_vector_.size(), ==, (size_t)GetDOF());
    OPENRAVE_ASSERT_OP(step_length_, >=, 0); // == 0 is valid for auto-steps
    OPENRAVE_ASSERT_OP(max_iterations_, >=, 0); // == 0 is valid for auto-iterations

    // check all stateless functions, which means ie anything but configuration samplers
    vector<dReal> vstate;
    if (!!_getstatefn) {
        _getstatefn(vstate);
        OPENRAVE_ASSERT_OP(vstate.size(), ==, (size_t)GetDOF());
    }
    if (!!_setstatevaluesfn) {
        // need to save/restore state before calling this function?
        //_setstatefn();
    }
    if (!!_costfn) {
        _costfn(vstate);
    }
    if (!!_goalfn) {
        _goalfn(vstate);
    }
    if (!!_distmetricfn) {
        dReal dist = _distmetricfn(vstate, vstate);
        OPENRAVE_ASSERT_OP(dist, <=, 10 * g_fEpsilon);
    }
    if (!!_checkpathvelocityconstraintsfn) {
        _checkpathvelocityconstraintsfn(vstate, vstate, std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenStart, 0, ConstraintFilterReturnPtr());
    }
    if (!!_neighstatefn && vstate.size() > 0) {
        vector<dReal> vstate2 = vstate;
        vector<dReal> vzeros(vstate.size());
        int neighstatus = _neighstatefn(vstate2, vzeros, NSO_OnlyHardConstraints);
        OPENRAVE_ASSERT_OP(neighstatus, &, NSS_Reached); // LSB indicates if _neighstatefn call is successful
        dReal dist = _distmetricfn(vstate, vstate2);
        if (IS_DEBUGLEVEL(Level_Debug)) {
            if (dist > 1000 * g_fEpsilon) {
                std::stringstream ss;
                ss << "vstate=";
                for (size_t i = 0; i < vstate.size(); ++i) {
                    ss << vstate[i] << ", ";
                }
                ss << "; vstate2=";
                for (size_t i = 0; i < vstate2.size(); ++i) {
                    ss << vstate2[i] << ", ";
                }
                RAVELOG_DEBUG_FORMAT("unequal states: %s", ss.str());
            }
        }

        OPENRAVE_ASSERT_OP(dist, <=, 1000 * g_fEpsilon);
    }
    if (!!_diffstatefn && vstate.size() > 0) {
        vector<dReal> vstate2 = vstate;
        _diffstatefn(vstate2, vstate);
        OPENRAVE_ASSERT_OP(vstate2.size(), ==, (size_t)GetDOF());
    }
}

PlannerBase::PlannerProgress::PlannerProgress()
    : _iteration(0)
{
}

class CustomPlannerCallbackData : public std::enable_shared_from_this<CustomPlannerCallbackData>, public UserData {
public:
    CustomPlannerCallbackData(const PlannerBase::PlanCallbackFn& callbackfn, PlannerBasePtr planner)
        : _callbackfn(callbackfn)
        , _plannerweak(planner)
    {
    }
    virtual ~CustomPlannerCallbackData()
    {
        PlannerBasePtr planner = _plannerweak.lock();
        if (!!planner) {
            planner->__listRegisteredCallbacks.erase(_iterator);
        }
    }

    PlannerBase::PlanCallbackFn _callbackfn;
    PlannerBaseWeakPtr _plannerweak;
    std::list<UserDataWeakPtr>::iterator _iterator;
};

typedef std::shared_ptr<CustomPlannerCallbackData> CustomPlannerCallbackDataPtr;

PlannerBase::PlannerBase(EnvironmentBasePtr penv)
    : InterfaceBase(PT_Planner, penv)
{
}

bool PlannerBase::InitPlan(RobotBasePtr pbase, std::istream& isParameters)
{
    RAVELOG_WARN(str(boost::format("using default planner parameters structure to de-serialize parameters data inside %s, information might be lost!! Please define a InitPlan(robot,stream) function!\n") % GetXMLId()));
    std::shared_ptr<PlannerParameters> localparams(new PlannerParameters());
    isParameters >> *localparams;
    localparams->Validate();
    return InitPlan(pbase, localparams);
}

UserDataPtr PlannerBase::RegisterPlanCallback(const PlanCallbackFn& callbackfn)
{
    CustomPlannerCallbackDataPtr pdata(new CustomPlannerCallbackData(callbackfn, shared_planner()));
    pdata->_iterator = __listRegisteredCallbacks.insert(__listRegisteredCallbacks.end(), pdata);
    return pdata;
}

PlannerStatus PlannerBase::_ProcessPostPlanners(RobotBasePtr probot, TrajectoryBasePtr ptraj)
{
    if (GetParameters()->post_processing_planner_.size() == 0) {
        __cachePostProcessPlanner.reset();
        return PlannerStatus(PS_HasSolution);
    }
    if (!__cachePostProcessPlanner || __cachePostProcessPlanner->GetXMLId() != GetParameters()->post_processing_planner_) {
        __cachePostProcessPlanner = RaveCreatePlanner(GetEnv(), GetParameters()->post_processing_planner_);
        if (!__cachePostProcessPlanner) {
            __cachePostProcessPlanner = RaveCreatePlanner(GetEnv(), s_linearsmoother);
            if (!__cachePostProcessPlanner) {
                return PlannerStatus(PS_Failed);
            }
        }
    }

    // transfer the callbacks?
    list<UserDataPtr> listhandles;
    FOREACHC(it, __listRegisteredCallbacks)
    {
        CustomPlannerCallbackDataPtr pitdata = std::dynamic_pointer_cast<CustomPlannerCallbackData>(it->lock());
        if (!!pitdata) {
            listhandles.push_back(__cachePostProcessPlanner->RegisterPlanCallback(pitdata->_callbackfn));
        }
    }

    PlannerParametersPtr params(new PlannerParameters());
    params->copy(GetParameters());
    params->_sExtraParameters += GetParameters()->_sPostProcessingParameters;
    params->post_processing_planner_ = "";
    params->_sPostProcessingParameters = "";
    params->max_iterations_ = 0; // have to reset since path optimizers also use it and new parameters could be in extra parameters
    //params->max_planning_time_ = 0; // have to reset since path optimizers also use it and new parameters could be in extra parameters??
    if (__cachePostProcessPlanner->InitPlan(probot, params)) {
        return __cachePostProcessPlanner->PlanPath(ptraj);
    }

    // do not fall back to a default linear smoother like in the past! that makes behavior unpredictable
    return PlannerStatus(PS_Failed);
}

PlannerAction PlannerBase::_CallCallbacks(const PlannerProgress& progress)
{
    FOREACHC(it, __listRegisteredCallbacks)
    {
        CustomPlannerCallbackDataPtr pitdata = std::dynamic_pointer_cast<CustomPlannerCallbackData>(it->lock());
        if (!!pitdata) {
            PlannerAction ret = pitdata->_callbackfn(progress);
            if (ret != PA_None) {
                return ret;
            }
        }
    }
    return PA_None;
}
}
