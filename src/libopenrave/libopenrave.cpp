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
#include "libopenrave.h"

#include <boost/scoped_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/thread/once.hpp>

#include <streambuf>

#ifndef _WIN32
#include <sys/stat.h>
#include <sys/types.h>
#include <libintl.h>
#endif

#include <locale>
#include <set>

#include "plugindatabase.h"

#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>

#define LIBXML_SAX1_ENABLED
#include <libxml/globals.h>
#include <libxml/xmlerror.h>
#include <libxml/parser.h>
#include <libxml/parserInternals.h> // only for xmlNewInputFromFile()
#include <libxml/tree.h>

#include <libxml/debugXML.h>
#include <libxml/xmlmemory.h>
#include <openrave/rave_global.h>
#include <openrave/multi_controller_base.h>
#include <openrave/dummy_xml_reader.h>

#if OPENRAVE_LOG4CXX

#include <log4cxx/layout.h>
#include <log4cxx/patternlayout.h>
#include <log4cxx/consoleappender.h>

namespace log4cxx {

class ColorLayout : public Layout {
public:
    DECLARE_LOG4CXX_OBJECT(ColorLayout)
    BEGIN_LOG4CXX_CAST_MAP()
    LOG4CXX_CAST_ENTRY(ColorLayout)
    LOG4CXX_CAST_ENTRY_CHAIN(Layout)
    END_LOG4CXX_CAST_MAP()

    ColorLayout();
    ColorLayout(const LayoutPtr& layout);
    virtual ~ColorLayout();

    virtual void activateOptions(helpers::Pool& p) {
        _layout->activateOptions(p);
    }
    virtual void setOption(const LogString& option, const LogString& value) {
        _layout->setOption(option, value);
    }
    virtual bool ignoresThrowable() const {
        return _layout->ignoresThrowable();
    }

    virtual void format(LogString& output, const spi::LoggingEventPtr& event, helpers::Pool& pool) const;

protected:
    virtual LogString _Colorize(const spi::LoggingEventPtr& event) const;

    LayoutPtr _layout;
};

}

#endif

namespace OpenRAVE 
{




static std::set<std::string> _gettextDomainsInitialized;




#if OPENRAVE_LOG4CXX
log4cxx::LevelPtr RaveGetVerboseLogLevel()
{
    static log4cxx::LevelPtr level(new log4cxx::Level(log4cxx::Level::TRACE_INT, LOG4CXX_STR("VERBOSE"), 7));
    return level;
}

log4cxx::LoggerPtr RaveGetLogger()
{
    return RaveGlobal::instance()->GetLogger();
}
#endif



void RaveSetDebugLevel(int level)
{
    RaveGlobal::instance()->SetDebugLevel(level);
}

int RaveGetDebugLevel()
{
    return RaveGlobal::instance()->GetDebugLevel();
}

const std::map<InterfaceType,std::string>& RaveGetInterfaceNamesMap()
{
    return RaveGlobal::instance()->GetInterfaceNamesMap();
}

const std::string& RaveGetInterfaceName(InterfaceType type)
{
    return RaveGlobal::instance()->GetInterfaceName(type);
}

std::string RaveGetHomeDirectory()
{
    return RaveGlobal::instance()->GetHomeDirectory();
}

std::string RaveFindDatabaseFile(const std::string& filename, bool bRead)
{
    return RaveGlobal::instance()->FindDatabaseFile(filename,bRead);
}

int RaveInitialize(bool bLoadAllPlugins, int level)
{
    return RaveGlobal::instance()->Initialize(bLoadAllPlugins,level);
}

void RaveInitializeFromState(UserDataPtr globalstate)
{
    RaveGlobal::_state = std::dynamic_pointer_cast<RaveGlobal>(globalstate);
}

UserDataPtr RaveGlobalState()
{
    // only return valid pointer if initialized!
    std::shared_ptr<RaveGlobal> state = RaveGlobal::_state;
    if( !!state && state->_IsInitialized() ) {
        return state;
    }
    return UserDataPtr();
}

void RaveDestroy()
{
    RaveGlobal::instance()->Destroy();
}

void RaveAddCallbackForDestroy(const std::function<void()>& fn)
{
    RaveGlobal::instance()->AddCallbackForDestroy(fn);
}

int RaveGetEnvironmentId(EnvironmentBaseConstPtr penv)
{
    return RaveGlobal::instance()->GetEnvironmentId(penv);
}

EnvironmentBasePtr RaveGetEnvironment(int id)
{
    return RaveGlobal::instance()->GetEnvironment(id);
}

void RaveGetEnvironments(std::list<EnvironmentBasePtr>& listenvironments)
{
    RaveGlobal::instance()->GetEnvironments(listenvironments);
}

void RaveGetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& plugins)
{
    RaveGlobal::instance()->GetDatabase()->GetPluginInfo(plugins);
}

void RaveGetLoadedInterfaces(std::map<InterfaceType, std::vector<std::string> >& interfacenames)
{
    RaveGlobal::instance()->GetDatabase()->GetLoadedInterfaces(interfacenames);
}

void RaveReloadPlugins()
{
    RaveGlobal::instance()->GetDatabase()->ReloadPlugins();
}

bool RaveLoadPlugin(const std::string& libraryname)
{
    return RaveGlobal::instance()->GetDatabase()->LoadPlugin(libraryname);
}

bool RaveHasInterface(InterfaceType type, const std::string& interfacename)
{
    return RaveGlobal::instance()->GetDatabase()->HasInterface(type,interfacename);
}

InterfaceBasePtr RaveCreateInterface(EnvironmentBasePtr penv, InterfaceType type,const std::string& interfacename)
{
    return RaveGlobal::instance()->GetDatabase()->Create(penv, type,interfacename);
}

RobotBasePtr RaveCreateRobot(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateRobot(penv,name);
}

PlannerBasePtr RaveCreatePlanner(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreatePlanner(penv, name);
}

SensorSystemBasePtr RaveCreateSensorSystem(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateSensorSystem(penv, name);
}

ControllerBasePtr RaveCreateController(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateController(penv, name);
}

MultiControllerBasePtr RaveCreateMultiController(EnvironmentBasePtr env, const std::string& rawname)
{
    std::string name;
    if( rawname == "" ) {
        name = "genericmulticontroller";
    }
    else {
        name = rawname;
        std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    }
    // TODO remove hack once MultiController is a registered interface
    ControllerBasePtr pcontroller = RaveGlobal::instance()->GetDatabase()->CreateController(env, name);
    if( name == "genericmulticontroller" ) {
        return std::static_pointer_cast<MultiControllerBase>(pcontroller);
    }
    // don't support anything else
    return MultiControllerBasePtr();
}

ModuleBasePtr RaveCreateModule(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateModule(penv, name);
}

ModuleBasePtr RaveCreateProblem(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateModule(penv, name);
}

ModuleBasePtr RaveCreateProblemInstance(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateModule(penv, name);
}

IkSolverBasePtr RaveCreateIkSolver(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateIkSolver(penv, name);
}

PhysicsEngineBasePtr RaveCreatePhysicsEngine(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreatePhysicsEngine(penv, name);
}

SensorBasePtr RaveCreateSensor(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateSensor(penv, name);
}

CollisionCheckerBasePtr RaveCreateCollisionChecker(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateCollisionChecker(penv, name);
}

ViewerBasePtr RaveCreateViewer(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateViewer(penv, name);
}

KinBodyPtr RaveCreateKinBody(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateKinBody(penv, name);
}

TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateTrajectory(penv, name);
}

TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr penv, int dof)
{
    return RaveCreateTrajectory(penv,"");
}

SpaceSamplerBasePtr RaveCreateSpaceSampler(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateSpaceSampler(penv, name);
}

UserDataPtr RaveRegisterInterface(InterfaceType type, const std::string& name, const char* interfacehash, const char* envhash, const std::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)>& createfn)
{
    return RaveGlobal::instance()->GetDatabase()->RegisterInterface(type, name, interfacehash,envhash,createfn);
}

UserDataPtr RaveRegisterXMLReader(InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& fn)
{
    return RaveGlobal::instance()->RegisterXMLReader(type,xmltag,fn);
}

BaseXMLReaderPtr RaveCallXMLReader(InterfaceType type, const std::string& xmltag, InterfaceBasePtr pinterface, const AttributesList& atts)
{
    return RaveGlobal::instance()->CallXMLReader(type,xmltag,pinterface,atts);
}

std::string RaveFindLocalFile(const std::string& filename, const std::string& curdir)
{
    return RaveGlobal::instance()->FindLocalFile(filename,curdir);
}

bool RaveInvertFileLookup(std::string& newfilename, const std::string& filename)
{
    return RaveGlobal::instance()->InvertFileLookup(newfilename,filename);
}

void RaveSetDataAccess(int options)
{
    RaveGlobal::instance()->SetDataAccess(options);
}

int RaveGetDataAccess()
{
    return RaveGlobal::instance()->GetDataAccess();
}

std::string RaveGetDefaultViewerType()
{
    return RaveGlobal::instance()->GetDefaultViewerType();
}

const char *RaveGetLocalizedTextForDomain(const std::string& domainname, const char *msgid)
{
#ifndef _WIN32
    if (_gettextDomainsInitialized.find(domainname) == _gettextDomainsInitialized.end())
    {
        bindtextdomain(domainname.c_str(), OPENRAVE_LOCALE_INSTALL_DIR);
        _gettextDomainsInitialized.insert(domainname);
    }
    return dgettext(domainname.c_str(), msgid);
#else
    return msgid;
#endif
}




int RaveGetIndexFromAffineDOF(int affinedofs, DOFAffine _dof)
{
    int dof = static_cast<int>(_dof);
    dof &= affinedofs;
    int index = 0;
    if( dof&DOF_X ) {
        return index;
    }
    else if( affinedofs & DOF_X ) {
        ++index;
    }
    if( dof&DOF_Y ) {
        return index;
    }
    else if( affinedofs & DOF_Y ) {
        ++index;
    }
    if( dof&DOF_Z ) {
        return index;
    }
    else if( affinedofs & DOF_Z ) {
        ++index;
    }
    if( dof&DOF_RotationAxis ) {
        return index;
    }
    if( dof&DOF_Rotation3D ) {
        return index;
    }
    if( dof&DOF_RotationQuat ) {
        return index;
    }
    throw OPENRAVE_EXCEPTION_FORMAT(_("unspecified dof 0x%x, 0x%x"),affinedofs%dof,ORE_InvalidArguments);
}

DOFAffine RaveGetAffineDOFFromIndex(int affinedofs, int requestedindex)
{
    BOOST_ASSERT(requestedindex >= 0);
    int index = 0;
    if( index == requestedindex && (affinedofs&DOF_X) ) {
        return DOF_X;
    }
    else if( affinedofs & DOF_X ) {
        ++index;
    }
    if( index == requestedindex && (affinedofs&DOF_Y) ) {
        return DOF_Y;
    }
    else if( affinedofs & DOF_Y ) {
        ++index;
    }
    if( index == requestedindex  && (affinedofs&DOF_Z)) {
        return DOF_Z;
    }
    else if( affinedofs & DOF_Z ) {
        ++index;
    }
    if( index <= requestedindex && index+3 > requestedindex && (affinedofs&DOF_RotationAxis) ) {
        return DOF_RotationAxis;
    }
    if( index <= requestedindex && index+3 > requestedindex && (affinedofs&DOF_Rotation3D) ) {
        return DOF_Rotation3D;
    }
    if( index <= requestedindex && index+4 > requestedindex && (affinedofs&DOF_RotationQuat) ) {
        return DOF_RotationQuat;
    }
    throw OPENRAVE_EXCEPTION_FORMAT(_("requested index out of bounds %d (affinemask=0x%x)"),requestedindex%affinedofs, ORE_InvalidArguments);
}

int RaveGetAffineDOF(int affinedofs)
{
    if( affinedofs & DOF_RotationAxis ) {
        BOOST_ASSERT( !(affinedofs & (DOF_Rotation3D|DOF_RotationQuat)) );
    }
    else if( affinedofs & DOF_Rotation3D ) {
        BOOST_ASSERT( !(affinedofs & (DOF_RotationAxis|DOF_RotationQuat)) );
    }
    else if( affinedofs & DOF_RotationQuat ) {
        BOOST_ASSERT( !(affinedofs & (DOF_Rotation3D|DOF_RotationAxis)) );
    }
    int dof = 0;
    if( affinedofs & DOF_X ) {
        dof++;
    }
    if( affinedofs & DOF_Y ) {
        dof++;
    }
    if( affinedofs & DOF_Z ) {
        dof++;
    }
    if( affinedofs & DOF_RotationAxis ) {
        dof++;
    }
    else if( affinedofs & DOF_Rotation3D ) {
        dof += 3;
    }
    else if( affinedofs & DOF_RotationQuat ) {
        dof += 4;
    }
    return dof;
}

void RaveGetAffineDOFValuesFromTransform(std::vector<dReal>::iterator itvalues, const Transform& t, int affinedofs, const Vector& vActvAffineRotationAxis)
{
    if( affinedofs & DOF_X ) {
        *itvalues++ = t.trans.x;
    }
    if( affinedofs & DOF_Y ) {
        *itvalues++ = t.trans.y;
    }
    if( affinedofs & DOF_Z ) {
        *itvalues++ = t.trans.z;
    }
    if( affinedofs & DOF_RotationAxis ) {
        *itvalues++ = -normalizeAxisRotation(vActvAffineRotationAxis, t.rot).first;
    }
    else if( affinedofs & DOF_Rotation3D ) {
        dReal fsin = RaveSqrt(t.rot.y * t.rot.y + t.rot.z * t.rot.z + t.rot.w * t.rot.w);
        dReal fangle = 2 * atan2(fsin, t.rot.x);
        if( fsin > 0 ) {
            dReal normalizer = fangle / fsin;
            *itvalues++ = normalizer * t.rot.y;
            *itvalues++ = normalizer * t.rot.z;
            *itvalues++ = normalizer * t.rot.w;
        }
        else {
            *itvalues++ = 0;
            *itvalues++ = 0;
            *itvalues++ = 0;
        }
    }
    else if( affinedofs & DOF_RotationQuat ) {
        *itvalues++ = t.rot.x;
        *itvalues++ = t.rot.y;
        *itvalues++ = t.rot.z;
        *itvalues++ = t.rot.w;
    }
}

void RaveGetAffineDOFValuesFromVelocity(std::vector<dReal>::iterator itvalues, const Vector& linearvel, const Vector& angularvel, const Vector& quatrotation, int affinedofs, const Vector& axis)
{
    if( affinedofs & DOF_X ) {
        *itvalues++ = linearvel.x;
    }
    if( affinedofs & DOF_Y ) {
        *itvalues++ = linearvel.y;
    }
    if( affinedofs & DOF_Z ) {
        *itvalues++ = linearvel.z;
    }
    if( affinedofs & DOF_RotationAxis ) {
        *itvalues++ = axis.dot3(angularvel);
    }
    else if( affinedofs & DOF_Rotation3D ) {
        *itvalues++ = angularvel.x;
        *itvalues++ = angularvel.y;
        *itvalues++ = angularvel.z;
    }
    else if( affinedofs & DOF_RotationQuat ) {
        Vector qdot = 0.5 * quatMultiply(Vector(0,angularvel.x,angularvel.y,angularvel.z), quatrotation);
        *itvalues++ = qdot.x;
        *itvalues++ = qdot.y;
        *itvalues++ = qdot.z;
        *itvalues++ = qdot.w;
    }
}

void RaveGetTransformFromAffineDOFValues(Transform& t, std::vector<dReal>::const_iterator itvalues, int affinedofs, const Vector& vActvAffineRotationAxis, bool normalize)
{
    if( affinedofs & DOF_X ) {
        t.trans.x = *itvalues++;
    }
    if( affinedofs & DOF_Y ) {
        t.trans.y = *itvalues++;
    }
    if( affinedofs & DOF_Z ) {
        t.trans.z = *itvalues++;
    }
    if( affinedofs & DOF_RotationAxis ) {
        dReal angle = *itvalues++;
        dReal fsin = RaveSin(angle*(dReal)0.5);
        t.rot.x = RaveCos(angle*(dReal)0.5);
        t.rot.y = vActvAffineRotationAxis.x * fsin;
        t.rot.z = vActvAffineRotationAxis.y * fsin;
        t.rot.w = vActvAffineRotationAxis.z * fsin;
    }
    else if( affinedofs & DOF_Rotation3D ) {
        if( normalize ) {
            dReal x = *itvalues++;
            dReal y = *itvalues++;
            dReal z = *itvalues++;
            dReal fang = RaveSqrt(x*x + y*y + z*z);
            if( fang > 0 ) {
                dReal fnormalizer = sin((dReal)0.5 * fang) / fang;
                t.rot.x = cos((dReal)0.5 * fang);
                t.rot.y = fnormalizer * x;
                t.rot.z = fnormalizer * y;
                t.rot.w = fnormalizer * z;
            }
            else {
                t.rot = Vector(1,0,0,0); // identity
            }
        }
        else {
            // normalization turned off, but affine dofs have only DOF_Rotation3D. In order to convert this to quaternion velocity
            // will need current quaternion value (qrot), which is unknown, so assume identity
            // qvel = [0,axisangle] * qrot * 0.5
            t.rot[0] = 0;
            t.rot[1] = 0.5 * *itvalues++;
            t.rot[2] = 0.5 * *itvalues++;
            t.rot[3] = 0.5 * *itvalues++;
        }
    }
    else if( affinedofs & DOF_RotationQuat ) {
        // have to normalize since user might not be aware of this particular parameterization of rotations
        t.rot.x = *itvalues++;
        t.rot.y = *itvalues++;
        t.rot.z = *itvalues++;
        t.rot.w = *itvalues++;
        if( normalize ) {
            t.rot.normalize4();
        }
    }
}

void RaveGetVelocityFromAffineDOFVelocities(Vector& linearvel, Vector& angularvel, std::vector<dReal>::const_iterator itvalues, int affinedofs, const Vector& axis, const Vector& quatrotation)
{
    if( affinedofs & DOF_X ) {
        linearvel.x = *itvalues++;
    }
    if( affinedofs & DOF_Y ) {
        linearvel.y = *itvalues++;
    }
    if( affinedofs & DOF_Z ) {
        linearvel.z = *itvalues++;
    }
    if( affinedofs & DOF_RotationAxis ) {
        dReal angle = *itvalues++;
        angularvel = axis*angle;
    }
    else if( affinedofs & DOF_Rotation3D ) {
        angularvel.x = *itvalues++;
        angularvel.y = *itvalues++;
        angularvel.z = *itvalues++;
    }
    else if( affinedofs & DOF_RotationQuat ) {
        // have to normalize since user might not be aware of this particular parameterization of rotations
        Vector q;
        q.x = *itvalues++;
        q.y = *itvalues++;
        q.z = *itvalues++;
        q.w = *itvalues++;
        Vector angularvel2 = quatMultiply(q, quatInverse(quatrotation)); // qdot = 0.5 * angularvel * q
        angularvel.x = angularvel2.y * 2;
        angularvel.y = angularvel2.z * 2;
        angularvel.z = angularvel2.w * 2;
    }
}





void RaveInitRandomGeneration(uint32_t seed)
{
    RaveGlobal::instance()->GetDefaultSampler()->SetSeed(seed);
}

uint32_t RaveRandomInt()
{
    std::vector<uint32_t> sample;
    RaveGlobal::instance()->GetDefaultSampler()->SampleSequence(sample);
    return sample.at(0);
}

float RaveRandomFloat(IntervalType interval)
{
    std::vector<dReal> sample;
    RaveGlobal::instance()->GetDefaultSampler()->SampleSequence(sample,1,interval);
    return float(sample.at(0));
}

double RaveRandomDouble(IntervalType interval)
{
    std::vector<dReal> sample;
    RaveGlobal::instance()->GetDefaultSampler()->SampleSequence(sample,1,interval);
    return double(sample.at(0));
}

namespace LocalXML {

void RaveXMLErrorFunc(void *ctx, const char *msg, ...)
{
    va_list args;

    va_start(args, msg);
    RAVELOG_ERROR("XML Parse error: ");
    vprintf(msg,args);
    va_end(args);
}

struct XMLREADERDATA
{
    XMLREADERDATA(BaseXMLReader& reader, xmlParserCtxtPtr ctxt) : _reader(reader), _ctxt(ctxt) {
    }
    BaseXMLReader& _reader;
    BaseXMLReaderPtr _pdummy;
    xmlParserCtxtPtr _ctxt;
};

void DefaultStartElementSAXFunc(void *ctx, const xmlChar *name, const xmlChar **atts)
{
    AttributesList listatts;
    if( atts != NULL ) {
        for (int i = 0; (atts[i] != NULL); i+=2) {
            listatts.emplace_back((const char*)atts[i], (const char*)atts[i+1]);
            std::transform(listatts.back().first.begin(), listatts.back().first.end(), listatts.back().first.begin(), ::tolower);
        }
    }

    XMLREADERDATA* pdata = (XMLREADERDATA*)ctx;
    string s = (const char*)name;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if( !!pdata->_pdummy ) {
        RAVELOG_VERBOSE(str(boost::format("unknown field %s\n")%s));
        pdata->_pdummy->startElement(s,listatts);
    }
    else {
        BaseXMLReader::ProcessElement pestatus = pdata->_reader.startElement(s, listatts);
        if( pestatus != BaseXMLReader::PE_Support ) {
            // not handling, so create a temporary class to handle it
            pdata->_pdummy.reset(new DummyXMLReader(s,"(libxml)"));
        }
    }
}

void DefaultEndElementSAXFunc(void *ctx, const xmlChar *name)
{
    XMLREADERDATA* pdata = (XMLREADERDATA*)ctx;
    string s = (const char*)name;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if( !!pdata->_pdummy ) {
        if( pdata->_pdummy->endElement(s) ) {
            pdata->_pdummy.reset();
        }
    }
    else {
        if( pdata->_reader.endElement(s) ) {
            //RAVEPRINT(L"%s size read %d\n", name, data->_ctxt->input->consumed);
            xmlStopParser(pdata->_ctxt);
        }
    }
}

void DefaultCharactersSAXFunc(void *ctx, const xmlChar *ch, int len)
{
    XMLREADERDATA* pdata = (XMLREADERDATA*)ctx;
    if( !!pdata->_pdummy ) {
        pdata->_pdummy->characters(string((const char*)ch, len));
    }
    else {
        pdata->_reader.characters(string((const char*)ch, len));
    }
}

bool xmlDetectSAX2(xmlParserCtxtPtr ctxt)
{
    if (ctxt == NULL) {
        return false;
    }
#ifdef LIBXML_SAX1_ENABLED
    if (ctxt->sax &&  ctxt->sax->initialized == XML_SAX2_MAGIC && (ctxt->sax->startElementNs != NULL || ctxt->sax->endElementNs != NULL)) {
        ctxt->sax2 = 1;
    }
#else
    ctxt->sax2 = 1;
#endif

    ctxt->str_xml = xmlDictLookup(ctxt->dict, BAD_CAST "xml", 3);
    ctxt->str_xmlns = xmlDictLookup(ctxt->dict, BAD_CAST "xmlns", 5);
    ctxt->str_xml_ns = xmlDictLookup(ctxt->dict, XML_XML_NAMESPACE, 36);
    if ( ctxt->str_xml==NULL || ctxt->str_xmlns==NULL || ctxt->str_xml_ns == NULL) {
        return false;
    }
    return true;
}

bool ParseXMLData(BaseXMLReader& reader, const char* buffer, int size)
{
    static xmlSAXHandler s_DefaultSAXHandler = { 0};
    if( size <= 0 ) {
        size = strlen(buffer);
    }
    if( !s_DefaultSAXHandler.initialized ) {
        // first time, so init
        s_DefaultSAXHandler.startElement = DefaultStartElementSAXFunc;
        s_DefaultSAXHandler.endElement = DefaultEndElementSAXFunc;
        s_DefaultSAXHandler.characters = DefaultCharactersSAXFunc;
        s_DefaultSAXHandler.error = RaveXMLErrorFunc;
        s_DefaultSAXHandler.initialized = 1;
    }

    xmlSAXHandlerPtr sax = &s_DefaultSAXHandler;
    int ret = 0;
    xmlParserCtxtPtr ctxt;

    ctxt = xmlCreateMemoryParserCtxt(buffer, size);
    if (ctxt == NULL) {
        return false;
    }
    if (ctxt->sax != (xmlSAXHandlerPtr) &xmlDefaultSAXHandler) {
        xmlFree(ctxt->sax);
    }
    ctxt->sax = sax;
    xmlDetectSAX2(ctxt);

    XMLREADERDATA readerdata(reader, ctxt);
    ctxt->userData = &readerdata;

    xmlParseDocument(ctxt);

    if (ctxt->wellFormed) {
        ret = 0;
    }
    else {
        if (ctxt->errNo != 0) {
            ret = ctxt->errNo;
        }
        else {
            ret = -1;
        }
    }
    if (sax != NULL) {
        ctxt->sax = NULL;
    }
    if (ctxt->myDoc != NULL) {
        xmlFreeDoc(ctxt->myDoc);
        ctxt->myDoc = NULL;
    }
    xmlFreeParserCtxt(ctxt);

    return ret==0;
}

}

} // end namespace OpenRAVE

#if OPENRAVE_LOG4CXX

using namespace log4cxx;

IMPLEMENT_LOG4CXX_OBJECT(ColorLayout);

ColorLayout::ColorLayout() : Layout()
{
}

ColorLayout::ColorLayout(const LayoutPtr& layout) : Layout(), _layout(layout)
{
}

ColorLayout::~ColorLayout()
{
}

void ColorLayout::format(LogString& output, const spi::LoggingEventPtr& event, helpers::Pool& pool) const
{
    _layout->format(output, event, pool);

    // add color
    output.reserve(output.size() + 32);
    output.insert(0, _Colorize(event));
    output.append(LOG4CXX_STR("\x1b[0m"));
}

LogString ColorLayout::_Colorize(const spi::LoggingEventPtr& event) const
{
    int bg = -1;
    int fg = -1;
    bool bold = false;
    LogString csi;

    csi.reserve(32);

    if (event->getLevel()->isGreaterOrEqual(Level::getFatal())) {
        fg = OPENRAVECOLOR_FATALLEVEL;
    } else if (event->getLevel()->isGreaterOrEqual(Level::getError())) {
        fg = OPENRAVECOLOR_ERRORLEVEL;
    } else if (event->getLevel()->isGreaterOrEqual(Level::getWarn())) {
        fg = OPENRAVECOLOR_WARNLEVEL;
    } else if (event->getLevel()->isGreaterOrEqual(Level::getInfo())) {
    } else if (event->getLevel()->isGreaterOrEqual(Level::getDebug())) {
        fg = OPENRAVECOLOR_DEBUGLEVEL;
    } else {
        fg = OPENRAVECOLOR_VERBOSELEVEL;
    }

    csi += LOG4CXX_STR("\x1b[0");

    if (bg >= 0) {
        csi += LOG4CXX_STR(';');
        csi += LOG4CXX_STR('4');
        csi += LOG4CXX_STR('0') + bg;
    }

    if (fg >= 0) {
        csi += LOG4CXX_STR(';');
        csi += LOG4CXX_STR('3');
        csi += LOG4CXX_STR('0') + fg;
    }

    if (bold) {
        csi += LOG4CXX_STR(';');
        csi += LOG4CXX_STR('1');
    }

    csi += LOG4CXX_STR('m');

    return csi;
}

#endif
