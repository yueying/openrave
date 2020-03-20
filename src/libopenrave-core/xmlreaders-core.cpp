// -*- coding: utf-8 -*-
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
#include "ravep.h"

#include <openrave/xml_readers.h>

#include <libxml/xmlstring.h>
#define LIBXML_SAX1_ENABLED
#include <libxml/globals.h>
#include <libxml/xmlerror.h>
#include <libxml/parser.h>
#include <libxml/parserInternals.h> // only for xmlNewInputFromFile()
#include <libxml/tree.h>

#include <libxml/debugXML.h>
#include <libxml/xmlmemory.h>

#include <iostream>
#include <sstream>

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem.hpp>
#endif

#include <boost/utility.hpp>
#include <boost/thread/once.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

BOOST_STATIC_ASSERT(sizeof(xmlChar) == 1);

#if defined(OPENRAVE_IS_ASSIMP3)
#include <assimp/scene.h>
#include <assimp/LogStream.hpp>
#include <assimp/DefaultLogger.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#elif defined(OPENRAVE_ASSIMP)
#include <assimp.hpp>
#include <aiScene.h>
#include <aiPostProcess.h>
#include <DefaultLogger.h>
#endif

#ifdef OPENRAVE_IVCON
#include <ivcon.h>
#endif

#include <openrave/tri_mesh.h>

namespace OpenRAVEXMLParser
{

static boost::once_flag __onceCreateXMLMutex = BOOST_ONCE_INIT;
static boost::once_flag __onceSetAssimpLog = BOOST_ONCE_INIT;

/// lock for parsing XML, don't make it a static variable in order to ensure it remains valid for as long as possible
static EnvironmentMutex* __mutexXML;
void __CreateXMLMutex()
{
    __mutexXML = new EnvironmentMutex();
}

#ifdef OPENRAVE_ASSIMP

class myStream : public Assimp::LogStream
{
public:
    myStream() {
    }

    ~myStream() {
    }

    void write(const char* message) {
        RAVELOG_VERBOSE("%s",message);
    }
};

void __SetAssimpLog()
{
    using namespace Assimp;
    Assimp::DefaultLogger::create("",Assimp::Logger::VERBOSE);
    // Select the kinds of messages you want to receive on this log stream
#ifdef OPENRAVE_ASSIMP_PRE_R896
    const unsigned int severity = Logger::DEBUGGING|Logger::INFO|Logger::WARN|Logger::ERR;
#else
    const unsigned int severity = Logger::Debugging|Logger::Info|Logger::Warn|Logger::Err;
#endif

    // Attaching it to the default logger
    DefaultLogger::get()->attachStream( new myStream(), severity );
}
#endif

EnvironmentMutex* GetXMLMutex()
{
    boost::call_once(__CreateXMLMutex,__onceCreateXMLMutex);
    return __mutexXML;
}

/// the directory of the file currently parsing
std::string& GetParseDirectory() {
    static string s; return s;
}

class SetParseDirectoryScope
{
public:
    SetParseDirectoryScope(const std::string& newdir) {
        _olddir = GetParseDirectory();
        GetParseDirectory() = newdir;
    }
    ~SetParseDirectoryScope() {
        GetParseDirectory() = _olddir;
    }

private:
    std::string _olddir;
};

int& GetXMLErrorCount()
{
    static int errorcount=0;
    return errorcount;
}

#ifdef OPENRAVE_ASSIMP
class aiSceneManaged
{
public:
    aiSceneManaged(const std::string& dataorfilename, bool bIsFilename=true, const std::string& formathint=std::string(), unsigned int flags = aiProcess_JoinIdenticalVertices|aiProcess_Triangulate|aiProcess_FindDegenerates|aiProcess_PreTransformVertices|aiProcess_SortByPType) {
        boost::call_once(__SetAssimpLog,__onceSetAssimpLog);
        _importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT|aiPrimitiveType_LINE);
        if( bIsFilename ) {
            _scene = _importer.ReadFile(dataorfilename.c_str(),flags);
        }
        else {
            _scene = _importer.ReadFileFromMemory(dataorfilename.c_str(),dataorfilename.size(), flags, formathint.c_str());
        }
        if( _scene == NULL ) {
            RAVELOG_VERBOSE("assimp error: %s\n",_importer.GetErrorString());
        }
    }
    virtual ~aiSceneManaged() {
        _importer.FreeScene();
    }
    Assimp::Importer _importer;
    const struct aiScene* _scene;
};

static bool _AssimpCreateTriMesh(const aiScene* scene, aiNode* node, const Vector& scale, TriMesh& trimesh, RaveVector<float>& diffuseColor, RaveVector<float>& ambientColor, float& ftransparency)
{
    if( !node ) {
        return false;
    }
    aiMatrix4x4 transform = node->mTransformation;
    aiNode *pnode = node->mParent;
    while (pnode) {
        // Don't convert to y-up orientation, which is what the root node is in, Assimp does
        if (pnode->mParent != NULL) {
            transform = pnode->mTransformation * transform;
        }
        pnode = pnode->mParent;
    }

    std::vector<Vector>& vertices = trimesh.vertices;
    std::vector<int>& indices = trimesh.indices;
    {
        size_t vertexOffset = vertices.size();
        size_t nTotalVertices=0;
        for (size_t i = 0; i < node->mNumMeshes; i++) {
            aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
            nTotalVertices += input_mesh->mNumVertices;
        }

        vertices.reserve(vertices.size()+nTotalVertices);
        for (size_t i = 0; i < node->mNumMeshes; i++) {
            aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
            for (size_t j = 0; j < input_mesh->mNumVertices; j++) {
                aiVector3D p = input_mesh->mVertices[j];
                p *= transform;
                vertices.push_back(Vector(p.x*scale.x,p.y*scale.y,p.z*scale.z));
            }
        }
        for (size_t i = 0; i < node->mNumMeshes; i++) {
            aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
            size_t indexCount = 0;
            for (size_t j = 0; j < input_mesh->mNumFaces; j++) {
                aiFace& face = input_mesh->mFaces[j];
                indexCount += 3*(face.mNumIndices-2);
            }
            indices.reserve(indices.size()+indexCount);
            for (size_t j = 0; j < input_mesh->mNumFaces; j++) {
                aiFace& face = input_mesh->mFaces[j];
                if( face.mNumIndices == 3 ) {
                    indices.push_back(vertexOffset+face.mIndices[0]);
                    indices.push_back(vertexOffset+face.mIndices[1]);
                    indices.push_back(vertexOffset+face.mIndices[2]);
                }
                else {
                    for (size_t k = 2; k < face.mNumIndices; ++k) {
                        indices.push_back(face.mIndices[0]+vertexOffset);
                        indices.push_back(face.mIndices[k-1]+vertexOffset);
                        indices.push_back(face.mIndices[k]+vertexOffset);
                    }
                }
            }
            vertexOffset += input_mesh->mNumVertices;
        }
    }
    for (size_t i=0; i < node->mNumChildren; ++i) {
        _AssimpCreateTriMesh(scene, node->mChildren[i], scale, trimesh, diffuseColor, ambientColor, ftransparency);
    }
    return true;
}

static bool _AssimpCreateGeometries(const aiScene* scene, aiNode* node, const Vector& scale, std::list<KinBody::GeometryInfo>& geometries_list)
{
    if( !node ) {
        return false;
    }
    aiMatrix4x4 transform = node->mTransformation;
    aiNode *pnode = node->mParent;
    while (pnode) {
        // Don't convert to y-up orientation, which is what the root node is in, Assimp does
        if (pnode->mParent != NULL) {
            transform = pnode->mTransformation * transform;
        }
        pnode = pnode->mParent;
    }

    for (size_t i = 0; i < node->mNumMeshes; i++) {
        geometries_list.push_back(KinBody::GeometryInfo());
        KinBody::GeometryInfo& g = geometries_list.back();
        g.type_ = GT_TriMesh;
        g.render_scale_vec_ = scale;
        aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
        g.mesh_collision_.vertices.resize(input_mesh->mNumVertices);
        for (size_t j = 0; j < input_mesh->mNumVertices; j++) {
            aiVector3D p = input_mesh->mVertices[j];
            p *= transform;
            g.mesh_collision_.vertices[j] = Vector(p.x*scale.x,p.y*scale.y,p.z*scale.z);
        }
        size_t indexCount = 0;
        for (size_t j = 0; j < input_mesh->mNumFaces; j++) {
            aiFace& face = input_mesh->mFaces[j];
            indexCount += 3*(face.mNumIndices-2);
        }
        g.mesh_collision_.indices.reserve(indexCount);
        for (size_t j = 0; j < input_mesh->mNumFaces; j++) {
            aiFace& face = input_mesh->mFaces[j];
            if( face.mNumIndices == 3 ) {
                g.mesh_collision_.indices.push_back(face.mIndices[0]);
                g.mesh_collision_.indices.push_back(face.mIndices[1]);
                g.mesh_collision_.indices.push_back(face.mIndices[2]);
            }
            else {
                for (size_t k = 2; k < face.mNumIndices; ++k) {
                    g.mesh_collision_.indices.push_back(face.mIndices[0]);
                    g.mesh_collision_.indices.push_back(face.mIndices[k-1]);
                    g.mesh_collision_.indices.push_back(face.mIndices[k]);
                }
            }
        }

        if( !!scene->mMaterials&& input_mesh->mMaterialIndex<scene->mNumMaterials) {
            aiMaterial* mtrl = scene->mMaterials[input_mesh->mMaterialIndex];
            aiColor4D color;
            aiGetMaterialColor(mtrl,AI_MATKEY_COLOR_DIFFUSE,&color);
            g.diffuse_color_vec_ = Vector(color.r,color.g,color.b,color.a);
            aiGetMaterialColor(mtrl,AI_MATKEY_COLOR_AMBIENT,&color);
            g.ambient_color_vec_ = Vector(color.r,color.g,color.b,color.a);
        }
    }

    for (size_t i=0; i < node->mNumChildren; ++i) {
        _AssimpCreateGeometries(scene, node->mChildren[i], scale, geometries_list);
    }
    return true;
}

static bool _ParseSpecialSTLFile(EnvironmentBasePtr penv, const std::string& filename, const Vector& vscale, std::list<KinBody::GeometryInfo>& geometries_list)
{
    // some formats (screen) has # in the beginning until the STL solid definition.
    ifstream f(filename.c_str());
    string strline;
    if( !!f ) {
        bool bTransformOffset = false;
        Transform toffset;
        Vector vcolor(0.8,0.8,0.8);
        bool bFound = false;
        stringstream::streampos pos = f.tellg();
        while( !!getline(f, strline) ) {
            boost::trim(strline);
            if( strline.size() > 0 && strline[0] == '#' ) {
                size_t indCURRENT_LOCATION = strline.find("<CURRENT_LOCATION>");
                if( indCURRENT_LOCATION != string::npos ) {
                    stringstream ss(strline.substr(indCURRENT_LOCATION+18));
                    dReal rotx, roty, rotz;
                    ss >> toffset.trans.x >> toffset.trans.y >> toffset.trans.z >> rotx >> roty >> rotz;
                    if( !!ss ) {
                        bTransformOffset = true;
                        toffset.trans *= vscale;
                        toffset.rot = quatMultiply(quatFromAxisAngle(Vector(0,0,rotz*PI/180)), quatMultiply(quatFromAxisAngle(Vector(rotx*PI/180,0,0)), quatFromAxisAngle(Vector(0,roty*PI/180,0))));
                    }
                    else {
                        RAVELOG_WARN("<CURRENT_LOCATION> bad format\n");
                        toffset = Transform();
                    }
                }
                size_t indCOLOR_INFORMATION = strline.find("<COLOR_INFORMATION>");
                if( indCOLOR_INFORMATION != string::npos ) {
                    stringstream ss(strline.substr(indCOLOR_INFORMATION+19));
                    ss >> vcolor.x >> vcolor.y >> vcolor.z;
                    if( !!ss ) {
                    }
                    else {
                        RAVELOG_WARN("<COLOR_INFORMATION> bad format\n");
                    }
                }
                continue;
            }
            if( strline.size() >= 5 && strline.substr(0,5) == string("solid") ) {
                bFound = true;
                break;
            }
            pos = f.tellg();
        }
        if( bFound ) {
            RAVELOG_INFO_FORMAT("STL file %s has screen metadata", filename);

            f.seekg(0, std::ios::end);
            stringstream::streampos endpos = f.tellg();                
            f.seekg(pos);
            string newdata;
            newdata.reserve(endpos - pos);
            newdata.assign((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
            std::shared_ptr<aiSceneManaged> scene(new aiSceneManaged(newdata, false));
            if( (!scene->_scene || !scene->_scene->mRootNode) && newdata.size() >= 5 && newdata.substr(0,5) == std::string("solid") ) {
                // most likely a binary STL file with the first 5 words being solid. unfortunately assimp does not handle this well, so
                newdata[0] = 'x';
                scene.reset(new aiSceneManaged(newdata, false, "STL"));
            }

            if( !!scene->_scene && !!scene->_scene->mRootNode && !!scene->_scene->HasMeshes() ) {
                if( _AssimpCreateGeometries(scene->_scene,scene->_scene->mRootNode, vscale, geometries_list) ) {
                    FOREACH(itgeom, geometries_list) {
                        itgeom->diffuse_color_vec_ = vcolor;
                        if( bTransformOffset ) {
                            itgeom->mesh_collision_.ApplyTransform(toffset.inverse());
                        }
                    }
                    return true;
                }
            }
        }
    }
    return false;
}

#endif

bool CreateTriMeshFromFile(EnvironmentBasePtr penv, const std::string& filename, const Vector& vscale, TriMesh& trimesh, RaveVector<float>& diffuseColor, RaveVector<float>& ambientColor, float& ftransparency)
{
    string extension;
    if( filename.find_last_of('.') != string::npos ) {
        extension = filename.substr(filename.find_last_of('.')+1);
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
    }

#ifdef OPENRAVE_ASSIMP
    // assimp doesn't support vrml/iv, so don't waste time
    if( extension != "iv" && extension != "wrl" && extension != "vrml" ) {
        {
            aiSceneManaged scene(filename);
            if( !!scene._scene && !!scene._scene->mRootNode && !!scene._scene->HasMeshes() ) {
                if( _AssimpCreateTriMesh(scene._scene,scene._scene->mRootNode, vscale, trimesh, diffuseColor, ambientColor, ftransparency) ) {
                    return true;
                }
            }
        }
        if( extension == "stl" ) {
            // some formats (screen) has # in the beginning until the STL solid definition.
            ifstream f(filename.c_str());
            string strline;
            if( !!f ) {
                bool bFound = false;
                stringstream::streampos pos = f.tellg();
                while( !!getline(f, strline) ) {
                    boost::trim(strline);
                    if( strline.size() > 0 && strline[0] == '#' ) {
                        continue;
                    }
                    if( strline.size() >= 5 && strline.substr(0,5) == string("solid") ) {
                        bFound = true;
                        break;
                    }
                    pos = f.tellg();
                }
                if( bFound ) {
                    stringbuf buf;
                    f.seekg(pos);
                    f.get(buf, 0);

                    string newdata = buf.str();
                    aiSceneManaged scene(newdata, false, extension);
                    if( !!scene._scene && !!scene._scene->mRootNode && !!scene._scene->HasMeshes() ) {
                        if( _AssimpCreateTriMesh(scene._scene,scene._scene->mRootNode, vscale, trimesh, diffuseColor, ambientColor, ftransparency) ) {
                            return true;
                        }
                    }
                }

#ifdef OPENRAVE_ASSIMP
                std::list<KinBody::GeometryInfo> geometries_list;
                if( _ParseSpecialSTLFile(penv, filename, vscale, geometries_list) ) {
                    trimesh.vertices.clear();
                    trimesh.indices.clear();
                    FOREACH(itgeom, geometries_list) {
                        trimesh.Append(itgeom->mesh_collision_, itgeom->transform_);
                    }
                    return true;
                }
#endif
            }
        }
        if( extension == "stl" || extension == "x") {
            return false;
        }
    }
#endif

    ModuleBasePtr ivmodelloader = RaveCreateModule(penv,"ivmodelloader");
    if( !!ivmodelloader ) {
        stringstream sout, sin;
        sin << "LoadModel " << filename;
        sout << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        if( ivmodelloader->SendCommand(sout,sin) ) {
            sout >> trimesh >> diffuseColor >> ambientColor >> ftransparency;
            if( !!sout ) {
                FOREACH(it,trimesh.vertices) {
                    it->x *= vscale.x;
                    it->y *= vscale.y;
                    it->z *= vscale.z;
                }
                return true;
            }
        }
    }

#ifdef OPENRAVE_IVCON
    RAVELOG_DEBUG("using ivcon for geometry reading\n");
    vector<float> vertices;
    if( ivcon::ReadFile(filename.c_str(), vertices, trimesh.indices) ) {
        trimesh.vertices.resize(vertices.size()/3);
        for(size_t i = 0; i < vertices.size(); i += 3) {
            trimesh.vertices[i/3] = Vector(vscale.x*vertices[i],vscale.y*vertices[i+1],vscale.z*vertices[i+2]);
        }
        return true;
    }
#endif
    return false;
}

bool CreateTriMeshFromData(const std::string& data, const std::string& formathint, const Vector& vscale, TriMesh& trimesh, RaveVector<float>& diffuseColor, RaveVector<float>& ambientColor, float& ftransparency)
{
#ifdef OPENRAVE_ASSIMP
    aiSceneManaged scene(data, false, formathint);
    if( !!scene._scene && !!scene._scene->mRootNode && !!scene._scene->HasMeshes() ) {
        if( _AssimpCreateTriMesh(scene._scene,scene._scene->mRootNode, vscale, trimesh, diffuseColor, ambientColor, ftransparency) ) {
            return true;
        }
    }
#endif

    return false;
}

struct XMLREADERDATA
{
    XMLREADERDATA(BaseXMLReaderPtr preader, xmlParserCtxtPtr ctxt) : _preader(preader), _ctxt(ctxt) {
    }
    BaseXMLReaderPtr _preader, _pdummy;
    xmlParserCtxtPtr _ctxt;
};

static void DefaultStartElementSAXFunc(void *ctx, const xmlChar *name, const xmlChar **atts)
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
        if( ((XMLREADERDATA*)ctx)->_preader->startElement(s, listatts) != BaseXMLReader::PE_Support ) {
            // not handling, so create a temporary class to handle it
            pdata->_pdummy.reset(new DummyXMLReader(s,"(libxml)"));
        }
    }
}

static void DefaultEndElementSAXFunc(void *ctx, const xmlChar *name)
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
        if( pdata->_preader->endElement(s) ) {
            //RAVEPRINT(L"%s size read %d\n", name, data->_ctxt->input->consumed);
            xmlStopParser(pdata->_ctxt);
        }
    }
}

static void DefaultCharactersSAXFunc(void *ctx, const xmlChar *ch, int len)
{
    XMLREADERDATA* pdata = (XMLREADERDATA*)ctx;
    if( !!pdata->_pdummy ) {
        pdata->_pdummy->characters(string((const char*)ch, len));
    }
    else {
        pdata->_preader->characters(string((const char*)ch, len));
    }
}

static void RaveXMLErrorFunc(void *ctx, const char *msg, ...)
{
    va_list args;
    va_start(args, msg);
    RAVELOG_ERROR("XML Parse error: ");
    vprintf(msg,args);
    va_end(args);
    GetXMLErrorCount()++;
}

static xmlSAXHandler* GetSAXHandler()
{
    static xmlSAXHandler s_DefaultSAXHandler = { 0};
    if( !s_DefaultSAXHandler.initialized ) {
        // first time, so init
        s_DefaultSAXHandler.startElement = DefaultStartElementSAXFunc;
        s_DefaultSAXHandler.endElement = DefaultEndElementSAXFunc;
        s_DefaultSAXHandler.characters = DefaultCharactersSAXFunc;
        s_DefaultSAXHandler.error = RaveXMLErrorFunc;
        s_DefaultSAXHandler.initialized = 1;
    }
    return &s_DefaultSAXHandler;
}

static bool xmlDetectSAX2(xmlParserCtxtPtr ctxt)
{
    if (ctxt == NULL)
        return false;
#ifdef LIBXML_SAX1_ENABLED
    if ((ctxt->sax) &&  (ctxt->sax->initialized == XML_SAX2_MAGIC) && ((ctxt->sax->startElementNs != NULL) || (ctxt->sax->endElementNs != NULL))) {
        ctxt->sax2 = 1;
    }
#else
    ctxt->sax2 = 1;
#endif // LIBXML_SAX1_ENABLED

    ctxt->str_xml = xmlDictLookup(ctxt->dict, BAD_CAST "xml", 3);
    ctxt->str_xmlns = xmlDictLookup(ctxt->dict, BAD_CAST "xmlns", 5);
    ctxt->str_xml_ns = xmlDictLookup(ctxt->dict, XML_XML_NAMESPACE, 36);
    if ((ctxt->str_xml==NULL) || (ctxt->str_xmlns==NULL) || (ctxt->str_xml_ns == NULL)) {
        return false;
    }
    return true;
}

static int raveXmlSAXUserParseFile(xmlSAXHandlerPtr sax, BaseXMLReaderPtr preader, const std::string& filename)
{
    int ret = 0;
    xmlParserCtxtPtr ctxt;
    ctxt = xmlCreateFileParserCtxt(filename.c_str());
    if (ctxt == NULL) {
        return -1;
    }
    if (ctxt->sax != (xmlSAXHandlerPtr) &xmlDefaultSAXHandler) {
        xmlFree(ctxt->sax);
    }
    ctxt->sax = sax;
    xmlDetectSAX2(ctxt);

    XMLREADERDATA reader(preader, ctxt);
    ctxt->userData = &reader;

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
    return ret;
}

static int raveXmlSAXUserParseMemory(xmlSAXHandlerPtr sax, BaseXMLReaderPtr preader, const char *buffer, int size)
{
    int ret = 0;
    xmlParserCtxtPtr ctxt;

    ctxt = xmlCreateMemoryParserCtxt(buffer, size);
    if (ctxt == NULL) {
        return -1;
    }
    if (ctxt->sax != (xmlSAXHandlerPtr) &xmlDefaultSAXHandler) {
        xmlFree(ctxt->sax);
    }
    ctxt->sax = sax;
    xmlDetectSAX2(ctxt);

    XMLREADERDATA reader(preader, ctxt);
    ctxt->userData = &reader;
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
    return ret;
}

bool ParseXMLFile(BaseXMLReaderPtr preader, const std::string& filename)
{
	std::string filedata = RaveFindLocalFile(filename,GetParseDirectory());
    if( filedata.size() == 0 )
	{
        return false;
    }
    EnvironmentMutex::scoped_lock lock(*GetXMLMutex());

#ifdef HAVE_BOOST_FILESYSTEM
    SetParseDirectoryScope scope(boost::filesystem::path(filedata).parent_path().string());
#endif
    preader->file_name_ = filedata;

    int ret=-1;
    try 
	{
        ret = raveXmlSAXUserParseFile(GetSAXHandler(), preader, filedata.c_str());
        if( ret != 0 ) 
		{
            RAVELOG_WARN(str(boost::format("xmlSAXUserParseFile: error parsing %s (error %d)\n")%filedata%ret));
        }
    }
    catch(const std::exception& ex)
	{
        RAVELOG_ERROR(str(boost::format("xmlSAXUserParseFile: error parsing %s: %s\n")%filedata%ex.what()));
        ret = -1;
    }
//    catch (...) {
//        RAVELOG_ERROR(str(boost::format("xmlSAXUserParseFile: error parsing %s\n")%filedata));
//        ret = -1;
//    }

    // hmm....... necessary?
    //xmlCleanupParser();
    //xmlMemoryDump();

    return ret == 0;
}

bool ParseXMLData(BaseXMLReaderPtr preader, const std::string& pdata)
{
    if( pdata.size() == 0 ) {
        return false;
    }
    EnvironmentMutex::scoped_lock lock(*GetXMLMutex());
    return raveXmlSAXUserParseMemory(GetSAXHandler(), preader, pdata.c_str(), pdata.size())==0;
}

class KinBodyXMLReader;
typedef std::shared_ptr<KinBodyXMLReader> KinBodyXMLReaderPtr;
typedef std::shared_ptr<KinBodyXMLReader const> KinBodyXMLReaderConstPtr;

class StreamXMLReader : public BaseXMLReader
{
public:
    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList& atts)
    {
        string_stream_.str("");         // have to clear the string
        if( !!cur_reader_ ) 
		{
            if( cur_reader_->startElement(xmlname,atts) == PE_Support )
                return PE_Support;
            return PE_Ignore;
        }
        return PE_Pass;
    }

    virtual bool endElement(const std::string& xmlname)
    {
        if( !!cur_reader_ )
		{
            if( cur_reader_->endElement(xmlname) )
                cur_reader_.reset();
        }
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( !!cur_reader_ ) 
		{
            cur_reader_->characters(ch);
        }
        else 
		{
            string_stream_.clear();
            string_stream_ << ch;
        }
    }
protected:
    std::stringstream string_stream_;
    std::shared_ptr<BaseXMLReader> cur_reader_;
};

class LinkXMLReader : public StreamXMLReader
{
public:

    static bool CreateGeometries(EnvironmentBasePtr penv, 
		const std::string& filename, const Vector& vscale, 
		std::list<KinBody::GeometryInfo>& geometries_list)
    {
        std::string extension;
        if( filename.find_last_of('.') != std::string::npos ) 
		{
            extension = filename.substr(filename.find_last_of('.')+1);
            std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
        }

#ifdef OPENRAVE_ASSIMP
        // assimp doesn't support vrml/iv, so don't waste time
        if( extension != "iv" && extension != "wrl" && extension != "vrml" ) 
		{
            //Assimp::DefaultLogger::get()->setLogSeverity(Assimp::Logger::Debugging);
            {
                aiSceneManaged scene(filename);
                if( !!scene._scene && !!scene._scene->mRootNode && !!scene._scene->HasMeshes() ) 
				{
                    if( _AssimpCreateGeometries(scene._scene,scene._scene->mRootNode, vscale, geometries_list) ) 
					{
                        return true;
                    }
                }
            }
            if( extension == "stl" || extension == "x") 
			{
                if( extension == "stl" ) 
				{
                    if( _ParseSpecialSTLFile(penv, filename, vscale, geometries_list) ) 
					{
                        return true;
                    }
                    RAVELOG_WARN_FORMAT("failed to load STL file %s. If it is in binary format, \
                     make sure the first 5 characters of the file are not 'solid'!", filename);
                }
                return false;
            }
        }
#endif

        // for other importers, just convert into one big trimesh
        geometries_list.push_back(KinBody::GeometryInfo());
        KinBody::GeometryInfo& g = geometries_list.back();
        g.type_ = GT_TriMesh;
        g.diffuse_color_vec_=Vector(1,0.5f,0.5f,1);
        g.ambient_color_vec_=Vector(0.1,0.0f,0.0f,0);
        g.render_scale_vec_ = vscale;
        if( !CreateTriMeshFromFile(penv,filename,vscale,
			g.mesh_collision_,
			g.diffuse_color_vec_,
			g.ambient_color_vec_,
			g.transparency_) ) 
		{
            return false;
        }
        return true;
    }

    LinkXMLReader(KinBody::LinkPtr& plink, KinBodyPtr pparent, const AttributesList &atts)
		: link_(plink) 
	{
        parent_ = pparent;
        mass_type_ = MT_None;
        mass_density_ = 1;
        mass_extents_ = Vector(1,1,1);
        total_mass_ = 1;
        mass_custom_ = MASS::GetSphericalMass(1,Vector(0,0,0),1);
        is_skip_geometry_ = false;
        scale_geometry_ = Vector(1,1,1);

        bool is_static_set = false;
        bool is_static = false;
        bool is_enabled = true;
        std::string linkname, linkfilename;

        for(auto itatt:atts) 
		{
            if( itatt.first == "name" ) 
			{
                linkname = itatt.second;
                for(auto itcurlink:pparent->GetLinks()) 
				{
                    if( itcurlink->GetName() == linkname ) 
					{
                        link_ = itcurlink;
                        break;
                    }
                }
            }
            else if( itatt.first == "type" )
			{
                is_static_set = true;
                if( _stricmp(itatt.second.c_str(), "static") == 0 )
				{
                    is_static = true;
                }
            }
            else if( itatt.first == "file" )
			{
                linkfilename = itatt.second;
            }
            else if( itatt.first == "skipgeometry" ) 
			{
                is_skip_geometry_ = _stricmp(itatt.second.c_str(), "true") == 0 || itatt.second=="1";
            }
            else if( itatt.first == "scalegeometry" ) 
			{
                std::stringstream ss(itatt.second);
                Vector v(1,1,1);
                ss >> v.x >> v.y >> v.z;
                if( !ss ) 
				{
                    v.z = v.y = v.x;
                }
                scale_geometry_ *= v;
            }
            else if( itatt.first == "enable" ) 
			{
                is_enabled = !(_stricmp(itatt.second.c_str(), "false") == 0 || itatt.second=="0");
            }
        }

        // if not appending to a body and plink pointer valid, append to it instead
        if( !link_ && !!plink &&( plink->GetParent() == pparent) ) 
		{
            link_ = plink;
        }
        if( linkfilename.size() > 0 ) 
		{
            ParseXMLFile(BaseXMLReaderPtr(new LinkXMLReader(link_, parent_, AttributesList())), linkfilename);
        }

        if( !link_ )
		{
            link_.reset(new KinBody::Link(pparent));
        }
        if( linkname.size() > 0 )
		{
            link_->info_.name_ = linkname;
        }
        if( is_static_set )
		{
            link_->info_.is_static_ = is_static;
        }
        link_->info_.is_enabled_ = is_enabled;
    }
    virtual ~LinkXMLReader() 
	{
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList& atts)
    {
        switch( StreamXMLReader::startElement(xmlname,atts) ) 
		{
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        if( processing_tag_.size() > 0 ) 
		{
            if( processing_tag_ == "mass" ) 
			{
                return (xmlname == "density" 
					|| xmlname == "total" 
					|| xmlname == "radius" 
					|| !(mass_type_ == MT_Box && xmlname == "extents") 
					|| (mass_type_ == MT_Custom && xmlname == "com") 
					|| (mass_type_ == MT_Custom && xmlname == "inertia")) ? PE_Support : PE_Ignore;
            }
            return PE_Ignore;
        }

        if( xmlname == "body" ) 
		{
            orig_transform_ = link_->GetTransform();
            link_->SetTransform(Transform());
            processing_tag_ = "";
            AttributesList newatts = atts;
            newatts.emplace_back("skipgeometry", is_skip_geometry_ ? "1" : "0");
            newatts.emplace_back("scalegeometry", str(boost::format("%f %f %f")%scale_geometry_.x
				%scale_geometry_.y%scale_geometry_.z));
            cur_reader_.reset(new LinkXMLReader(link_, parent_, newatts));
            return PE_Support;
        }

        processing_tag_ = xmlname;
        if( xmlname == "translation" 
			|| xmlname == "rotationmat" 
			|| xmlname == "rotationaxis" 
			|| xmlname == "quat" 
			|| xmlname == "offsetfrom" )
		{
            return PE_Support;
        }
        else if( xmlname == "mass" ) 
		{
            // find the type of mass and create
            mass_type_ = MT_Sphere;
            for(auto itatt:atts)
			{
                if( itatt.first == "type") 
				{
                    if( _stricmp(itatt.second.c_str(), "mimicgeom") == 0 )
					{
                        mass_type_ = MT_MimicGeom;
                    }
                    else if( _stricmp(itatt.second.c_str(), "box") == 0 ) 
					{
                        mass_type_ = MT_Box;
                    }
                    else if( _stricmp(itatt.second.c_str(), "sphere") == 0 ) 
					{
                        mass_type_ = MT_Sphere;
                    }
                    else if( _stricmp(itatt.second.c_str(), "custom") == 0 ) 
					{
                        mass_type_ = MT_Custom;
                    }
                    break;
                }
            }
            return PE_Support;
        }
        else if( processing_tag_ == "geometry" 
			|| processing_tag_ == "geom" ) 
		{
            processing_tag_ = "";
            if( is_skip_geometry_ ) 
			{
                return PE_Ignore;
            }
            cur_reader_.reset(new xmlreaders::GeometryInfoReader(KinBody::GeometryInfoPtr(),atts));
            return PE_Support;
        }

        processing_tag_ = "";
        return PE_Pass;
    }

    virtual bool endElement(const std::string& xmlname)
    {
        if( !!cur_reader_ ) 
		{
            if( cur_reader_->endElement(xmlname) ) 
			{
                if( xmlname == "body" )
				{
                    // directly apply transform to all geomteries
                    Transform tnew = link_->GetTransform();
                    for(auto& itgeom: link_->geometries_vector_) 
					{
                        itgeom->info_.transform_ = tnew * itgeom->info_.transform_;
                    }
                    link_->collision_.ApplyTransform(tnew);
                    link_->SetTransform(orig_transform_);
                }

                xmlreaders::GeometryInfoReaderPtr geomreader 
					= std::dynamic_pointer_cast<xmlreaders::GeometryInfoReader>(cur_reader_);
                if( !!geomreader ) 
				{
                    KinBody::GeometryInfoPtr info = geomreader->GetGeometryInfo();

                    // geometry is not in the default group, so we add it to the LinkInfo without instantiating it
                    std::string groupname = geomreader->GetGroupName();
                    if( groupname != "self" ) 
					{
                        link_->info_.extra_geometries_map_[groupname].push_back(info);
                        cur_reader_.reset();
                        return false;
                    }

                    TransformMatrix tm(info->transform_); tm.trans = Vector();
                    TransformMatrix tminv = tm.inverse();
                    tm.m[0] *= scale_geometry_.x; tm.m[1] *= scale_geometry_.x; tm.m[2] *= scale_geometry_.x;
                    tm.m[4] *= scale_geometry_.y; tm.m[5] *= scale_geometry_.y; tm.m[6] *= scale_geometry_.y;
                    tm.m[8] *= scale_geometry_.z; tm.m[9] *= scale_geometry_.z; tm.m[10] *= scale_geometry_.z;
                    TransformMatrix tmres = tminv * tm;
                    // have to scale in link space, so get scale in geomspace
                    Vector geomspacescale(RaveSqrt(tmres.m[0]*tmres.m[0]+tmres.m[4]*tmres.m[4]+tmres.m[8]*tmres.m[8]),
						RaveSqrt(tmres.m[1]*tmres.m[1]+tmres.m[5]*tmres.m[5]+tmres.m[9]*tmres.m[9]),
						RaveSqrt(tmres.m[2]*tmres.m[2]+tmres.m[6]*tmres.m[6]+tmres.m[10]*tmres.m[10]));

                    if( !!fn_GetModelsDir_ ) 
					{
                        bool bsame = info->render_file_name_ == info->collision_file_name_;
                        info->render_file_name_ = fn_GetModelsDir_(info->render_file_name_);
                        if( bsame ) 
						{
                            info->collision_file_name_ = info->render_file_name_;
                        }
                        else 
						{
                            info->collision_file_name_ = fn_GetModelsDir_(info->collision_file_name_);
                        }
                    }
                    std::list<KinBody::GeometryInfo> geometries_list;
                    if( info->type_ == GT_TriMesh ) 
					{
                        bool is_success = false;
                        if( info->collision_file_name_.size() > 0 )
						{
                            if( !CreateGeometries(parent_->GetEnv(),
								info->collision_file_name_,
								info->collision_scale_vec_, geometries_list) )
							{
                                RAVELOG_WARN(str(boost::format("failed to find %s\n")%info->collision_file_name_));
                            }
                            else 
							{
                                is_success = true;
                            }
                        }
                        if( info->render_file_name_.size() > 0 ) 
						{
                            if( !is_success )
							{
                                if( !CreateGeometries(parent_->GetEnv(), 
									info->render_file_name_, 
									info->render_scale_vec_, 
									geometries_list) ) 
								{
                                    RAVELOG_WARN(str(boost::format("failed to find %s\n")%info->render_file_name_));
                                }
                                else 
								{
                                    is_success = true;
                                }
                            }
                        }
                        if( geometries_list.size() > 0 )
						{
                            // append all the geometries to the link. make sure the render filename is specified in only one geometry.
                            std::string extension;
                            if( info->render_file_name_.find_last_of('.') != std::string::npos ) 
							{
                                extension = info->render_file_name_.substr(info->render_file_name_.find_last_of('.')+1);
                            }
                            for(auto& itnewgeom:geometries_list)
							{
                                itnewgeom.is_visible_ = info->is_visible_;
                                itnewgeom.is_modifiable_ = info->is_modifiable_;
                                itnewgeom.transform_ = info->transform_;
                                itnewgeom.transparency_ = info->transparency_;
                                itnewgeom.render_file_name_ = std::string("__norenderif__:")+extension;
                                for(auto& it:itnewgeom.mesh_collision_.vertices) 
								{
                                    it = tmres * it;
                                }
                                if( geomreader->IsOverwriteDiffuse() ) 
								{
                                    itnewgeom.diffuse_color_vec_ = info->diffuse_color_vec_;
                                }
                                if( geomreader->IsOverwriteAmbient() ) 
								{
                                    itnewgeom.ambient_color_vec_ = info->ambient_color_vec_;
                                }
                                if( geomreader->IsOverwriteTransparency() ) 
								{
                                    itnewgeom.transparency_ = info->transparency_;
                                }
                                itnewgeom.transform_.trans *= scale_geometry_;
                                link_->collision_.Append(itnewgeom.mesh_collision_, itnewgeom.transform_);
                            }
                            geometries_list.front().render_scale_vec_ = info->render_scale_vec_*geomspacescale;
                            geometries_list.front().render_file_name_ = info->render_file_name_;
                            geometries_list.front().collision_scale_vec_ = info->collision_scale_vec_*geomspacescale;
                            geometries_list.front().collision_file_name_ = info->collision_file_name_;
                            geometries_list.front().is_visible_ = info->is_visible_;
                            for(auto& itinfo: geometries_list) 
							{
                                link_->geometries_vector_.push_back(
									KinBody::Link::GeometryPtr(new KinBody::Link::Geometry(link_,itinfo)));
                            }
                        }
                        else 
						{
                            info->render_scale_vec_ = info->render_scale_vec_*geomspacescale;
                            for(auto& it:info->mesh_collision_.vertices) 
							{
                                it = tmres * it;
                            }
                            info->transform_.trans *= scale_geometry_;
                            link_->collision_.Append(info->mesh_collision_, info->transform_);
                            link_->geometries_vector_.push_back(
								KinBody::Link::GeometryPtr(new KinBody::Link::Geometry(link_,*info)));
                        }
                    }
                    else 
					{
                        info->render_scale_vec_ = info->render_scale_vec_*geomspacescale;
                        info->render_file_name_ = info->render_file_name_;
                        if( info->type_ == GT_Cylinder ) 
						{   
							// axis has to point on y
                            // rotate on x axis by pi/2
                            Transform trot;
                            trot.rot = quatFromAxisAngle(Vector(1, 0, 0), PI/2);
                            info->transform_.rot = (info->transform_*trot).rot;
                        }

                        // call before attaching the geom
                        KinBody::Link::GeometryPtr geom(new KinBody::Link::Geometry(link_,*info));
                        geom->info_.InitCollisionMesh();
                        for(auto& it:info->mesh_collision_.vertices) 
						{
                            it = tmres * it;
                        }
                        info->transform_.trans *= scale_geometry_;
                        info->gemo_outer_extents_data_ *= geomspacescale;
                        link_->collision_.Append(geom->GetCollisionMesh(), info->transform_);
                        link_->geometries_vector_.push_back(geom);
                    }
                }

                cur_reader_.reset();
            }
            return false;
        }
        else if( processing_tag_ == "mass" ) 
{
            if( xmlname == "density" ) 
			{
                if( mass_type_ == MT_BoxMass )
				{
                    mass_type_ = MT_Box;
                }
                else if( mass_type_ == MT_SphereMass) 
				{
                    mass_type_ = MT_Sphere;
                }
                else if( mass_type_ == MT_MimicGeomMass) 
				{
                    mass_type_ = MT_MimicGeom;
                }
                string_stream_ >> mass_density_;
            }
            else if( xmlname == "total" ) 
			{
                if( mass_type_ == MT_Box ) 
				{
                    mass_type_ = MT_BoxMass;
                }
                else if( mass_type_ == MT_Sphere) 
				{
                    mass_type_ = MT_SphereMass;
                }
                else if( mass_type_ == MT_MimicGeom) 
				{
                    mass_type_ = MT_MimicGeomMass;
                }
                string_stream_ >> total_mass_;
                mass_custom_.fTotalMass = total_mass_;
            }
            else if( xmlname == "radius" ) 
			{
                string_stream_ >> mass_extents_.x;
                mass_extents_.x *= scale_geometry_.x;
            }
            else if((mass_type_ == MT_Box)&&(xmlname == "extents"))
			{
                string_stream_ >> mass_extents_.x >> mass_extents_.y >> mass_extents_.z;
                mass_extents_ *= scale_geometry_;
            }
            else if( xmlname == processing_tag_ ) 
			{
                processing_tag_ = "";
            }
            else if( mass_type_ == MT_Custom ) 
			{
                if( xmlname == "com" ) 
				{
                    string_stream_ >> mass_custom_.t.trans.x >> mass_custom_.t.trans.y >> mass_custom_.t.trans.z;
                    mass_custom_.t.trans = mass_custom_.t.trans*scale_geometry_;
                }
                else if( xmlname == "inertia" ) 
				{
                    string_stream_ >> mass_custom_.t.m[0] >> mass_custom_.t.m[1] >> mass_custom_.t.m[2] >> mass_custom_.t.m[4] >> mass_custom_.t.m[5] >> mass_custom_.t.m[6] >> mass_custom_.t.m[8] >> mass_custom_.t.m[9] >> mass_custom_.t.m[10];
                }
            }
            return false;
        }

        if( xmlname == "body" ) 
		{
            if(( link_->GetGeometries().size() == 0) && !is_skip_geometry_) 
			{
                RAVELOG_VERBOSE(str(boost::format("link %s has no geometry attached!\n")%link_->GetName()));
            }
            // perform final processing stages
            MASS totalmass;
            if( mass_type_ == MT_MimicGeom ) 
			{
                for(auto itgeom: link_->GetGeometries()) {
                    MASS mass;
                    switch(itgeom->GetType()) {
                    case GT_Sphere:
                        mass = MASS::GetSphericalMassD(itgeom->GetSphereRadius(), Vector(),mass_density_);
                        break;
                    case GT_Box:
                        mass = MASS::GetBoxMassD(itgeom->GetBoxExtents(), Vector(), mass_density_);
                        break;
                    case GT_Cage:
                        mass = MASS::GetBoxMassD(0.5*itgeom->GetContainerOuterExtents(), Vector(), mass_density_);
                        break;
                    case GT_Container:
                        mass = MASS::GetBoxMassD(0.5*itgeom->GetContainerOuterExtents(), Vector(), mass_density_);
                        break;
                    case GT_Cylinder:
                        mass = MASS::GetCylinderMassD(itgeom->GetCylinderRadius(), itgeom->GetCylinderHeight(),
							Vector(), mass_density_);
                        break;
                    default:
                        break;
                    }

                    totalmass += mass.ChangeCoordinateSystem(itgeom->GetTransform());
                }
            }
            else if( mass_type_ == MT_MimicGeomMass ) {
                std::vector<MASS> masses;
                dReal dummytotal=0;
                FOREACHC(itgeom, link_->GetGeometries()) {
                    MASS mass;
                    switch((*itgeom)->GetType()) {
                    case GT_Sphere:
                        mass = MASS::GetSphericalMassD((*itgeom)->GetSphereRadius(), Vector(),1000);
                        break;
                    case GT_Box:
                        mass = MASS::GetBoxMassD((*itgeom)->GetBoxExtents(), Vector(), 1000);
                        break;
                    case GT_Container:
                        mass = MASS::GetBoxMassD(0.5*(*itgeom)->GetContainerOuterExtents(), Vector(), 1000);
                        break;
                    case GT_Cylinder:
                        mass = MASS::GetCylinderMassD((*itgeom)->GetCylinderRadius(), (*itgeom)->GetCylinderHeight(), Vector(), 1000);
                        break;
                    default:
                        break;
                    }
                    masses.push_back(mass.ChangeCoordinateSystem((*itgeom)->GetTransform()));
                    //Lazily find the effective volume by finding a dummy total mass
                    dummytotal+=mass.fTotalMass;
                }
                RAVELOG_VERBOSE("Total dummy mass is %f\n",dummytotal);
                dReal ratio=total_mass_/dummytotal;
                //Store new masses based on the percentage of the total geometry volume in each piece
                FOREACH(itmass,masses){
                    itmass->fTotalMass=itmass->fTotalMass*ratio;
                    RAVELOG_VERBOSE("Assigning mass  %f\n",itmass->fTotalMass);
                    totalmass += *itmass;
                }

            }
            else if( mass_type_ == MT_Box ) {
                totalmass = MASS::GetBoxMassD(mass_extents_, Vector(), mass_density_);
            }
            else if( mass_type_ == MT_BoxMass ) {
                totalmass = MASS::GetBoxMass(mass_extents_, Vector(), total_mass_);
            }
            else if( mass_type_ == MT_Sphere ) {
                totalmass = MASS::GetSphericalMassD(mass_extents_.x, Vector(), mass_density_);
            }
            else if( mass_type_ == MT_Custom ) {
                totalmass = mass_custom_;
            }
            else {
                totalmass = MASS::GetSphericalMass(mass_extents_.x, Vector(), total_mass_);
            }

            totalmass.GetMassFrame(link_->info_.mass_frame_transform_, link_->info_.inertia_moments_vector_);
            link_->info_.mass_ =totalmass.fTotalMass;
            orig_transform_ = link_->GetTransform();

            Transform cur;
            if( !!_offsetfrom ) {
                // recompute new transformation
                Transform root;
                if( !!_fnGetOffsetFrom ) {
                    root = _fnGetOffsetFrom(_offsetfrom);
                }
                else {
                    root = _offsetfrom->GetTransform();
                }
                cur = link_->GetTransform();
                cur = root * cur;
                orig_transform_ = root * orig_transform_;         // update orig trans separately
                link_->SetTransform(cur);
            }

            return true;
        }

        if( xmlname == "translation" ) {
            Vector v;
            string_stream_ >> v.x >> v.y >> v.z;
            link_->info_.transform_.trans += v*scale_geometry_;
        }
        else if( xmlname == "rotationmat" ) {
            TransformMatrix tnew;
            string_stream_ >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
            link_->info_.transform_.rot = (Transform(tnew)*link_->info_.transform_).rot;
        }
        else if( xmlname == "rotationaxis" ) {
            Vector vaxis; dReal fangle=0;
            string_stream_ >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
            Transform tnew; tnew.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
            link_->info_.transform_.rot = (tnew*link_->info_.transform_).rot;
        }
        else if( xmlname == "quat" ) {
            Transform tnew;
            string_stream_ >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
            tnew.rot.normalize4();
            link_->info_.transform_.rot = (tnew*link_->info_.transform_).rot;
        }
        else if( xmlname == "offsetfrom" ) {
            // figure out which body
            string linkname;
            string_stream_ >> linkname;
            _offsetfrom = parent_->GetLink(linkname);
            if( !_offsetfrom ) {
                RAVELOG_WARN(str(boost::format("Failed to find offsetfrom body %s\n")%linkname));
                GetXMLErrorCount()++;
            }
        }

        if( xmlname !=processing_tag_ ) {
            RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%processing_tag_));
        }
        processing_tag_ = "";
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( processing_tag_.size() > 0 ) {
            string_stream_.clear();
            string_stream_ << ch;
        }
        else {
            StreamXMLReader::characters(ch);
        }
    }

    void SetMassType(MassType type, float fValue, const Vector &vMassExtents)
    {
        mass_type_ = type;
        mass_density_ = total_mass_ = fValue;
        mass_extents_ = vMassExtents;
    }

    Transform GetOrigTransform() const {
        return orig_transform_;
    }

    boost::function<string(const std::string&)> fn_GetModelsDir_;
    boost::function<Transform(KinBody::LinkPtr)> _fnGetOffsetFrom;

private:
    MASS mass_;                            //!< current mass of the object
    KinBody::LinkPtr& link_;
    KinBodyPtr parent_;
    KinBody::LinkPtr _offsetfrom;                            //!< all transformations are relative to the this body
    bool is_skip_geometry_;
    Vector scale_geometry_;
    Transform orig_transform_;

    // Mass
    MassType mass_type_;                   //!< if true, mass is craeted so that it mimics the geometry
    std::string processing_tag_;         /// if not empty, currently processing
    MASS mass_custom_;
    float mass_density_, total_mass_;
    Vector mass_extents_;                   //!< used only if mass is a box
};

bool CreateGeometries(EnvironmentBasePtr penv, const std::string& filename, 
	const Vector &vscale, std::list<KinBody::GeometryInfo>& geometries_list)
{
    return LinkXMLReader::CreateGeometries(penv,filename,vscale,geometries_list);
}

// Joint Reader
class JointXMLReader : public StreamXMLReader
{
public:
    JointXMLReader(KinBody::JointPtr& pjoint, KinBodyPtr pparent, const AttributesList &atts) : _pjoint(pjoint) {
        _bNegateJoint = false;
        _pparent = pparent;
        _pjoint.reset(new KinBody::Joint(pparent));
        _pjoint->info_.type_ = KinBody::JointHinge;
        _vScaleGeometry = Vector(1,1,1);

        FOREACHC(itatt,atts) {
            if( itatt->first == "name" ) {
                _pjoint->info_.name_ = itatt->second;
            }
            else if( itatt->first == "type" ) {
                if( _stricmp(itatt->second.c_str(), "hinge") == 0 ) {
                    _pjoint->info_.type_ = KinBody::JointHinge;
                }
                else if( _stricmp(itatt->second.c_str(), "slider") == 0 ) {
                    _pjoint->info_.type_ = KinBody::JointSlider;
                }
                else if( _stricmp(itatt->second.c_str(), "universal") == 0 ) {
                    _pjoint->info_.type_ = KinBody::JointUniversal;
                }
                else if( _stricmp(itatt->second.c_str(), "hinge2") == 0 ) {
                    _pjoint->info_.type_ = KinBody::JointHinge2;
                }
                else if( _stricmp(itatt->second.c_str(), "spherical") == 0 ) {
                    _pjoint->info_.type_ = KinBody::JointSpherical;
                }
                else {
                    RAVELOG_WARN(str(boost::format("unrecognized joint type: %s, setting to hinge\n")%itatt->second));
                    _pjoint->info_.type_ = KinBody::JointHinge;
                }
            }
            else if( itatt->first == "enable" ) {
                _pjoint->info_.is_active_ = !(_stricmp(itatt->second.c_str(), "false") == 0 || itatt->second=="0");
            }
            else if( itatt->first == "mimic" ) {
                RAVELOG_WARN("mimic attribute on <joint> tag is deprecated! Use mimic_pos, mimic_vel, and mimic_accel\n");
                stringstream ss(itatt->second);
                dReal a=1, b=0;
                string strmimicjoint;
                ss >> strmimicjoint >> a >> b;
                if( !ss ) {
                    RAVELOG_WARN(str(boost::format("failed to set mimic properties correctly from: %s\n")%itatt->second));
                }
                _pjoint->mimic_array_[0].reset(new KinBody::Mimic());
                _pjoint->mimic_array_[0]->_equations[0] = str(boost::format("%s*%f+%f")%strmimicjoint%a%b);
                _pjoint->mimic_array_[0]->_equations[1] = str(boost::format("|%s %f")%strmimicjoint%a);
                _pjoint->mimic_array_[0]->_equations[2] = str(boost::format("|%s %f")%strmimicjoint%a);
            }
            else if( itatt->first.size() >= 9&&itatt->first.substr(0,9) == "mimic_pos") {
                if( !_pjoint->mimic_array_[0] ) {
                    _pjoint->mimic_array_[0].reset(new KinBody::Mimic());
                }
                _pjoint->mimic_array_[0]->_equations[0] = itatt->second;
            }
            else if( itatt->first.size() >= 9 && itatt->first.substr(0,9) == "mimic_vel") {
                if( !_pjoint->mimic_array_[0] ) {
                    _pjoint->mimic_array_[0].reset(new KinBody::Mimic());
                }
                _pjoint->mimic_array_[0]->_equations[1] = itatt->second;
            }
            else if( itatt->first.size() >= 11 && itatt->first.substr(0,11) == "mimic_accel") {
                if( !_pjoint->mimic_array_[0] ) {
                    _pjoint->mimic_array_[0].reset(new KinBody::Mimic());
                }
                _pjoint->mimic_array_[0]->_equations[2] = itatt->second;
            }
            else if( itatt->first == "circular" ) {
                _pjoint->info_.is_circular_[0] = !(_stricmp(itatt->second.c_str(), "false") == 0 || itatt->second=="0");
                for(int i = 1; i < _pjoint->GetDOF(); ++i) {
                    _pjoint->info_.is_circular_[i] = _pjoint->info_.is_circular_[0];
                }
            }
            else if( itatt->first == "scalegeometry" ) {
                stringstream ss(itatt->second);
                Vector v(1,1,1);
                ss >> v.x >> v.y >> v.z;
                if( !ss ) {
                    v.z = v.y = v.x;
                }
                _vScaleGeometry *= v;
            }
        }

        _vAxes.resize(_pjoint->GetDOF());
        if( _pjoint->GetType() == KinBody::JointSlider ) {
            for(int i = 0; i < _pjoint->GetDOF(); ++i) {
                _pjoint->info_.lower_limit_vector_.at(i) = -10;
                _pjoint->info_.upper_limit_vector_.at(i) = 10;
            }
        }
        else if( _pjoint->GetType() == KinBody::JointSpherical ) {
            _vAxes.at(0) = Vector(1,0,0);
            _vAxes.at(1) = Vector(0,1,0);
            _vAxes.at(2) = Vector(0,0,1);
            for(int i = 0; i < _pjoint->GetDOF(); ++i) {
                _pjoint->info_.lower_limit_vector_.at(i) = -100;
                _pjoint->info_.upper_limit_vector_.at(i) = 100;
            }
        }
        else {
            for(int i = 0; i < _pjoint->GetDOF(); ++i) {
                _pjoint->info_.lower_limit_vector_.at(i) = -PI;
                _pjoint->info_.upper_limit_vector_.at(i) = PI;
            }
        }
        FOREACH(it,_pjoint->info_.weights_vector_) {
            *it = 1;
        }
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList &atts)
    {
        switch( StreamXMLReader::startElement(xmlname,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        if( xmlname == "actuator" ) {
            _processingtag = "";
            cur_reader_.reset(new xmlreaders::ElectricMotorActuatorInfoReader(ElectricMotorActuatorInfoPtr(), atts));
            return PE_Support;
        }

        static std::array<string, 24> tags = { { "body", "offsetfrom", "weight", "lostop", "histop", "limits", "limitsrad", "limitsdeg", "maxvel", "maxveldeg", "hardmaxvel", "maxaccel", "maxacceldeg", "maxtorque", "maxinertia", "maxforce", "resolution", "anchor", "axis", "axis1", "axis2", "axis3", "mode", "initial" }};
        if( find(tags.begin(),tags.end(),xmlname) != tags.end() ) {
            _processingtag = xmlname;
            return PE_Support;
        }
        return PE_Pass;
    }

    virtual bool endElement(const std::string& xmlname)
    {
        int numindices = _pjoint->GetDOF();
        dReal fRatio = _pjoint->info_.type_ == KinBody::JointSlider ? (dReal)1 : (dReal)PI / 180.0f;         // most, but not all, joint take degrees

        if( !!cur_reader_ ) {
            if( cur_reader_->endElement(xmlname) ) {
                xmlreaders::ElectricMotorActuatorInfoReaderPtr actuatorreader = std::dynamic_pointer_cast<xmlreaders::ElectricMotorActuatorInfoReader>(cur_reader_);
                if( !!actuatorreader ) {
                    _pjoint->info_.electric_motor_info_ = actuatorreader->GetActuatorInfo();
                }
                cur_reader_.reset();
            }
        }
        else if( xmlname == "joint" ) {
            if( _pparent->GetLinks().size() == 0 ) {
                RAVELOG_WARN("parent kinbody has no links defined yet!\n");
                return false;
            }

            if( _bNegateJoint ) {
                FOREACH(itaxis,_vAxes) {
                    *itaxis = -*itaxis;
                }
            }

            string defaultname = "J_";
            // check if joint needs an artificial offset, only for revolute joints that have identifying points!
            if(( _pjoint->info_.type_ == KinBody::JointUniversal) ||( _pjoint->info_.type_ == KinBody::JointHinge2) ||( _pjoint->info_.type_ == KinBody::JointHinge) ) {
                for(int i = 0; i < numindices; ++i) {
                    if(( _pjoint->info_.lower_limit_vector_[i] < -PI) ||( _pjoint->info_.upper_limit_vector_[i] > PI) ) {
                        // TODO, necessary?
                        _pjoint->info_.offsets_vector_[i] = 0.5f * (_pjoint->info_.lower_limit_vector_[i] + _pjoint->info_.upper_limit_vector_[i]);
                    }
                }
            }

            Transform toffsetfrom;
            if( !!_offsetfrom ) {
                if( !!_fnGetOffsetFrom ) {
                    toffsetfrom = _fnGetOffsetFrom(_offsetfrom);
                }
                else {
                    toffsetfrom = _offsetfrom->GetTransform();
                }
            }

            toffsetfrom = attachedbodies[0]->GetTransform().inverse() * toffsetfrom;
            FOREACH(itaxis,_vAxes) {
                *itaxis = toffsetfrom.rotate(*itaxis);
            }
            _pjoint->_ComputeInternalInformation(attachedbodies[0],attachedbodies[1],toffsetfrom*_vanchor,_vAxes,_vinitialvalues);
            return true;
        }
        else if( xmlname == "weight" ) {
            for(int i = 0; i < numindices; ++i) {
                string_stream_ >> _pjoint->info_.weights_vector_.at(i);
            }
        }
        else if( xmlname == "initial" ) {
            _vinitialvalues = std::vector<dReal>((istream_iterator<dReal>(string_stream_)), istream_iterator<dReal>());
        }
        else if( xmlname == "body" ) {
            // figure out which body
            int index = !attachedbodies[0] ? 0 : 1;
            bool bQuery = true;
            string linkname;
            string_stream_ >> linkname;

            FOREACHC(itlink, _pparent->GetLinks()) {
                if( _stricmp((*itlink)->GetName().c_str(), linkname.c_str()) == 0 ) {
                    bQuery = !(*itlink)->IsStatic();
                    attachedbodies[index] = *itlink;
                    break;
                }
            }

            if( !attachedbodies[index] && bQuery ) {
                RAVELOG_WARN(str(boost::format("Failed to find body %s for joint %s\n")%linkname%_pjoint->info_.name_));
                GetXMLErrorCount()++;
            }
        }
        else if((xmlname == "limits")||(xmlname == "limitsrad")||(xmlname == "limitsdeg")) {
            if( _bNegateJoint ) {
                throw OpenRAVEException(_tr("cannot specify <limits> with <lostop> and <histop>, choose one"));
            }
            dReal fmult = xmlname == "limitsdeg" ? fRatio : dReal(1.0);
            vector<dReal> values = vector<dReal>((istream_iterator<dReal>(string_stream_)), istream_iterator<dReal>());
            if( (int)values.size() == 2*_pjoint->GetDOF() ) {
                for(int i = 0; i < _pjoint->GetDOF(); ++i ) {
                    _pjoint->info_.lower_limit_vector_.at(i) = fmult*min(values[2*i+0],values[2*i+1]);
                    _pjoint->info_.upper_limit_vector_.at(i) = fmult*max(values[2*i+0],values[2*i+1]);
                }
            }
            else {
                RAVELOG_WARN(str(boost::format("<limits> tag has %d values, expected %d! ignoring...\n")%values.size()%_pjoint->GetDOF()));
            }
        }
        else if( xmlname == "lostop" ) {
            _bNegateJoint = true;
            RAVELOG_ERROR(str(boost::format("%s: <lostop> is deprecated, please use <limits> (now in radians), <limitsrad>, or <limitsdeg> tag and negate your joint axis!\n")%_pparent->GetName()));
            for(int i = 0; i < numindices; ++i) {
                string_stream_ >> _pjoint->info_.lower_limit_vector_.at(i);
                _pjoint->info_.lower_limit_vector_.at(i) *= fRatio;
            }
        }
        else if( xmlname == "histop" ) {
            _bNegateJoint = true;
            RAVELOG_ERROR(str(boost::format("%s: <histop> deprecated, please use <limits> (now in radians), <limitsrad>, <limitsdeg> tag and negate your joint axis!\n")%_pparent->GetName()));
            for(int i = 0; i < numindices; ++i) {
                string_stream_ >> _pjoint->info_.upper_limit_vector_.at(i);
                _pjoint->info_.upper_limit_vector_.at(i) *= fRatio;
            }
        }
        else if( xmlname == "maxvel" ) {
            for(int idof = 0; idof < _pjoint->GetDOF(); ++idof) {
                string_stream_ >> _pjoint->info_.max_velocity_vector_[idof];
            }
        }
        else if( xmlname == "maxveldeg" ) {
            for(int idof = 0; idof < _pjoint->GetDOF(); ++idof) {
                string_stream_ >> _pjoint->info_.max_velocity_vector_[idof];
                _pjoint->info_.max_velocity_vector_[idof] *= PI/180.0;
            }
        }
        else if( xmlname == "hardmaxvel" ) {
            for(int idof = 0; idof < _pjoint->GetDOF(); ++idof) {
                string_stream_ >> _pjoint->info_.hard_max_velocity_vector_[idof];
            }
        }
        else if( xmlname == "maxaccel" ) {
            for(int idof = 0; idof < _pjoint->GetDOF(); ++idof) {
                string_stream_ >> _pjoint->info_.max_accelerate_vector_[idof];
            }
        }
        else if( xmlname == "maxacceldeg" ) {
            for(int idof = 0; idof < _pjoint->GetDOF(); ++idof) {
                string_stream_ >> _pjoint->info_.max_accelerate_vector_[idof];
                _pjoint->info_.max_accelerate_vector_[idof] *= PI/180.0;
            }
        }
        else if( xmlname == "maxtorque" ) {
            for(int idof = 0; idof < _pjoint->GetDOF(); ++idof) {
                string_stream_ >> _pjoint->info_.max_torque_vector_[idof];
            }
        }
        else if( xmlname == "maxinertia" ) {
            for(int idof = 0; idof < _pjoint->GetDOF(); ++idof) {
                string_stream_ >> _pjoint->info_.max_inertia_vector_[idof];
            }
        }
        else if( xmlname == "resolution" ) {
            dReal fResolution = 0.02;
            string_stream_ >> fResolution;
            fResolution *= fRatio;
            FOREACH(itvalue, _pjoint->info_.resolution_vector_) {
                *itvalue = fResolution;
            }
        }
        else if( xmlname == "offsetfrom" ) {
            // figure out which body
            string linkname; string_stream_ >> linkname;
            _offsetfrom = _pparent->GetLink(linkname);

            if( !_offsetfrom ) {
                RAVELOG_WARN(str(boost::format("Failed to find body %s\n")%linkname));
                GetXMLErrorCount()++;
            }
        }
        else {
            // could be type specific
            switch(_pjoint->info_.type_) {
            case KinBody::JointHinge:
                if( xmlname == "anchor" ) {
                    string_stream_ >> _vanchor.x >> _vanchor.y >> _vanchor.z;
                    _vanchor *= _vScaleGeometry;
                }
                else if( xmlname == "axis" ) {
                    string_stream_ >> _vAxes.at(0).x >> _vAxes.at(0).y >> _vAxes.at(0).z;
                    _vAxes.at(0).normalize3();
                }
                break;
            case KinBody::JointSlider:
                if( xmlname == "axis" ) {
                    string_stream_ >> _vAxes.at(0).x >> _vAxes.at(0).y >> _vAxes.at(0).z;
                    _vAxes.at(0).normalize3();
                }
                break;
            case KinBody::JointUniversal:
                if( xmlname == "anchor" ) {
                    string_stream_ >> _vanchor.x >> _vanchor.y >> _vanchor.z;
                    _vanchor *= _vScaleGeometry;
                }
                else if( xmlname == "axis1" ) {
                    string_stream_ >> _vAxes.at(0).x >> _vAxes.at(0).y >> _vAxes.at(0).z;
                    _vAxes.at(0).normalize3();
                }
                else if( xmlname == "axis2" ) {
                    string_stream_ >> _vAxes.at(1).x >> _vAxes.at(1).y >> _vAxes.at(1).z;
                    _vAxes.at(0).normalize3();
                }
                break;
            case KinBody::JointHinge2:
                if( xmlname == "anchor" ) {
                    string_stream_ >> _vanchor.x >> _vanchor.y >> _vanchor.z;
                    _vanchor *= _vScaleGeometry;
                }
                else if( xmlname == "axis1" ) {
                    string_stream_ >> _vAxes.at(0).x >> _vAxes.at(0).y >> _vAxes.at(0).z;
                    _vAxes.at(0).normalize3();
                }
                else if( xmlname == "axis2" ) {
                    string_stream_ >> _vAxes.at(1).x >> _vAxes.at(1).y >> _vAxes.at(1).z;
                    _vAxes.at(0).normalize3();
                }
                break;
            case KinBody::JointSpherical:
                if( xmlname == "anchor" ) {
                    string_stream_ >> _vanchor.x >> _vanchor.y >> _vanchor.z;
                    _vanchor *= _vScaleGeometry;
                }
                break;
            default:
                throw OpenRAVEException(str(boost::format(_tr("bad joint type: 0x%x"))%_pjoint->info_.type_));
                break;
            }
        }

        _processingtag.resize(0);
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( _processingtag.size() > 0 ) {
            string_stream_.clear();
            string_stream_ << ch;
        }
        else {
            StreamXMLReader::characters(ch);
        }
    }

    boost::function<string(const std::string&)> _fnGetModelsDir;
    boost::function<Transform(KinBody::LinkPtr)> _fnGetOffsetFrom;

private:
    KinBody::LinkPtr _offsetfrom;         //!< all transforms are relative to this body
    KinBodyPtr _pparent;
    KinBody::JointPtr& _pjoint;
    std::vector<Vector> _vAxes;
    Vector _vanchor, _vScaleGeometry;
    bool _bNegateJoint;
    string _processingtag;
    std::array<KinBody::LinkPtr,2> attachedbodies;
    std::vector<dReal> _vinitialvalues;
};

class InterfaceXMLReader;
typedef std::shared_ptr<InterfaceXMLReader> InterfaceXMLReaderPtr;
typedef std::shared_ptr<InterfaceXMLReader const> InterfaceXMLReaderConstPtr;

class InterfaceXMLReader : public StreamXMLReader
{
public:
    InterfaceXMLReader(EnvironmentBasePtr penv, 
		InterfaceBasePtr& pinterface, InterfaceType type, 
		const std::string &xmltag, const AttributesList &atts)
		: _penv(penv), _type(type), _pinterface(pinterface), _xmltag(xmltag) 
	{
        is_processed_last_tag_ = false;
        _atts = atts;
        std::string strtype;
        FOREACHC(itatt,atts)
		{
            if( itatt->first == "type" ) 
			{
                strtype = itatt->second;
            }
            else if( itatt->first == "file" ) {
                AttributesList listnewatts;
                FOREACHC(itatt2,atts) {
                    if( itatt2->first != "file" ) {
                        listnewatts.push_back(*itatt2);
                    }
                }

                //BaseXMLReaderPtr preader = CreateInterfaceReader(_penv,_type,_pinterface, xmltag, listnewatts);
                //bool is_success = ParseXMLFile(preader, itatt->second);
                string filedata = RaveFindLocalFile(itatt->second,GetParseDirectory());
                if( filedata.size() == 0 ) {
                    continue;
                }

                try {
#ifdef HAVE_BOOST_FILESYSTEM
                    SetParseDirectoryScope scope(boost::filesystem::path(filedata).parent_path().string());
#endif
                    if( !_pinterface ) 
					{
                        // reason to bring all the other attributes since interface is not created yet? (there might be a problem with this?)
                        switch(_type) 
						{
                        case PT_KinBody:
                            _pinterface = _penv->ReadKinBodyURI(KinBodyPtr(), filedata, listnewatts);
                            break;
                        case PT_Robot:
                            _pinterface = _penv->ReadRobotURI(RobotBasePtr(), filedata, listnewatts);
                            break;
                        default:
                            _pinterface = _penv->ReadInterfaceURI(filedata, listnewatts);
                        }
                        if( !!_pinterface &&( _pinterface->GetInterfaceType() != _type) )
						{
                            RAVELOG_ERROR(str(boost::format("unexpected interface created %s\n")
								%RaveGetInterfaceName(_pinterface->GetInterfaceType())));
                            _pinterface.reset();
                        }
                    }
                    else 
					{
                        switch(_type)
						{
                        case PT_KinBody:
                            _pinterface = _penv->ReadKinBodyURI(RaveInterfaceCast<KinBody>(_pinterface),filedata,listnewatts);
                            break;
                        case PT_Robot:
                            _pinterface = _penv->ReadRobotURI(RaveInterfaceCast<RobotBase>(_pinterface),filedata,listnewatts);
                            break;
                        default:
                            _pinterface = _penv->ReadInterfaceURI(_pinterface,_type,filedata,listnewatts);
                        }

                    }
                }
                catch(const std::exception& ex) 
				{
                    RAVELOG_ERROR(str(boost::format("failed to process %s: %s\n")%itatt->second%ex.what()));
                    _pinterface.reset();
                }
//                catch(...) {
//                    RAVELOG_ERROR(str(boost::format("failed to process %s\n")%itatt->second));
//                    _pinterface.reset();
//                }

                if( !_pinterface ) 
				{
                    RAVELOG_DEBUG(str(boost::format("Failed to load filename %s\n")%itatt->second));
                    GetXMLErrorCount()++;
                    break;
                }
                file_name_ = _pinterface->GetURI();
            }
        }

        if( _xmltag.size() == 0 ) {
            _xmltag = RaveGetInterfaceName(_type);
        }

        if( strtype.size() > 0 ) {
            _pinterface = RaveCreateInterface(_penv, _type,strtype);
            if( !_pinterface ) {
                RAVELOG_ERROR(str(boost::format("xml readers failed to create instance of type %s:%s\n")%RaveGetInterfaceName(_type)%strtype));
                GetXMLErrorCount()++;
            }
            else {
                _pcustomreader = RaveCallXMLReader(_pinterface->GetInterfaceType(),_pinterface->GetXMLId(),_pinterface,_atts);
                if( !!_pcustomreader ) {
                    // should set a name for it to get stored
                    _readername = _pinterface->GetXMLId();
                }
                SetFilename(file_name_);
            }
        }
    }

    virtual void _CheckInterface()
    {
        if( !_pinterface ) {
            if( !_pinterface ) {
                switch(_type) {
                case PT_KinBody:
                    _pinterface = RaveCreateKinBody(_penv);
                    break;
                case PT_Robot:
                    _pinterface = RaveCreateInterface(_penv, PT_Robot, "GenericRobot");
                    if( !_pinterface ) {
                        _pinterface = RaveCreateInterface(_penv, PT_Robot, "");
                    }
                    break;
                case PT_Controller:
                    _pinterface = RaveCreateInterface(_penv, PT_Controller, "IdealController");
                    break;
                default:
                    _pinterface = RaveCreateInterface(_penv, _type, "");
                    break;
                }
            }

            if( !_pinterface ) {
                RAVELOG_ERROR(str(boost::format("xml readers failed to create instance of type %ss\n")%RaveGetInterfaceName(_type)));
            }
            else {
                _pcustomreader = RaveCallXMLReader(_pinterface->GetInterfaceType(),_pinterface->GetXMLId(),_pinterface,_atts);
                if( !!_pcustomreader ) {
                    // should set a name for it to get stored
                    _readername = _pinterface->GetXMLId();
                }
            }

            SetFilename(file_name_);
        }
    }

    void SetFilename(const string &filename)
    {
        if( !!_pinterface &&( _pinterface->str_uri_.size() == 0) ) {
            _pinterface->str_uri_ = filename;
        }
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList &atts)
    {
        if( !!_pcustomreader ) {
            return _pcustomreader->startElement(xmlname, atts);
        }

        switch( StreamXMLReader::startElement(xmlname,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        // check for registers readers
        if( !!_pinterface ) {
            _pcustomreader = RaveCallXMLReader(_type,xmlname,_pinterface,atts);
            if( !!_pcustomreader ) {
                _readername = xmlname;
                if( !!_pcustomreader ) {
                    return PE_Support;
                }
            }
        }

        if (xmlname == "sendcommand" ) {
            _interfaceprocessingtag = xmlname;
            return PE_Support;
        }

        return PE_Pass;
    }

    virtual bool endElement(const std::string& xmlname)
    {
        if( !!cur_reader_ ) {
            if( cur_reader_->endElement(xmlname) ) {
                cur_reader_.reset();
            }
        }
        else if( !!_pcustomreader ) {
            if( _pcustomreader->endElement(xmlname) ) {
                _CheckInterface();
                if( _readername.size() > 0 ) {
                    _pinterface->readable_interfaces_map_[_readername] = _pcustomreader->GetReadable();
                }
                _pcustomreader.reset();
                if( xmlname == _xmltag ) {
                    return true;
                }
            }
        }
        else if( _interfaceprocessingtag.size() > 0 ) {
            if( xmlname == "sendcommand" ) {
                _CheckInterface();
                if( !!_pinterface ) {
                    stringstream sout;
                    if( !_pinterface->SendCommand(sout,string_stream_) ) {
                        RAVELOG_WARN("interface command failed\n");
                    }
                }
                else {
                    RAVELOG_INFO(str(boost::format("failed to send command: %s\n")%xmlname));
                }
            }
            _interfaceprocessingtag.resize(0);
        }
        else if( xmlname == _xmltag ) {
            _CheckInterface();
            if( is_processed_last_tag_ ) {
                RAVELOG_WARN(str(boost::format("already processed last tag for %s!\n")%xmlname));
            }
            is_processed_last_tag_ = true;
            return true;
        }
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( _interfaceprocessingtag.size() > 0 ) {
            string_stream_.clear();
            string_stream_ << ch;
        }
        else if( !!_pcustomreader ) {
            _pcustomreader->characters(ch);
        }
        else {
            StreamXMLReader::characters(ch);
        }
    }

    virtual XMLReadablePtr GetReadable() {
        return XMLReadablePtr(new InterfaceXMLReadable(_pinterface));
    }
protected:
    EnvironmentBasePtr _penv;
    InterfaceType _type;
    InterfaceBasePtr& _pinterface;
    BaseXMLReaderPtr _pcustomreader;
    string _xmltag, _interfaceprocessingtag;
    string _interfacename, _readername;
    bool is_processed_last_tag_;
    AttributesList _atts;
};
/// KinBody reader
/// reads kinematic chain specific entries, can instantiate this reader from another reader
class KinBodyXMLReader : public InterfaceXMLReader
{
public:
    KinBodyXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pchain, InterfaceType type, const AttributesList &atts, int roottransoffset) : InterfaceXMLReader(penv,pchain,type,"kinbody",atts), roottransoffset(roottransoffset) {
        _bSkipGeometry = false;
        _vScaleGeometry = Vector(1,1,1);
        _masstype = MT_None;
        _fMassValue = 1;
        _vMassExtents = Vector(1,1,1);
        _bOverwriteDiffuse = false;
        _bOverwriteAmbient = false;
        _bOverwriteTransparency = false;
        _bMakeJoinedLinksAdjacent = true;
        rootoffset = rootjoffset = rootjpoffset = -1;
        FOREACHC(itatt,atts) {
            if( itatt->first == "prefix" ) {
                _prefix = itatt->second;
            }
            else if( itatt->first == "name" ) {
                _bodyname = itatt->second;
            }
            else if( itatt->first == "makejoinedlinksadjacent") {
                _bMakeJoinedLinksAdjacent = atoi(itatt->second.c_str())!=0;
            }
            else if( itatt->first == "skipgeometry" ) {
                _bSkipGeometry = _stricmp(itatt->second.c_str(), "true") == 0 || itatt->second=="1";
            }
            else if( itatt->first == "scalegeometry" ) {
                stringstream ss(itatt->second);
                Vector v(1,1,1);
                ss >> v.x >> v.y >> v.z;
                if( !ss ) {
                    v.z = v.y = v.x;
                }
                _vScaleGeometry *= v;
            }
            else if((itatt->first != "file")&&(itatt->first != "type")) {
                RAVELOG_WARN(str(boost::format("unknown kinbody attribute %s\n")%itatt->first));
            }
        }
        _CheckInterface();
    }

    virtual void _CheckInterface()
    {
        InterfaceXMLReader::_CheckInterface();
        _pchain = RaveInterfaceCast<KinBody>(_pinterface);
        if( !!_pchain &&( rootoffset < 0) ) {
            _pchain->_bMakeJoinedLinksAdjacent = _bMakeJoinedLinksAdjacent;
            rootoffset = (int)_pchain->GetLinks().size();
            rootjoffset = (int)_pchain->GetJoints().size();
            rootjpoffset = (int)_pchain->GetPassiveJoints().size();
            //RAVELOG_INFO(str(boost::format("links: %d, prefix: %s: %x\n")%_pchain->GetLinks().size()%_prefix%this));
            // reisze _vTransforms to be the same size as the initial number of links
            std::vector<dReal> doflastsetvalues;
            _pchain->GetLinkTransformations(_vTransforms, doflastsetvalues);
        }
    }

    Transform GetOffsetFrom(KinBody::LinkPtr plink)
    {
        if( plink->GetIndex() < 0 || plink->GetIndex() >= (int)_vTransforms.size() ) {
            return plink->GetTransform();
        }
        return _vTransforms.at(plink->GetIndex());
    }

    std::string GetModelsDir(const std::string& filename) const
    {
        if( filename.size() == 0 ) {
            return filename;
        }
#ifdef _WIN32
        if( filename.find_first_of(':') != string::npos ) {
            return filename;
        }
#else
        if( filename[0] == '/' ) {
            return filename;
        }
#endif

        std::string fullfilename;
        if( _strModelsDir.size() > 0 ) {
            string s = GetParseDirectory();
            if( s.size() > 0 ) {
                s += s_filesep;
            }
            s += _strModelsDir;
            fullfilename = RaveFindLocalFile(filename, s);
            if( fullfilename.size() > 0 ) {
                return fullfilename;
            }
        }

        // failed to find in _strModelsDir, so try the regular GetParseDirectory()
        return RaveFindLocalFile(filename, GetParseDirectory());
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList &atts)
    {
        if( _processingtag.size() > 0 ) {
            switch( StreamXMLReader::startElement(xmlname,atts) ) {
            case PE_Pass: break;
            case PE_Support: return PE_Support;
            case PE_Ignore: return PE_Ignore;
            }

            if( _processingtag == "mass" ) {
                return (xmlname == "density" || xmlname == "total" || xmlname == "radius" || (_masstype == MT_Box && xmlname == "extents") || (_masstype == MT_Custom && (xmlname == "com"||xmlname == "inertia"))) ? PE_Support : PE_Ignore;
            }
            return PE_Ignore;
        }

        switch( InterfaceXMLReader::startElement(xmlname,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        if( xmlname == "kinbody" ) {
            AttributesList newatts = atts;
            newatts.emplace_back("skipgeometry", _bSkipGeometry ? "1" : "0");
            newatts.emplace_back("scalegeometry", str(boost::format("%f %f %f")%_vScaleGeometry.x%_vScaleGeometry.y%_vScaleGeometry.z));
            cur_reader_ = CreateInterfaceReader(_penv,PT_KinBody,_pinterface, xmlname, newatts);
            return PE_Support;
        }

        _CheckInterface();
        if( xmlname == "body" ) {
            _plink.reset();
            AttributesList newatts = atts;
            newatts.emplace_back("skipgeometry", _bSkipGeometry ? "1" : "0");
            newatts.emplace_back("scalegeometry", str(boost::format("%f %f %f")%_vScaleGeometry.x%_vScaleGeometry.y%_vScaleGeometry.z));
            std::shared_ptr<LinkXMLReader> plinkreader(new LinkXMLReader(_plink, _pchain, newatts));
            plinkreader->SetMassType(_masstype, _fMassValue, _vMassExtents);
            plinkreader->fn_GetModelsDir_ = boost::bind(&KinBodyXMLReader::GetModelsDir,this,_1);
            plinkreader->_fnGetOffsetFrom = boost::bind(&KinBodyXMLReader::GetOffsetFrom,this,_1);
            cur_reader_ = plinkreader;
            return PE_Support;
        }
        else if( xmlname == "joint" ) {
            _pjoint.reset();
            AttributesList newatts = atts;
            newatts.emplace_back("scalegeometry", str(boost::format("%f %f %f")%_vScaleGeometry.x%_vScaleGeometry.y%_vScaleGeometry.z));
            std::shared_ptr<JointXMLReader> pjointreader(new JointXMLReader(_pjoint,_pchain, atts));
            pjointreader->_fnGetModelsDir = boost::bind(&KinBodyXMLReader::GetModelsDir,this,_1);
            pjointreader->_fnGetOffsetFrom = boost::bind(&KinBodyXMLReader::GetOffsetFrom,this,_1);
            cur_reader_ = pjointreader;
            return PE_Support;
        }

        if( xmlname == "mass" ) {
            // find the type of mass and create
            _masstype = MT_Sphere;
            FOREACHC(itatt, atts) {
                if( itatt->first == "type" ) {
                    if( _stricmp(itatt->second.c_str(), "mimicgeom") == 0 ) {
                        _masstype = MT_MimicGeom;
                    }
                    else if( _stricmp(itatt->second.c_str(), "box") == 0 ) {
                        _masstype = MT_Box;
                    }
                    else if( _stricmp(itatt->second.c_str(), "sphere") == 0 ) {
                        _masstype = MT_Sphere;
                    }

                    break;
                }
            }

            _processingtag = xmlname;
            return PE_Support;
        }

        static std::array<string, 10> tags = { { "translation", "rotationmat", "rotationaxis", "quat", "jointvalues", "adjacent", "modelsdir", "diffusecolor", "transparency", "ambientcolor"}};
        if( find(tags.begin(),tags.end(),xmlname) != tags.end() ) {
            _processingtag = xmlname;
            return PE_Support;
        }
        return PE_Pass;
    }

    virtual bool endElement(const std::string& xmlname)
    {
        if( !!cur_reader_ ) {
            if( cur_reader_->endElement(xmlname) ) {
                if( xmlname == "body" ) {
                    if( !_plink )
                        throw OpenRAVEException(_tr("link should be valid"));

                    if( _plink->index_ < 0 ) {
                        // not in array yet
                        _plink->index_ = (int)_pchain->links_vector_.size();
                        _pchain->links_vector_.push_back(_plink);
                        _vTransforms.push_back(Transform());
                    }

                    // do this later, or else offsetfrom will be messed up!
                    _vTransforms.at(_plink->GetIndex()) = std::dynamic_pointer_cast<LinkXMLReader>(cur_reader_)->GetOrigTransform();
                    _plink.reset();
                }
                else if( xmlname == "joint" ) {
                    _pjoint->dof_index_ = _pchain->GetDOF();
                    std::shared_ptr<JointXMLReader> pjointreader = std::dynamic_pointer_cast<JointXMLReader>(cur_reader_);
                    if( _pjoint->info_.is_active_ ) {
                        _pjoint->jointindex = (int)_pchain->joints_vector_.size();
                        _pchain->joints_vector_.push_back(_pjoint);
                    }
                    else {
                        _pjoint->jointindex = -1;
                        _pjoint->dof_index_ = -1;
                        _pchain->passive_joints_vector_.push_back(_pjoint);
                    }
                    BOOST_ASSERT( _pjoint->dof_index_ < _pchain->GetDOF());
                    _pjoint.reset();
                }
                else if( xmlname == "kinbody" ) {
                    // most likely new transforms were added, so update
                    _CheckInterface();
                    std::vector<dReal> doflastsetvalues;
                    _pchain->GetLinkTransformations(_vTransforms, doflastsetvalues);
                }
                else
                    RAVELOG_INFOA(str(boost::format("releasing unknown tag %s\n")%xmlname));

                cur_reader_.reset();
            }
        }
        else if( _processingtag == "mass" ) {
            if( xmlname == "mass" ) {
                _processingtag = "";
            }
            else if( xmlname == "density" ) {
                if( _masstype == MT_BoxMass )
                    _masstype = MT_Box;
                else if( _masstype == MT_SphereMass)
                    _masstype = MT_Sphere;
                string_stream_ >> _fMassValue;
            }
            else if( xmlname == "total" ) {
                if( _masstype == MT_Box ) {
                    _masstype = MT_BoxMass;
                }
                else if( _masstype == MT_Sphere) {
                    _masstype = MT_SphereMass;
                }
                string_stream_ >> _fMassValue;
            }
            else if( xmlname == "radius" ) {
                string_stream_ >> _vMassExtents.x;
            }
            else if( _masstype == MT_Box && xmlname == "extents" ) {
                string_stream_ >> _vMassExtents.x >> _vMassExtents.y >> _vMassExtents.z;
            }
        }
        else if( _processingtag.size() > 0 ) {
            if( xmlname == "translation" ) {
                Vector v;
                string_stream_ >> v.x >> v.y >> v.z;
                _trans.trans += v*_vScaleGeometry;
            }
            else if( xmlname == "rotationaxis" ) {
                Vector vaxis; dReal fangle=0;
                string_stream_ >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
                Transform tnew; tnew.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
                _trans.rot = (tnew*_trans).rot;
            }
            else if( xmlname == "quat" ) {
                Transform tnew;
                string_stream_ >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
                tnew.rot.normalize4();
                _trans.rot = (tnew*_trans).rot;
            }
            else if( xmlname == "rotationmat" ) {
                TransformMatrix tnew;
                string_stream_ >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
                _trans.rot = (Transform(tnew)*_trans).rot;
            }
            else if( xmlname == "adjacent" ) {
                pair<string, string> entry;
                string_stream_ >> entry.first >> entry.second;
                _pchain->_vForcedAdjacentLinks.push_back(entry);
            }
            else if( xmlname == "modelsdir" ) {
                _strModelsDir = string_stream_.str();
                boost::trim(_strModelsDir);
                _strModelsDir += "/";
            }
            else if( xmlname == "diffuseColor" ) {
                // check attributes for format (default is vrml)
                string_stream_ >> _diffusecol.x >> _diffusecol.y >> _diffusecol.z;
                _bOverwriteDiffuse = true;
            }
            else if( xmlname == "ambientColor" ) {
                // check attributes for format (default is vrml)
                string_stream_ >> _ambientcol.x >> _ambientcol.y >> _ambientcol.z;
                _bOverwriteAmbient = true;
            }
            else if( xmlname == "transparency" ) {
                string_stream_ >> _transparency;
                _bOverwriteTransparency = true;
            }
            else if( xmlname == "jointvalues" ) {
                _vjointvalues.reset(new std::vector<dReal>((istream_iterator<dReal>(string_stream_)), istream_iterator<dReal>()));
            }

            if( xmlname !=_processingtag ) {
                RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
            }
            _processingtag = "";
        }
        else if( InterfaceXMLReader::endElement(xmlname) ) {
            if( _bodyname.size() > 0 ) {
                _pchain->SetName(_bodyname);
            }
            if( file_name_.size() > 0 ) {
                SetFilename(file_name_);
            }

            // add prefix
            if( _prefix.size() > 0 ) {
                //RAVELOG_INFO("write prefix: links: %d-%d, 0x%x\n",rootoffset,(int)_pchain->links_vector_.size(),this);
                BOOST_ASSERT(rootoffset >= 0 && rootoffset<=(int)_pchain->links_vector_.size());
                for(vector<KinBody::LinkPtr>::iterator itlink = _pchain->links_vector_.begin()+rootoffset; itlink != _pchain->links_vector_.end(); ++itlink) {
                    (*itlink)->info_.name_ = _prefix + (*itlink)->info_.name_;
                }
                BOOST_ASSERT(rootjoffset >= 0 && rootjoffset<=(int)_pchain->joints_vector_.size());
                for(vector<KinBody::JointPtr>::iterator itjoint = _pchain->joints_vector_.begin()+rootjoffset; itjoint != _pchain->joints_vector_.end(); ++itjoint) {
                    (*itjoint)->info_.name_ = _prefix +(*itjoint)->info_.name_;
                }
                BOOST_ASSERT(rootjpoffset >= 0 && rootjpoffset<=(int)_pchain->passive_joints_vector_.size());
                for(vector<KinBody::JointPtr>::iterator itjoint = _pchain->passive_joints_vector_.begin()+rootjpoffset; itjoint != _pchain->passive_joints_vector_.end(); ++itjoint) {
                    (*itjoint)->info_.name_ = _prefix +(*itjoint)->info_.name_;
                }
            }

            if( _bOverwriteDiffuse ) {
                // overwrite the color
                FOREACH(itlink, _pchain->links_vector_) {
                    FOREACH(itgeom, (*itlink)->geometries_vector_) {
                        (*itgeom)->info_.diffuse_color_vec_ = _diffusecol;
                    }
                }
            }
            if( _bOverwriteAmbient ) {
                // overwrite the color
                FOREACH(itlink, _pchain->links_vector_) {
                    FOREACH(itgeom, (*itlink)->geometries_vector_) {
                        (*itgeom)->info_.ambient_color_vec_ = _ambientcol;
                    }
                }
            }
            if( _bOverwriteTransparency ) {
                // overwrite the color
                FOREACH(itlink, _pchain->links_vector_) {
                    FOREACH(itgeom, (*itlink)->geometries_vector_)
                    {
                        (*itgeom)->info_.transparency_ = _transparency;
                    }
                }
            }

            // transform all the bodies with trans
            Transform cur;
            BOOST_ASSERT(roottransoffset>=0 && roottransoffset<=(int)_pchain->links_vector_.size());
            for(vector<KinBody::LinkPtr>::iterator itlink = _pchain->links_vector_.begin()+roottransoffset; itlink != _pchain->links_vector_.end(); ++itlink) {
                (*itlink)->SetTransform(_trans * (*itlink)->GetTransform());
            }
            Vector com = _pchain->GetCenterOfMass();
            RAVELOG_VERBOSE("%s: COM = (%f,%f,%f)\n", _pchain->GetName().c_str(), com.x, com.y, com.z);
            return true;
        }
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( _processingtag.size() > 0 ) {
            string_stream_.clear();
            string_stream_ << ch;
        }
        else {
            InterfaceXMLReader::characters(ch);
        }
    }

    const std::shared_ptr< std::vector<dReal> > GetJointValues() {
        return _vjointvalues;
    }
protected:
    KinBodyPtr _pchain;
    Transform _trans;

    // default mass type passed to every LinkXMLReader
    int rootoffset, rootjoffset, rootjpoffset, roottransoffset;                         //!< the initial number of links when KinBody is created (so that global translations and rotations only affect the new links)
    MassType _masstype;                     //!< if true, mass is craeted so that it mimics the geometry
    float _fMassValue;                       //!< density or total mass
    Vector _vMassExtents;

    std::vector<Transform> _vTransforms;             //!< original transforms of the bodies for offsetfrom

    string _strModelsDir, _bodyname;
    string _prefix;         //!< add this prefix to all names of links and joints
    KinBody::LinkPtr _plink;
    KinBody::JointPtr _pjoint;

    RaveVector<float> _diffusecol, _ambientcol;
    float _transparency;
    bool _bSkipGeometry;
    Vector _vScaleGeometry;
    bool _bMakeJoinedLinksAdjacent;
    std::shared_ptr< std::vector<dReal> > _vjointvalues;

    string _processingtag;         /// if not empty, currently processing
    bool _bOverwriteDiffuse, _bOverwriteAmbient, _bOverwriteTransparency;
};

class ControllerXMLReader : public InterfaceXMLReader
{
public:
    ControllerXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pinterface, const AttributesList &atts,RobotBasePtr probot=RobotBasePtr()) : InterfaceXMLReader(penv,pinterface,PT_Controller,RaveGetInterfaceName(PT_Controller),atts) {
        _probot = probot;
        control_transformation = 0;
        FOREACHC(itatt, atts) {
            if( itatt->first == "robot" ) {
                _robotname = itatt->second;
            }
            else if( itatt->first == "joints" ) {
                stringstream ss(itatt->second);
                _vjoints.reset(new std::vector<std::string>((istream_iterator<std::string>(ss)), istream_iterator<std::string>()));
            }
            else if( itatt->first == "transform" ) {
                stringstream ss(itatt->second);
                ss >> control_transformation;
            }
        }
    }
    virtual ~ControllerXMLReader() {
    }

    virtual bool endElement(const std::string& xmlname)
    {
        if( InterfaceXMLReader::endElement(xmlname) ) {
            if( !_probot ) {
                if( _robotname.size() > 0 ) {
                    KinBodyPtr pbody = _penv->GetKinBody(_robotname.c_str());
                    if( pbody->IsRobot() )
                        _probot = RaveInterfaceCast<RobotBase>(pbody);
                }
            }

            if( !!_probot ) {
                std::vector<int> dofindices;
                if( !_vjoints ) {
                    for(int i = 0; i < _probot->GetDOF(); ++i) {
                        dofindices.push_back(i);
                    }
                }
                else {
                    FOREACH(it,*_vjoints) {
                        KinBody::JointPtr pjoint = _probot->GetJoint(*it);
                        if( !!pjoint ) {
                            for(int i = 0; i < pjoint->GetDOF(); ++i) {
                                dofindices.push_back(pjoint->GetDOFIndex()+i);
                            }
                        }
                        else {
                            RAVELOG_WARN(str(boost::format("could not find joint %s\n")%*it));
                        }
                    }
                }
                _CheckInterface();
                _probot->SetController(RaveInterfaceCast<ControllerBase>(_pinterface),dofindices,control_transformation);
            }
            else {
                RAVELOG_WARN("controller is unused\n");
            }
            return true;
        }
        return false;
    }

    string _robotname;
    std::shared_ptr< vector<string> > _vjoints;
    int control_transformation;
    RobotBasePtr _probot;
};

class ManipulatorXMLReader : public StreamXMLReader
{
public:
    ManipulatorXMLReader(RobotBasePtr probot, const AttributesList &atts) {
        _vScaleGeometry = Vector(1,1,1);
        FOREACHC(itatt,atts) {
            if( itatt->first == "name" ) {
                _manipinfo.name_ = itatt->second;
            }
        }
        _probot = probot;
    }

    virtual const RobotBase::ManipulatorInfo& GetInfo() {
        return _manipinfo;
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList &atts)
    {
        switch( StreamXMLReader::startElement(xmlname,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        if( _processingtag.size() > 0 ) {
            return PE_Ignore;
        }
        if (( xmlname == "effector") ||( xmlname == "gripperjoints") ||( xmlname == "joints") ||( xmlname == "armjoints") ||( xmlname == "base") ||( xmlname == "iksolver") ||( xmlname == "closingdir") ||( xmlname == "palmdirection") ||( xmlname=="direction") ||( xmlname == "closingdirection") ||( xmlname == "translation") ||( xmlname == "quat") ||( xmlname == "rotationaxis") ||( xmlname == "rotationmat") || xmlname == "chuckingdirection") {
            _processingtag = xmlname;
            return PE_Support;
        }
        return PE_Pass;
    }

    virtual bool endElement(const std::string& xmlname)
    {
        if( StreamXMLReader::endElement(xmlname) ) {
            return true;
        }

        if( xmlname == "manipulator" ) {
            _probot->manipulators_vector_.push_back(RobotBase::ManipulatorPtr(new RobotBase::Manipulator(_probot,_manipinfo)));
            return true;
        }
        else if( xmlname == "effector" ) {
            string_stream_ >> _manipinfo.effector_link_name_;
            if( !!_probot && !_probot->GetLink(_manipinfo.effector_link_name_) ) {
                RAVELOG_WARN(str(boost::format("Failed to find manipulator end effector %s")%_manipinfo.effector_link_name_));
                GetXMLErrorCount()++;
            }
        }
        else if( xmlname == "base" ) {
            string_stream_ >> _manipinfo.base_link_name_;
            if( !!_probot && !_probot->GetLink(_manipinfo.base_link_name_) ) {
                RAVELOG_WARN(str(boost::format("Failed to find manipulator base %s")%_manipinfo.base_link_name_));
                GetXMLErrorCount()++;
            }
        }
        else if((xmlname == "joints")||(xmlname == "gripperjoints")) {
            _manipinfo.gripper_joint_names_vector_ = vector<string>((istream_iterator<string>(string_stream_)), istream_iterator<string>());
        }
        else if( xmlname == "armjoints" ) {
            RAVELOG_WARN("<armjoints> for <manipulator> tag is not used anymore\n");
        }
        else if((xmlname == "direction")||(xmlname == "palmdirection")) {
            if( xmlname == "palmdirection" ) {
                RAVELOG_WARN("<palmdirection> tag in Manipulator changed to <direction>\n");
            }
            string_stream_ >> _manipinfo.direction_.x >> _manipinfo.direction_.y >> _manipinfo.direction_.z;
            dReal flen = _manipinfo.direction_.lengthsqr3();
            if( flen == 0 ) {
                RAVELOG_WARN("palm direction is 0, setting to default value\n");
                _manipinfo.direction_ = Vector(0,0,1);
            }
            else {
                _manipinfo.direction_ /= RaveSqrt(flen);
            }
        }
        else if( xmlname == "iksolver" ) {
            string iklibraryname = string_stream_.str();
            IkSolverBasePtr piksolver;
            if( RaveHasInterface(PT_IkSolver,iklibraryname) ) {
                piksolver = RaveCreateIkSolver(_probot->GetEnv(), iklibraryname);
            }
            if( !piksolver ) {
                // try adding the current directory
                if( RaveHasInterface(PT_IkSolver,GetParseDirectory()+s_filesep+iklibraryname)) {
                    string fullname = GetParseDirectory(); fullname.push_back(s_filesep); fullname += iklibraryname;
                    piksolver = RaveCreateIkSolver(_probot->GetEnv(), fullname);
                }

                if( !piksolver ) {
                    // try loading the shared object
                    ModuleBasePtr pIKFastLoader;
                    {
                        list<ModuleBasePtr> listModules;
                        _probot->GetEnv()->GetModules(listModules);
                        FOREACHC(itprob, listModules) {
                            if( _stricmp((*itprob)->GetXMLId().c_str(),"ikfast") == 0 ) {
                                pIKFastLoader = *itprob;
                                break;
                            }
                        }
                    }

                    if( !pIKFastLoader ) {
                        pIKFastLoader = RaveCreateModule(_probot->GetEnv(), "ikfast");
                        if( !!pIKFastLoader ) {
                            _probot->GetEnv()->AddModule(pIKFastLoader,"");
                        }
                    }

                    if( !!pIKFastLoader ) {
                        string ikonly;
                        string_stream_ >> ikonly;
                        stringstream scmd(string("AddIkLibrary ") + ikonly + string(" ") + ikonly);
                        stringstream sout;
                        if( !ifstream(ikonly.c_str()) || !pIKFastLoader->SendCommand(sout, scmd)) {
                            string fullname = GetParseDirectory(); fullname.push_back(s_filesep); fullname += ikonly;
                            scmd.str(string("AddIkLibrary ") + fullname + string(" ") + fullname);
                            if( !ifstream(fullname.c_str()) || !pIKFastLoader->SendCommand(sout, scmd)) {
                            }
                            else {
                                // need to use the original iklibrary string due to parameters being passed in
                                string fullname = "ikfast ";
                                fullname += GetParseDirectory(); fullname.push_back(s_filesep); fullname += iklibraryname;
                                piksolver = RaveCreateIkSolver(_probot->GetEnv(), fullname);
                            }
                        }
                        else {
                            string fullname = "ikfast "; fullname += iklibraryname;
                            piksolver = RaveCreateIkSolver(_probot->GetEnv(), fullname);
                        }
                    }
                    else {
                        RAVELOG_WARN("Failed to load IKFast module\n");
                    }
                }
            }

            if( !piksolver ) {
                RAVELOG_WARN(str(boost::format("failed to create iksolver %s")%iklibraryname));
            }
            else {
                _manipinfo.ik_solver_xml_id_ = piksolver->GetXMLId();
            }
            if( !!piksolver ) {
                _manipinfo.ik_solver_xml_id_ = piksolver->GetXMLId();
            }
        }
        else if( xmlname == "closingdirection" || xmlname == "closingdir" || xmlname == "chuckingdirection" ) {
            _manipinfo.chucking_direction_vector_ = vector<dReal>((istream_iterator<dReal>(string_stream_)), istream_iterator<dReal>());
            FOREACH(it, _manipinfo.chucking_direction_vector_) {
                if( *it > 0 ) {
                    *it = 1;
                }
                else if( *it < 0 ) {
                    *it = -1;
                }
            }
        }
        else if( xmlname == "translation" ) {
            Vector v;
            string_stream_ >> v.x >> v.y >> v.z;
            _manipinfo.local_tool_transform_.trans += v*_vScaleGeometry;
        }
        else if( xmlname == "quat" ) {
            Transform tnew;
            string_stream_ >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
            tnew.rot.normalize4();
            _manipinfo.local_tool_transform_.rot = (tnew*_manipinfo.local_tool_transform_).rot;
        }
        else if( xmlname == "rotationaxis" ) {
            Vector vaxis; dReal fangle=0;
            string_stream_ >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
            Transform tnew; tnew.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
            _manipinfo.local_tool_transform_.rot = (tnew*_manipinfo.local_tool_transform_).rot;
        }
        else if( xmlname == "rotationmat" ) {
            TransformMatrix tnew;
            string_stream_ >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
            _manipinfo.local_tool_transform_.rot = (Transform(tnew)*_manipinfo.local_tool_transform_).rot;
        }

        if( xmlname !=_processingtag ) {
            RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
        }
        _processingtag = "";
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( _processingtag.size() > 0 ) {
            string_stream_.clear();
            string_stream_ << ch;
        }
        else {
            StreamXMLReader::characters(ch);
        }
    }

protected:
    RobotBase::ManipulatorInfo _manipinfo;
    RobotBasePtr _probot;
    string _processingtag;
    Vector _vScaleGeometry;
};

/// sensors specifically attached to a robot
class AttachedSensorXMLReader : public StreamXMLReader
{
public:
    AttachedSensorXMLReader(RobotBase::AttachedSensorPtr& psensor, RobotBasePtr probot, const AttributesList &atts) : _psensor(psensor) {
        string name;
        _vScaleGeometry = Vector(1,1,1);
        FOREACHC(itatt, atts) {
            if( itatt->first == "name" ) {
                name = itatt->second;
            }
        }

        if( !_psensor ) {
            // check for current sensors
            FOREACH(itsensor,probot->GetAttachedSensors()) {
                if(( name.size() > 0) &&( (*itsensor)->GetName() == name) ) {
                    _psensor = *itsensor;
                    break;
                }
            }

            if( !_psensor ) {
                _psensor.reset(new RobotBase::AttachedSensor(probot));
                probot->attached_sensors_vector_.push_back(_psensor);
            }
        }

        _psensor->info_.name_ = name;
        _probot = _psensor->GetRobot();
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList &atts)
    {
        switch( StreamXMLReader::startElement(xmlname,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        if( _processingtag.size() > 0 ) {
            return PE_Ignore;
        }
        if( xmlname == "sensor" ) {
            // create the sensor
            _psensorinterface.reset();
            cur_reader_ = CreateInterfaceReader(_probot->GetEnv(),PT_Sensor,_psensorinterface, xmlname, atts);
            return PE_Support;
        }

        if (( xmlname == "link") ||( xmlname == "translation") ||( xmlname == "quat") ||( xmlname == "rotationaxis") ||( xmlname == "rotationmat") ) {
            _processingtag = xmlname;
            return PE_Support;
        }
        return PE_Pass;
    }
    virtual bool endElement(const std::string& xmlname)
    {
        if( !!cur_reader_ ) {
            if( cur_reader_->endElement(xmlname) ) {
                cur_reader_.reset();
                _psensor->sensor_ = RaveInterfaceCast<SensorBase>(_psensorinterface);
            }
            return false;
        }
        else if( xmlname == "attachedsensor" ) {
            if( !_psensor->sensor_ ) {
                RAVELOG_VERBOSE("Attached robot sensor %s points to no real sensor!\n",_psensor->GetName().c_str());
            }
            else {
                _psensor->pdata = _psensor->sensor_->CreateSensorData();
                if( _psensor->pattachedlink.expired() ) {
                    RAVELOG_INFOA("no attached link, setting to base of robot\n");
                    if( _probot->GetLinks().size() == 0 ) {
                        RAVELOG_INFOA("robot has no links!\n");
                        _psensor->pattachedlink.reset();
                    }
                    else {
                        _psensor->pattachedlink = _probot->GetLinks().at(0);
                    }
                }
            }
            return true;
        }
        else if( xmlname == "link" ) {
            string linkname;
            string_stream_ >> linkname;
            _psensor->pattachedlink = _probot->GetLink(linkname);

            if( _psensor->pattachedlink.expired() ) {
                RAVELOG_WARN("Failed to find attached sensor link %s\n", linkname.c_str());
                GetXMLErrorCount()++;
            }
        }
        else if( xmlname == "translation" ) {
            Vector v;
            string_stream_ >> v.x >> v.y >> v.z;
            _psensor->info_.relative_transform_.trans += v*_vScaleGeometry;
        }
        else if( xmlname == "quat" ) {
            Transform tnew;
            string_stream_ >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
            tnew.rot.normalize4();
            _psensor->info_.relative_transform_.rot = (tnew*_psensor->info_.relative_transform_).rot;
        }
        else if( xmlname == "rotationaxis" ) {
            Vector vaxis; dReal fangle=0;
            string_stream_ >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
            Transform tnew; tnew.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
            _psensor->info_.relative_transform_.rot = (tnew*_psensor->info_.relative_transform_).rot;
        }
        else if( xmlname == "rotationmat" ) {
            TransformMatrix tnew;
            string_stream_ >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
            _psensor->info_.relative_transform_.rot = (Transform(tnew)*_psensor->info_.relative_transform_).rot;
        }

        if( xmlname !=_processingtag )
            RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
        _processingtag = "";
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( _processingtag.size() > 0 ) {
            string_stream_.clear();
            string_stream_ << ch;
        }
        else {
            StreamXMLReader::characters(ch);
        }
    }

protected:
    RobotBasePtr _probot;
    RobotBase::AttachedSensorPtr& _psensor;
    InterfaceBasePtr _psensorinterface;
    string _processingtag;
    string args;         //!< arguments to pass to sensor when initializing
    Vector _vScaleGeometry;
};

class RobotXMLReader : public InterfaceXMLReader
{
public:
    RobotXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& probot, const AttributesList &atts, int roottransoffset) : InterfaceXMLReader(penv,probot,PT_Robot,"robot",atts), roottransoffset(roottransoffset) {
        _bSkipGeometry = false;
        _vScaleGeometry = Vector(1,1,1);
        rootoffset = rootjoffset = rootjpoffset = -1;
        FOREACHC(itatt, atts) {
            if( itatt->first == "name" ) {
                _robotname = itatt->second;
            }
            else if( itatt->first == "prefix" ) {
                _prefix = itatt->second;
            }
            else if( itatt->first == "skipgeometry" ) {
                _bSkipGeometry = _stricmp(itatt->second.c_str(), "true") == 0 || itatt->second=="1";
            }
            else if( itatt->first == "scalegeometry" ) {
                stringstream ss(itatt->second);
                Vector v(1,1,1);
                ss >> v.x >> v.y >> v.z;
                if( !ss ) {
                    v.z = v.y = v.x;
                }
                _vScaleGeometry *= v;
            }
        }
        _CheckInterface();
    }

    virtual void _CheckInterface()
    {
        InterfaceXMLReader::_CheckInterface();
        _probot = RaveInterfaceCast<RobotBase>(_pinterface);
        if( !!_probot ) {
            if( rootoffset < 0 ) {
                rootoffset = (int)_probot->GetLinks().size();
                rootjoffset = (int)_probot->GetJoints().size();
                rootjpoffset = (int)_probot->GetPassiveJoints().size();
                FOREACH(itmanip,_probot->GetManipulators()) {
                    _setInitialManipulators.insert(*itmanip);
                }
                FOREACH(itsensor,_probot->GetAttachedSensors()) {
                    _setInitialSensors.insert(*itsensor);
                }
            }
        }
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList &atts)
    {
        if( _processingtag.size() > 0 ) {
            switch( StreamXMLReader::startElement(xmlname,atts) ) {
            case PE_Pass: break;
            case PE_Support: return PE_Support;
            case PE_Ignore: return PE_Ignore;
            }
            return PE_Ignore;
        }

        switch( InterfaceXMLReader::startElement(xmlname,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        if( xmlname == "robot" ) {
            AttributesList newatts = atts;
            newatts.emplace_back("skipgeometry", _bSkipGeometry ? "1" : "0");
            newatts.emplace_back("scalegeometry", str(boost::format("%f %f %f")%_vScaleGeometry.x%_vScaleGeometry.y%_vScaleGeometry.z));
            cur_reader_ = CreateInterfaceReader(_penv, PT_Robot, _pinterface, xmlname, newatts);
            return PE_Support;
        }

        _CheckInterface();
        if( xmlname == "kinbody" ) {
            AttributesList newatts = atts;
            newatts.emplace_back("skipgeometry", _bSkipGeometry ? "1" : "0");
            newatts.emplace_back("scalegeometry", str(boost::format("%f %f %f")%_vScaleGeometry.x%_vScaleGeometry.y%_vScaleGeometry.z));
            cur_reader_ = CreateInterfaceReader(_penv,PT_KinBody,_pinterface, xmlname, newatts);
        }
        else if( xmlname == "manipulator" ) {
            cur_reader_.reset(new ManipulatorXMLReader(_probot, atts));
        }
        else if( xmlname == "attachedsensor" ) {
            _psensor.reset();
            cur_reader_.reset(new AttachedSensorXMLReader(_psensor, _probot, atts));
        }
        else if( xmlname == "controller" ) {
            _pcontroller.reset();
            cur_reader_.reset(new ControllerXMLReader(_probot->GetEnv(),_pcontroller,atts,_probot));
        }
        else if((xmlname == "translation")||(xmlname == "rotationmat")||(xmlname == "rotationaxis")||(xmlname == "quat")||(xmlname == "jointvalues")) {
            _processingtag = xmlname;
        }
        else {
            return PE_Pass;
        }
        return PE_Support;
    }

    virtual bool endElement(const std::string& xmlname)
    {
        if( !!cur_reader_ ) {
            if( cur_reader_->endElement(xmlname) ) {
                KinBodyXMLReaderPtr kinbodyreader = std::dynamic_pointer_cast<KinBodyXMLReader>(cur_reader_);
                if( !!kinbodyreader ) {
                    if( !_vjointvalues ) {
                        _vjointvalues = kinbodyreader->GetJointValues();
                    }
                }
                cur_reader_.reset();
            }
            _probot = RaveInterfaceCast<RobotBase>(_pinterface);         // might be updated by readers
            return false;
        }
        else if( _processingtag.size() > 0 ) {
            if( xmlname == "translation" ) {
                Vector v;
                string_stream_ >> v.x >> v.y >> v.z;
                _trans.trans += v*_vScaleGeometry;
            }
            else if( xmlname == "rotationaxis" ) {
                Vector vaxis; dReal fangle=0;
                string_stream_ >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
                Transform tnew; tnew.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
                _trans.rot = (tnew*_trans).rot;
            }
            else if( xmlname == "quat" ) {
                Transform tnew;
                string_stream_ >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
                tnew.rot.normalize4();
                _trans.rot = (tnew*_trans).rot;
            }
            else if( xmlname == "rotationmat" ) {
                TransformMatrix tnew;
                string_stream_ >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
                _trans.rot = (Transform(tnew)*_trans).rot;
            }
            else if( xmlname == "controller" ) {
            }
            else if( xmlname == "jointvalues" ) {
                _vjointvalues.reset(new std::vector<dReal>((istream_iterator<dReal>(string_stream_)), istream_iterator<dReal>()));
            }

            if( xmlname !=_processingtag ) {
                RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
            }
            _processingtag = "";
        }
        else if( InterfaceXMLReader::endElement(xmlname) ) {
            if( _robotname.size() > 0 ) {
                _probot->SetName(_robotname);
            }
            if( file_name_.size() > 0 ) {
                SetFilename(file_name_);
            }

            // put the sensors and manipulators in front of what was declared. this is necessary so that user-based manipulator definitions come before the pre-defined ones.
            if( _setInitialSensors.size() > 0 ) {
                std::vector<RobotBase::AttachedSensorPtr> vtemp; vtemp.reserve(_probot->GetAttachedSensors().size());
                FOREACH(itsensor,_probot->GetAttachedSensors()) {
                    if( _setInitialSensors.find(*itsensor) == _setInitialSensors.end() ) {
                        vtemp.insert(vtemp.begin(),*itsensor);
                    }
                    else {
                        vtemp.push_back(*itsensor);
                    }
                }
                _probot->attached_sensors_vector_.swap(vtemp);
            }
            if( _setInitialManipulators.size() > 0 ) {
                std::vector<RobotBase::ManipulatorPtr> vtemp; vtemp.reserve(_probot->GetManipulators().size());
                FOREACH(itmanip,_probot->GetManipulators()) {
                    if( _setInitialManipulators.find(*itmanip) == _setInitialManipulators.end() ) {
                        vtemp.insert(vtemp.begin(),*itmanip);
                    }
                    else {
                        vtemp.push_back(*itmanip);
                    }
                }
                _probot->manipulators_vector_.swap(vtemp);
            }

            // add prefix
            if( _prefix.size() > 0 ) {
                BOOST_ASSERT(rootoffset >= 0 && rootoffset<=(int)_probot->links_vector_.size());
                vector<KinBody::LinkPtr>::iterator itlink = _probot->links_vector_.begin()+rootoffset;
                while(itlink != _probot->links_vector_.end()) {
                    (*itlink)->info_.name_ = _prefix + (*itlink)->info_.name_;
                    ++itlink;
                }
                std::vector< std::pair<std::string, std::string> > jointnamepairs;
                jointnamepairs.reserve(_probot->joints_vector_.size());
                BOOST_ASSERT(rootjoffset >= 0 && rootjoffset<=(int)_probot->joints_vector_.size());
                vector<KinBody::JointPtr>::iterator itjoint = _probot->joints_vector_.begin()+rootjoffset;
                list<KinBody::JointPtr> listjoints;
                while(itjoint != _probot->joints_vector_.end()) {
                    jointnamepairs.emplace_back((*itjoint)->info_.name_,  _prefix +(*itjoint)->info_.name_);
                    (*itjoint)->info_.name_ = _prefix +(*itjoint)->info_.name_;
                    listjoints.push_back(*itjoint);
                    ++itjoint;
                }
                BOOST_ASSERT(rootjpoffset >= 0 && rootjpoffset<=(int)_probot->passive_joints_vector_.size());
                itjoint = _probot->passive_joints_vector_.begin()+rootjpoffset;
                while(itjoint != _probot->passive_joints_vector_.end()) {
                    jointnamepairs.emplace_back((*itjoint)->info_.name_,  _prefix +(*itjoint)->info_.name_);
                    (*itjoint)->info_.name_ = _prefix +(*itjoint)->info_.name_;
                    listjoints.push_back(*itjoint);
                    ++itjoint;
                }
                // repeat again for the mimic equations, if any exist
                FOREACH(itjoint2, listjoints) {
                    for(int idof = 0; idof < (*itjoint2)->GetDOF(); ++idof) {
                        if( (*itjoint2)->IsMimic(idof) ) {
                            for(int ieq = 0; ieq < 3; ++ieq) {
                                string neweq;
                                utils::SearchAndReplace(neweq,(*itjoint2)->mimic_array_[idof]->_equations[ieq],jointnamepairs);
                                (*itjoint2)->mimic_array_[idof]->_equations[ieq] = neweq;
                            }
                        }
                    }
                }
                FOREACH(itsensor, _probot->GetAttachedSensors()) {
                    if( _setInitialSensors.find(*itsensor) == _setInitialSensors.end() ) {
                        (*itsensor)->info_.name_ = _prefix + (*itsensor)->info_.name_;
                    }
                }
                FOREACH(itmanip,_probot->GetManipulators()) {
                    if( _setInitialManipulators.find(*itmanip) == _setInitialManipulators.end()) {
                        (*itmanip)->info_.name_ = _prefix + (*itmanip)->info_.name_;
                        (*itmanip)->info_.base_link_name_ = _prefix + (*itmanip)->info_.base_link_name_;
                        (*itmanip)->info_.effector_link_name_ = _prefix + (*itmanip)->info_.effector_link_name_;
                        FOREACH(itgrippername,(*itmanip)->info_.gripper_joint_names_vector_) {
                            *itgrippername = _prefix + *itgrippername;
                        }
                    }
                }
            }

            // transform all "new" bodies with trans
            BOOST_ASSERT(roottransoffset>=0&&roottransoffset<=(int)_probot->links_vector_.size());
            vector<KinBody::LinkPtr>::iterator itlink = _probot->links_vector_.begin()+roottransoffset;
            while(itlink != _probot->links_vector_.end()) {
                (*itlink)->SetTransform(_trans * (*itlink)->GetTransform());
                ++itlink;
            }

            // forces robot to reupdate its internal objects
            _probot->SetTransform(_probot->GetTransform());
            return true;
        }
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( _processingtag.size() > 0 ) {
            string_stream_.clear();
            string_stream_ << ch;
        }
        else {
            InterfaceXMLReader::characters(ch);
        }
    }

    const std::shared_ptr< std::vector<dReal> > GetJointValues() {
        return _vjointvalues;
    }

protected:
    RobotBasePtr _probot;
    InterfaceBasePtr _pcontroller;         //!< controller to set the robot at
    string _robotname;
    string _prefix;
    string _processingtag;

    std::shared_ptr<std::vector<dReal> >  _vjointvalues;
    RobotBase::AttachedSensorPtr _psensor;

    Transform _trans;
    bool _bSkipGeometry;
    Vector _vScaleGeometry;
    int rootoffset, roottransoffset;                         //!< the initial number of links when Robot is created (so that global translations and rotations only affect the new links)
    int rootjoffset, rootjpoffset;         //!< the initial number of joints when Robot is created
    std::set<RobotBase::ManipulatorPtr> _setInitialManipulators;
    std::set<RobotBase::AttachedSensorPtr> _setInitialSensors;
};

template <InterfaceType type> class DummyInterfaceXMLReader : public InterfaceXMLReader
{
public:
    DummyInterfaceXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pinterface, const string &xmltag, const AttributesList &atts) : InterfaceXMLReader(penv,pinterface,type,xmltag,atts) {
    }
    virtual ~DummyInterfaceXMLReader() {
    }
};

class ModuleXMLReader : public InterfaceXMLReader
{
public:
    ModuleXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pinterface, const AttributesList &atts) : InterfaceXMLReader(penv,pinterface,PT_Module,RaveGetInterfaceName(PT_Module),atts) {
        FOREACHC(itatt,atts) {
            if( itatt->first == "args" ) {
                _args = itatt->second;
            }
        }

        _CheckInterface();
    }

    const string& GetArgs() const {
        return _args;
    }

protected:
    string _args;
};

typedef std::shared_ptr<ModuleXMLReader> ModuleXMLReaderPtr;

class SensorXMLReader : public InterfaceXMLReader
{
public:
    SensorXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pinterface, const AttributesList &atts) : InterfaceXMLReader(penv,pinterface,PT_Sensor,RaveGetInterfaceName(PT_Sensor),atts) {
        string args;
        _vScaleGeometry = Vector(1,1,1);
        FOREACHC(itatt,atts) {
            if( itatt->first == "name" ) {
                _strname = itatt->second;
            }
            else if( itatt->first == "args" ) {
                RAVELOG_WARN("sensor args has been deprecated.\n");
            }
        }
    }

    virtual void _CheckInterface()
    {
        InterfaceXMLReader::_CheckInterface();
        _psensor = RaveInterfaceCast<SensorBase>(_pinterface);
        if( !!_psensor ) {
            _psensor->SetName(_strname);
        }
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList &atts)
    {
        if( _processingtag.size() > 0 ) {
            switch( StreamXMLReader::startElement(xmlname,atts) ) {
            case PE_Pass: break;
            case PE_Support: return PE_Support;
            case PE_Ignore: return PE_Ignore;
            }
        }

        switch( InterfaceXMLReader::startElement(xmlname,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        _CheckInterface();
        if( !_psensor ||( _processingtag.size() > 0) ) {
            return PE_Ignore;
        }
        if (( xmlname == "translation") ||( xmlname == "quat") ||( xmlname == "rotationaxis") ||( xmlname == "rotationmat") ) {
            _processingtag = xmlname;
            return PE_Support;
        }
        return PE_Pass;
    }
    virtual bool endElement(const std::string& xmlname)
    {
        if( !!cur_reader_ ) {
            if( cur_reader_->endElement(xmlname) ) {
                cur_reader_.reset();
            }
            return false;
        }
        else if( _processingtag.size() > 0 ) {
            if( xmlname == "translation" ) {
                Vector v;
                string_stream_ >> v.x >> v.y >> v.z;
                _tsensor.trans += v*_vScaleGeometry;
            }
            else if( xmlname == "quat" ) {
                Transform tnew;
                string_stream_ >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
                tnew.rot.normalize4();
                _tsensor.rot = (tnew*_tsensor).rot;
            }
            else if( xmlname == "rotationaxis" ) {
                Vector vaxis; dReal fangle=0;
                string_stream_ >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
                Transform tnew; tnew.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
                _tsensor.rot = (tnew*_tsensor).rot;
            }
            else if( xmlname == "rotationmat" ) {
                TransformMatrix tnew;
                string_stream_ >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
                _tsensor.rot = (Transform(tnew)*_tsensor).rot;
            }

            if( xmlname !=_processingtag ) {
                RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
            }
            _processingtag.resize(0);
        }
        else if( InterfaceXMLReader::endElement(xmlname) ) {
            if( !!_psensor ) {
                _psensor->SetTransform(_tsensor);
            }
            return true;
        }
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( _processingtag.size() > 0 ) {
            string_stream_.clear();
            string_stream_ << ch;
        }
        else {
            InterfaceXMLReader::characters(ch);
        }
    }

    const string& GetArgs() const {
        return _args;
    }

protected:
    SensorBasePtr _psensor;
    Transform _tsensor;
    string _args, _processingtag, _strname;
    Vector _vScaleGeometry;
};

class EnvironmentXMLReader : public StreamXMLReader
{
public:
    EnvironmentXMLReader(EnvironmentBasePtr penv, const AttributesList &atts, bool bInEnvironment) : _penv(penv), _bInEnvironment(bInEnvironment)
    {
        if( !_penv ) {
            throw OpenRAVEException(_tr("need valid environment"),ORE_InvalidArguments);
        }
        FOREACHC(itatt,atts) {
            if( itatt->first == "file" ) {
                AttributesList listnewatts;
                FOREACHC(itatt2,atts) {
                    if( itatt2->first != "file" ) {
                        listnewatts.push_back(*itatt2);
                    }
                }

                string filedata = RaveFindLocalFile(itatt->second,GetParseDirectory());
                if( filedata.size() == 0 ) {
                    continue;
                }
                _penv->Load(filedata);
            }
        }
        _tCamera.trans = Vector(0, 1.5f, 0.8f);
        _tCamera.rot = quatFromAxisAngle(Vector(1, 0, 0), (dReal)-0.5);
        _fCameraFocalDistance = 0;
        vBkgndColor = Vector(1,1,1);
        bTransSpecified = false;
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList &atts)
    {
        switch( StreamXMLReader::startElement(xmlname,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }
        if( _processingtag.size() > 0 ) {
            return PE_Ignore;
        }
        // check for any plugins
        FOREACHC(itname,RaveGetInterfaceNamesMap()) {
            if( xmlname == itname->second ) {
                if( !!_pinterface ) {
                    throw OpenRAVEException(_tr("interface should not be initialized"));
                }
                cur_reader_ = CreateInterfaceReader(_penv,itname->first,_pinterface,"",atts);
                if( !cur_reader_ ) {
                    RAVELOG_WARN("failed to create interface %s in <environment>\n", itname->second.c_str());
                    cur_reader_.reset(new DummyXMLReader(xmlname,"environment"));
                }
                return PE_Support;
            }
        }

        if( xmlname == "environment" ) {
            cur_reader_.reset(new EnvironmentXMLReader(_penv,atts,true));
            return PE_Support;
        }

        static std::array<string, 9> tags = { { "bkgndcolor", "camrotaxis", "camrotationaxis", "camrotmat", "camtrans", "camfocal", "bkgndcolor", "plugin", "unit"}};
        if( find(tags.begin(),tags.end(),xmlname) != tags.end() ) {
            _processingtag = xmlname;
            return PE_Support;
        }
        return PE_Pass;
    }

    virtual bool endElement(const std::string& xmlname)
    {
        if( !!cur_reader_ ) {
            if( cur_reader_->endElement(xmlname) ) {
                if( !_bInEnvironment ) {
                    InterfaceXMLReaderPtr pinterfacereader = std::dynamic_pointer_cast<InterfaceXMLReader>(cur_reader_);
                    if( !!pinterfacereader ) {
                        pinterfacereader->SetFilename(file_name_);
                    }
                }

                if( !!std::dynamic_pointer_cast<RobotXMLReader>(cur_reader_) ) {
                    std::shared_ptr<RobotXMLReader> robotreader = std::dynamic_pointer_cast<RobotXMLReader>(cur_reader_);
                    BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_Robot);
                    RobotBasePtr probot = RaveInterfaceCast<RobotBase>(_pinterface);
                    _penv->Add(probot);
                    if( !!robotreader->GetJointValues() ) {
                        if( (int)robotreader->GetJointValues()->size() != probot->GetDOF() ) {
                            RAVELOG_WARN(str(boost::format("<jointvalues> wrong number of values %d!=%d, robot=%s")%robotreader->GetJointValues()->size()%probot->GetDOF()%probot->GetName()));
                        }
                        else {
                            probot->SetDOFValues(*robotreader->GetJointValues());
                        }
                    }
                }
                else if( !!std::dynamic_pointer_cast<KinBodyXMLReader>(cur_reader_) ) {
                    KinBodyXMLReaderPtr kinbodyreader = std::dynamic_pointer_cast<KinBodyXMLReader>(cur_reader_);
                    BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_KinBody);
                    KinBodyPtr pbody = RaveInterfaceCast<KinBody>(_pinterface);
                    _penv->Add(pbody);
                    if( !!kinbodyreader->GetJointValues() ) {
                        if( (int)kinbodyreader->GetJointValues()->size() != pbody->GetDOF() ) {
                            RAVELOG_WARN(str(boost::format("<jointvalues> wrong number of values %d!=%d, body=%s")%kinbodyreader->GetJointValues()->size()%pbody->GetDOF()%pbody->GetName()));
                        }
                        else {
                            pbody->SetDOFValues(*kinbodyreader->GetJointValues());
                        }
                    }
                }
                else if( !!std::dynamic_pointer_cast<SensorXMLReader>(cur_reader_) ) {
                    BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_Sensor);
                    _penv->Add(RaveInterfaceCast<SensorBase>(_pinterface));
                }
                else if( !!std::dynamic_pointer_cast< DummyInterfaceXMLReader<PT_PhysicsEngine> >(cur_reader_) ) {
                    BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_PhysicsEngine);
                    _penv->SetPhysicsEngine(RaveInterfaceCast<PhysicsEngineBase>(_pinterface));
                }
                else if( !!std::dynamic_pointer_cast< DummyInterfaceXMLReader<PT_CollisionChecker> >(cur_reader_) ) {
                    BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_CollisionChecker);
                    _penv->SetCollisionChecker(RaveInterfaceCast<CollisionCheckerBase>(_pinterface));
                }
                else if( !!std::dynamic_pointer_cast<ModuleXMLReader>(cur_reader_) ) {
                    ModuleXMLReaderPtr modulereader = std::dynamic_pointer_cast<ModuleXMLReader>(cur_reader_);
                    ModuleBasePtr module = RaveInterfaceCast<ModuleBase>(_pinterface);
                    if( !!module ) {
                        int ret = _penv->AddModule(module,modulereader->GetArgs());
                        if( ret ) {
                            RAVELOG_WARN(str(boost::format("module %s returned %d\n")%module->GetXMLId()%ret));
                        }
                    }
                }
                else if( !!_pinterface ) {
                    RAVELOG_DEBUG("owning interface %s, type: %s\n",_pinterface->GetXMLId().c_str(),RaveGetInterfaceName(_pinterface->GetInterfaceType()).c_str());
                    _penv->OwnInterface(_pinterface);
                }
                _pinterface.reset();
                cur_reader_.reset();
            }
            return false;
        }
        if( xmlname == "environment" ) {
            // only move the camera if trans is specified
            if( !!_penv->GetViewer() ) {
                if( bTransSpecified ) {
                    _penv->GetViewer()->SetCamera(_tCamera, _fCameraFocalDistance);
                }
                _penv->GetViewer()->SetBkgndColor(vBkgndColor);
            }
            return true;
        }

        if( xmlname == "bkgndcolor" ) {
            string_stream_ >> vBkgndColor.x >> vBkgndColor.y >> vBkgndColor.z;
        }
        else if( xmlname == "camrotaxis" ) {
            RAVELOG_INFO("<camrotaxis> is deprecated, use <camrotationaxis> by rotating 180 around Z axis\n");
            Vector vaxis; dReal fangle=0;
            string_stream_ >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
            _tCamera.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
            // have to rotate due to old bug
            RaveTransform<float> trot; trot.rot = quatFromAxisAngle(RaveVector<float>(1,0,0),(float)PI);
            _tCamera = _tCamera*trot;
            bTransSpecified = true;
        }
        else if( xmlname == "camrotationaxis" ) {
            Vector vaxis; dReal fangle=0;
            string_stream_ >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
            _tCamera.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
            bTransSpecified = true;
        }
        else if( xmlname == "camrotmat" ) {
            TransformMatrix tnew;
            string_stream_ >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
            _tCamera.rot = Transform(tnew).rot;
            bTransSpecified = true;
        }
        else if( xmlname == "camtrans" ) {
            string_stream_ >> _tCamera.trans.x >> _tCamera.trans.y >> _tCamera.trans.z;
            bTransSpecified = true;
        }
        else if( xmlname == "camfocal" ) {
            string_stream_ >> _fCameraFocalDistance;
            bTransSpecified = true;
        }
        else if( xmlname == "plugin" ) {
            string pluginname;
            string_stream_ >> pluginname;
            RaveLoadPlugin(pluginname);
        }
        else if(xmlname == "unit"){
            std::pair<std::string, dReal> unit;
            string_stream_ >> unit.first >> unit.second;
            _penv->SetUnit(unit);
        }

        if( xmlname !=_processingtag ) {
            RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
        }
        _processingtag = "";
        return false;
    }

protected:
    EnvironmentBasePtr _penv;
    InterfaceBasePtr _pinterface;         // current processed interface
    Vector vBkgndColor;
    Transform _tCamera;         //!< default camera transformationn
    float _fCameraFocalDistance;
    string _processingtag;
    bool bTransSpecified;
    bool _bInEnvironment;
};

BaseXMLReaderPtr CreateEnvironmentReader(EnvironmentBasePtr penv, const AttributesList &atts)
{
    return BaseXMLReaderPtr(new EnvironmentXMLReader(penv,atts,false));
}

BaseXMLReaderPtr CreateInterfaceReader(EnvironmentBasePtr penv, InterfaceType type, InterfaceBasePtr& pinterface, const std::string& xmltag, const AttributesList &atts)
{
    switch(type) {
    case PT_Planner: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_Planner>(penv,pinterface,xmltag,atts));
    case PT_Robot: {
        KinBodyPtr pbody = RaveInterfaceCast<KinBody>(pinterface);
        int rootoffset = 0;
        if( !!pbody ) {
            rootoffset = (int)pbody->GetLinks().size();
        }
        return InterfaceXMLReaderPtr(new RobotXMLReader(penv,pinterface,atts,rootoffset));
    }
    case PT_SensorSystem: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_SensorSystem>(penv,pinterface,xmltag,atts));
    case PT_Controller: return InterfaceXMLReaderPtr(new ControllerXMLReader(penv,pinterface,atts));
    case PT_Module: return InterfaceXMLReaderPtr(new ModuleXMLReader(penv,pinterface,atts));
    case PT_IkSolver: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_IkSolver>(penv,pinterface,xmltag,atts));
    case PT_KinBody: {
        KinBodyPtr pbody = RaveInterfaceCast<KinBody>(pinterface);
        int rootoffset = 0;
        if( !!pbody ) {
            rootoffset = (int)pbody->GetLinks().size();
        }
        return InterfaceXMLReaderPtr(new KinBodyXMLReader(penv,pinterface,type,atts,rootoffset));
    }
    case PT_PhysicsEngine: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_PhysicsEngine>(penv,pinterface,xmltag,atts));
    case PT_Sensor: return InterfaceXMLReaderPtr(new SensorXMLReader(penv,pinterface,atts));
    case PT_CollisionChecker: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_CollisionChecker>(penv,pinterface,xmltag,atts));
    case PT_Trajectory: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_Trajectory>(penv,pinterface,xmltag,atts));
    case PT_Viewer: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_Viewer>(penv,pinterface,xmltag,atts));
    case PT_SpaceSampler: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_SpaceSampler>(penv,pinterface,xmltag,atts));
    }

    throw OpenRAVEException(str(boost::format(_tr("could not create interface of type %d"))%type),ORE_InvalidArguments);
}

class GlobalInterfaceXMLReader : public StreamXMLReader
{
public:
    GlobalInterfaceXMLReader(EnvironmentBasePtr penv, const AttributesList &atts, bool bAddToEnvironment=false)
		: _penv(penv), _atts(atts), _bAddToEnvironment(bAddToEnvironment) 
	{
    }
    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList &atts)
    {
        switch( StreamXMLReader::startElement(xmlname,atts) ) 
		{
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }
        AttributesList newatts = atts;
        newatts.insert(newatts.end(),_atts.begin(),_atts.end());
        _pinterface.reset();

        if( xmlname == "environment" ) 
		{
            cur_reader_ = CreateEnvironmentReader(_penv,newatts);
            if( !!cur_reader_ ) 
			{
                return PE_Support;
            }
        }

        // check for any plugins
        FOREACHC(itname,RaveGetInterfaceNamesMap()) 
		{
            if( xmlname == itname->second ) 
			{
                if( !!_pinterface ) 
				{
                    throw OpenRAVEException(_tr("interface should not be initialized"));
                }
                cur_reader_ = CreateInterfaceReader(_penv,itname->first,_pinterface,"",newatts);
                if( !_pinterface ) {
                    throw OpenRAVEException(str(boost::format(_tr("failed to create interface %s"))%itname->second));
                }
                return PE_Support;
            }
        }

        throw OpenRAVEException(str(boost::format(_tr("invalid interface tag %s"))%xmlname));
    }

    virtual bool endElement(const std::string& xmlname)
    {
        if( !!cur_reader_ ) {
            if( cur_reader_->endElement(xmlname) ) {
                if( !!_pinterface ) {
                    if( _bAddToEnvironment ) {
                        // set joint values if kinbody or robot
                        if( !!std::dynamic_pointer_cast<RobotXMLReader>(cur_reader_) ) {
                            std::shared_ptr<RobotXMLReader> robotreader = std::dynamic_pointer_cast<RobotXMLReader>(cur_reader_);
                            BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_Robot);
                            RobotBasePtr probot = RaveInterfaceCast<RobotBase>(_pinterface);
                            _penv->Add(probot);
                            if( !!robotreader->GetJointValues() ) {
                                if( (int)robotreader->GetJointValues()->size() != probot->GetDOF() ) {
                                    RAVELOG_WARN(str(boost::format("<jointvalues> wrong number of values %d!=%d, robot=%s")%robotreader->GetJointValues()->size()%probot->GetDOF()%probot->GetName()));
                                }
                                else {
                                    probot->SetDOFValues(*robotreader->GetJointValues());
                                }
                            }
                        }
                        else if( !!std::dynamic_pointer_cast<KinBodyXMLReader>(cur_reader_) ) {
                            KinBodyXMLReaderPtr kinbodyreader = std::dynamic_pointer_cast<KinBodyXMLReader>(cur_reader_);
                            BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_KinBody);
                            KinBodyPtr pbody = RaveInterfaceCast<KinBody>(_pinterface);
                            _penv->Add(pbody);
                            if( !!kinbodyreader->GetJointValues() ) {
                                if( (int)kinbodyreader->GetJointValues()->size() != pbody->GetDOF() ) {
                                    RAVELOG_WARN(str(boost::format("<jointvalues> wrong number of values %d!=%d, body=%s")%kinbodyreader->GetJointValues()->size()%pbody->GetDOF()%pbody->GetName()));
                                }
                                else {
                                    pbody->SetDOFValues(*kinbodyreader->GetJointValues());
                                }
                            }
                        }
                        else if( !!std::dynamic_pointer_cast<ModuleXMLReader>(cur_reader_) ) {
                            ModuleXMLReaderPtr modulereader = std::dynamic_pointer_cast<ModuleXMLReader>(cur_reader_);
                            ModuleBasePtr module = RaveInterfaceCast<ModuleBase>(_pinterface);
                            if( !!module ) {
                                int ret = _penv->AddModule(module,modulereader->GetArgs());
                                if( ret ) {
                                    RAVELOG_WARN(str(boost::format("module %s returned %d\n")%module->GetXMLId()%ret));
                                }
                            }
                        }
                        else {
                            _penv->Add(_pinterface);
                        }
                    }
                    return true;
                }
                bool bisenvironment = !!std::dynamic_pointer_cast<EnvironmentXMLReader>(cur_reader_);
                cur_reader_.reset();
                return bisenvironment;
            }
        }
        return false;
    }

    virtual XMLReadablePtr GetReadable() {
        return XMLReadablePtr(new InterfaceXMLReadable(_pinterface));
    }
protected:
    EnvironmentBasePtr _penv;
    InterfaceBasePtr _pinterface;         // current processed interface
    AttributesList _atts;         //!< attributes to always set on newly created interfaces
    bool _bAddToEnvironment; //!< if true, will add interface to environment
};

BaseXMLReaderPtr CreateInterfaceReader(EnvironmentBasePtr penv, const AttributesList &atts,bool bAddToEnvironment)
{
    return BaseXMLReaderPtr(new OpenRAVEXMLParser::GlobalInterfaceXMLReader(penv,atts,bAddToEnvironment));
}

} // end namespace OpenRAVEXMLParser
