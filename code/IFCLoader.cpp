/*
Open Asset Import Library (ASSIMP)
----------------------------------------------------------------------

Copyright (c) 2006-2010, ASSIMP Development Team
All rights reserved.

Redistribution and use of this software in source and binary forms, 
with or without modification, are permitted provided that the 
following conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.

* Neither the name of the ASSIMP team, nor the names of its
  contributors may be used to endorse or promote products
  derived from this software without specific prior
  written permission of the ASSIMP Development Team.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------
*/

/** @file  IFC.cpp
 *  @brief Implementation of the Industry Foundation Classes loader 
 */
#include "AssimpPCH.h"

#ifndef ASSIMP_BUILD_NO_IFC_IMPORTER

#include "IFCLoader.h"
#include "STEPFileReader.h"
#include "IFCReaderGen.h"

#include "StreamReader.h"
#include "MemoryIOWrapper.h"
#include "ProcessHelper.h"

#include <boost/tuple/tuple.hpp>

using namespace Assimp;
using namespace Assimp::Formatter;
namespace EXPRESS = STEP::EXPRESS;

template<> const std::string LogFunctions<IFCImporter>::log_prefix = "IFC: ";


/* DO NOT REMOVE this comment block. The genentitylist.sh script
 * just looks for names adhering to the IFC :: IfcSomething naming scheme
 * and includes all matches in the whitelist for code-generation. Thus,
 * all entity classes that are only indirectly referenced need to be
 * mentioned explicitly.

  IFC::IfcRepresentationMap
  IFC::IfcProductRepresentation
  IFC::IfcUnitAssignment
  IFC::IfcClosedShell
  IFC::IfcDoor

 */

namespace {

	// helper for std::for_each to delete all heap-allocated items in a container
template<typename T>
struct delete_fun
{
	void operator()(T* del) {
		delete del;
	}
};


	// intermediate data dump during conversion
struct ConversionData 
{
	ConversionData(const STEP::DB& db, const IFC::IfcProject& proj, aiScene* out,const IFCImporter::Settings& settings)
		: len_scale(1.0)
		, angle_scale(1.0)
		, db(db)
		, proj(proj)
		, out(out)
		, settings(settings)
	{}

	~ConversionData() {
		std::for_each(meshes.begin(),meshes.end(),delete_fun<aiMesh>());
		std::for_each(materials.begin(),materials.end(),delete_fun<aiMaterial>());
	}

	float len_scale, angle_scale;
	bool plane_angle_in_radians;

	const STEP::DB& db;
	const IFC::IfcProject& proj;
	aiScene* out;

	aiMatrix4x4 wcs;
	std::vector<aiMesh*> meshes;
	std::vector<aiMaterial*> materials;

	typedef std::map<const IFC::IfcRepresentationItem*, std::vector<unsigned int> > MeshCache;
	MeshCache cached_meshes;

	const IFCImporter::Settings& settings;
};

	// helper used during mesh construction
struct TempMesh
{
	std::vector<aiVector3D> verts;
	std::vector<unsigned int> vertcnt;
	std::vector<unsigned int> mat_idx;

	aiMesh* ToMesh() {
		ai_assert(verts.size() == std::accumulate(vertcnt.begin(),vertcnt.end(),0));

		if (verts.empty()) {
			return NULL;
		}

		std::auto_ptr<aiMesh> mesh(new aiMesh());

		// copy vertices
		mesh->mNumVertices = static_cast<unsigned int>(verts.size());
		mesh->mVertices = new aiVector3D[mesh->mNumVertices];
		std::copy(verts.begin(),verts.end(),mesh->mVertices);

		// and build up faces
		mesh->mNumFaces = static_cast<unsigned int>(vertcnt.size());
		mesh->mFaces = new aiFace[mesh->mNumFaces];

		for(unsigned int i = 0, acc = 0; i < mesh->mNumFaces; ++i) {
			aiFace& f = mesh->mFaces[i];

			f.mNumIndices = vertcnt[i];
			f.mIndices = new unsigned int[f.mNumIndices];
			for(unsigned int a = 0; a < f.mNumIndices; ++a) {
				f.mIndices[a] = acc++;
			}
		}

		// XXX materials
		mesh->mMaterialIndex = UINT_MAX;
		return mesh.release();
	}
};



// forward declarations
float ConvertSIPrefix(const std::string& prefix);
void SetUnits(ConversionData& conv);
void ConvertAxisPlacement(aiMatrix4x4& out, const IFC::IfcAxis2Placement& in, ConversionData& conv);
void SetCoordinateSpace(ConversionData& conv);
void ProcessSpatialStructures(ConversionData& conv);
aiNode* ProcessSpatialStructure(aiNode* parent, const IFC::IfcProduct& el ,ConversionData& conv);
void ProcessProductRepresentation(const IFC::IfcProduct& el, aiNode* nd, ConversionData& conv);
void MakeTreeRelative(ConversionData& conv);
void ConvertUnit(const EXPRESS::DataType* dt,ConversionData& conv);

} // anon

// ------------------------------------------------------------------------------------------------
// Constructor to be privately used by Importer
IFCImporter::IFCImporter()
{}

// ------------------------------------------------------------------------------------------------
// Destructor, private as well 
IFCImporter::~IFCImporter()
{
}

// ------------------------------------------------------------------------------------------------
// Returns whether the class can handle the format of the given file. 
bool IFCImporter::CanRead( const std::string& pFile, IOSystem* pIOHandler, bool checkSig) const
{
	const std::string& extension = GetExtension(pFile);
	if (extension == "ifc") {
		return true;
	}

	else if ((!extension.length() || checkSig) && pIOHandler)	{
		// note: this is the common identification for STEP-encoded files, so
		// it is only unambiguous as long as we don't support any further
		// file formats with STEP as their encoding.
		const char* tokens[] = {"ISO-10303-21"};
		return SearchFileHeaderForToken(pIOHandler,pFile,tokens,1);
	}
	return false;
}

// ------------------------------------------------------------------------------------------------
// List all extensions handled by this loader
void IFCImporter::GetExtensionList(std::set<std::string>& app) 
{
	app.insert("ifc");
}


// ------------------------------------------------------------------------------------------------
// Setup configuration properties for the loader
void IFCImporter::SetupProperties(const Importer* pImp)
{
	settings.skipSpaceRepresentations = pImp->GetPropertyBool(AI_CONFIG_IMPORT_IFC_SKIP_SPACE_REPRESENTATIONS,true);
	settings.skipCurveRepresentations = pImp->GetPropertyBool(AI_CONFIG_IMPORT_IFC_SKIP_CURVE_REPRESENTATIONS,true);
}


// ------------------------------------------------------------------------------------------------
// Imports the given file into the given scene structure. 
void IFCImporter::InternReadFile( const std::string& pFile, 
	aiScene* pScene, IOSystem* pIOHandler)
{
	boost::shared_ptr<IOStream> stream(pIOHandler->Open(pFile));
	if (!stream) {
		ThrowException("Could not open file for reading");
	}

	boost::scoped_ptr<STEP::DB> db(STEP::ReadFileHeader(stream));
	const STEP::HeaderInfo& head = const_cast<const STEP::DB&>(*db).GetHeader();

	if(!head.fileSchema.size() || head.fileSchema.substr(0,3) != "IFC") {
		ThrowException("Unrecognized file schema: " + head.fileSchema);
	}

	if (!DefaultLogger::isNullLogger()) {
		LogDebug("File schema is \'" + head.fileSchema + '\'');
		if (head.timestamp.length()) {
			LogDebug("Timestamp \'" + head.timestamp + '\'');
		}
		if (head.app.length()) {
			LogDebug("Application/Exporter identline is \'" + head.app  + '\'');
		}
	}

	// obtain a copy of the machine-generated IFC scheme
	EXPRESS::ConversionSchema schema;
	IFC::GetSchema(schema);

	// feed the IFC schema into the reader and pre-parse all lines
	STEP::ReadFile(*db, schema);

	const STEP::LazyObject* proj =  db->GetObject("ifcproject");
	if (!proj) {
		ThrowException("missing IfcProject entity");
	}

	ConversionData conv(*db,proj->To<IFC::IfcProject>(),pScene,settings);
	SetUnits(conv);
	SetCoordinateSpace(conv);
	ProcessSpatialStructures(conv);
	MakeTreeRelative(conv);

#ifdef ASSIMP_IFC_TEST
	db->EvaluateAll();
#endif

	// do final data copying
	if (conv.meshes.size()) {
		pScene->mNumMeshes = static_cast<unsigned int>(conv.meshes.size());
		pScene->mMeshes = new aiMesh*[pScene->mNumMeshes]();
		std::copy(conv.meshes.begin(),conv.meshes.end(),pScene->mMeshes);

		// needed to keep the d'tor from burning us
		conv.meshes.clear();
	}

	if (conv.materials.size()) {
		pScene->mNumMaterials = static_cast<unsigned int>(conv.materials.size());
		pScene->mMaterials = new aiMaterial*[pScene->mNumMaterials]();
		std::copy(conv.materials.begin(),conv.materials.end(),pScene->mMaterials);

		// needed to keep the d'tor from burning us
		conv.materials.clear();
	}

	// apply world coordinate system (which includes the scaling to convert to meters and a -90 degrees rotation around x)
	aiMatrix4x4 scale, rot;
	aiMatrix4x4::Scaling(aiVector3D(conv.len_scale,conv.len_scale,conv.len_scale),scale);
	aiMatrix4x4::RotationX(-AI_MATH_HALF_PI_F,rot);

	pScene->mRootNode->mTransformation = rot * scale * conv.wcs * pScene->mRootNode->mTransformation;

	// this must be last because objects are evaluated lazily as we process them
	if ( !DefaultLogger::isNullLogger() ){
		LogDebug((Formatter::format(),"STEP: evaluated ",db->GetEvaluatedObjectCount()," object records"));
	}
}

namespace {

// ------------------------------------------------------------------------------------------------
bool IsTrue(const EXPRESS::BOOLEAN& in)
{
	return (std::string)in == "TRUE" || (std::string)in == "T";
}

// ------------------------------------------------------------------------------------------------
float ConvertSIPrefix(const std::string& prefix)
{
	if (prefix == "EXA") {
		return 1e18f;
	}
	else if (prefix == "PETA") {
		return 1e15f;
	}
	else if (prefix == "TERA") {
		return 1e12f;
	}
	else if (prefix == "GIGA") {
		return 1e9f;
	}
	else if (prefix == "MEGA") {
		return 1e6f;
	}
	else if (prefix == "KILO") {
		return 1e3f;
	}
	else if (prefix == "HECTO") {
		return 1e2f;
	}
	else if (prefix == "DECA") {
		return 1e-0f;
	}
	else if (prefix == "DECI") {
		return 1e-1f;
	}
	else if (prefix == "CENTI") {
		return 1e-2f;
	}
	else if (prefix == "MILLI") {
		return 1e-3f;
	}
	else if (prefix == "MICRO") {
		return 1e-6f;
	}
	else if (prefix == "NANO") {
		return 1e-9f;
	}
	else if (prefix == "PICO") {
		return 1e-12f;
	}
	else if (prefix == "FEMTO") {
		return 1e-15f;
	}
	else if (prefix == "ATTO") {
		return 1e-18f;
	}
	else {
		IFCImporter::LogError("Unrecognized SI prefix: " + prefix);
		return 1;
	}
}

// ------------------------------------------------------------------------------------------------
void ConvertUnit(const IFC::IfcNamedUnit& unit,ConversionData& conv)
{
	if(const IFC::IfcSIUnit* const si = unit.ToPtr<IFC::IfcSIUnit>()) {

		if(si->UnitType == "LENGTHUNIT") { 
			conv.len_scale = si->Prefix ? ConvertSIPrefix(si->Prefix) : 1.f;
			IFCImporter::LogDebug("got units used for lengths");
		}
		if(si->UnitType == "PLANEANGLEUNIT") { 
			if (si->Name != "RADIAN") {
				IFCImporter::LogWarn("expected base unit for angles to be radian");
			}
		}
	}
	else if(const IFC::IfcConversionBasedUnit* const convu = unit.ToPtr<IFC::IfcConversionBasedUnit>()) {

		if(convu->UnitType == "PLANEANGLEUNIT") { 
			try {
				conv.angle_scale = convu->ConversionFactor->ValueComponent->To<EXPRESS::REAL>();
				ConvertUnit(convu->ConversionFactor->UnitComponent,conv);
				IFCImporter::LogDebug("got units used for angles");
			}
			catch(std::bad_cast&) {
				IFCImporter::LogError("skipping unknown IfcConversionBasedUnit.ValueComponent entry - expected REAL");
			}
		}
	}
}

// ------------------------------------------------------------------------------------------------
void ConvertUnit(const EXPRESS::DataType* dt,ConversionData& conv)
{
	try {
		const EXPRESS::ENTITY& e = dt->To<IFC::ENTITY>();

		const IFC::IfcNamedUnit& unit = e.ResolveSelect<IFC::IfcNamedUnit>(conv.db);
		if(unit.UnitType != "LENGTHUNIT" && unit.UnitType != "PLANEANGLEUNIT") {
			return;
		}

		ConvertUnit(unit,conv);
	}
	catch(std::bad_cast&) {
		// not entity, somehow
		IFCImporter::LogError("skipping unknown IfcUnit entry - expected entity");
	}
}

// ------------------------------------------------------------------------------------------------
void SetUnits(ConversionData& conv)
{
	// see if we can determine the coordinate space used to express. 
	for(size_t i = 0; i <  conv.proj.UnitsInContext->Units.size(); ++i ) {
		ConvertUnit(conv.proj.UnitsInContext->Units[i],conv);
	}
}

// ------------------------------------------------------------------------------------------------
void ConvertColor(aiColor4D& out, const IFC::IfcColourRgb& in)
{
	out.r = in.Red;
	out.g = in.Green;
	out.b = in.Blue;
	out.a = 1.f;
}

// ------------------------------------------------------------------------------------------------
void ConvertColor(aiColor4D& out, const IFC::IfcColourOrFactor* in,ConversionData& conv,const aiColor4D* base)
{
	if (const EXPRESS::REAL* const r = in->ToPtr<EXPRESS::REAL>()) {
		out.r = out.g = out.b = *r;
		if(base) {
			out.r *= base->r;
			out.g *= base->g;
			out.b *= base->b;
			out.a = base->a;
		}
		else out.a = 1.0;
	}
	else if (const IFC::IfcColourRgb* const rgb = in->ResolveSelectPtr<IFC::IfcColourRgb>(conv.db)) {
		ConvertColor(out,*rgb);
	}
	else {
		IFCImporter::LogWarn("skipping unknown IfcColourOrFactor entity");
	}
}

// ------------------------------------------------------------------------------------------------
void ConvertCartesianPoint(aiVector3D& out, const IFC::IfcCartesianPoint& in)
{
	out = aiVector3D();
	for(size_t i = 0; i < in.Coordinates.size(); ++i) {
		out[i] = in.Coordinates[i];
	}
}

// ------------------------------------------------------------------------------------------------
void ConvertDirection(aiVector3D& out, const IFC::IfcDirection& in)
{
	out = aiVector3D();
	for(size_t i = 0; i < in.DirectionRatios.size(); ++i) {
		out[i] = in.DirectionRatios[i];
	}
	const float len = out.Length();
	if (len<1e-6) {
		IFCImporter::LogWarn("direction vector too small, normalizing would result in a division by zero");
		return;
	}
	out /= len;
}

// ------------------------------------------------------------------------------------------------
void AssignMatrixAxes(aiMatrix4x4& out, const aiVector3D& x, const aiVector3D& y, const aiVector3D& z)
{
	out.a1 = x.x;
	out.b1 = x.y;
	out.c1 = x.z;

	out.a2 = y.x;
	out.b2 = y.y;
	out.c2 = y.z;

	out.a3 = z.x;
	out.b3 = z.y;
	out.c3 = z.z;
}

// ------------------------------------------------------------------------------------------------
void ConvertAxisPlacement(aiMatrix4x4& out, const IFC::IfcAxis2Placement3D& in, ConversionData& conv)
{
	aiVector3D loc;
	ConvertCartesianPoint(loc,in.Location);

	aiVector3D z(0.f,0.f,1.f),r(0.f,1.f,0.f),x;

	if (in.Axis) { 
		ConvertDirection(z,*in.Axis.Get());
	}
	if (in.RefDirection) {
		ConvertDirection(r,*in.RefDirection.Get());
	}

	aiVector3D v = r.Normalize();
	aiVector3D tmpx = z * (v*z);

	x = (v-tmpx).Normalize();
	aiVector3D y = (z^x);

	aiMatrix4x4::Translation(loc,out);
	AssignMatrixAxes(out,x,y,z);
}

// ------------------------------------------------------------------------------------------------
void ConvertAxisPlacement(aiMatrix4x4& out, const IFC::IfcAxis2Placement2D& in, ConversionData& conv)
{
	aiVector3D loc;
	ConvertCartesianPoint(loc,in.Location);

	aiVector3D x(1.f,0.f,1.f);
	if (in.RefDirection) {
		ConvertDirection(x,*in.RefDirection.Get());
	}

	const aiVector3D y = aiVector3D(x.y,-x.x,0.f);

	aiMatrix4x4::Translation(loc,out);
	AssignMatrixAxes(out,x,y,aiVector3D(0.f,0.f,1.f));
}

// ------------------------------------------------------------------------------------------------
void ConvertAxisPlacement(aiVector3D& axis, aiVector3D& pos, const IFC::IfcAxis1Placement& in, ConversionData& conv)
{
	ConvertCartesianPoint(pos,in.Location);
	if (in.Axis) {
		ConvertDirection(axis,in.Axis.Get());
	}
	else {
		axis = aiVector3D(0.f,0.f,1.f);
	}
}

// ------------------------------------------------------------------------------------------------
void ConvertAxisPlacement(aiMatrix4x4& out, const IFC::IfcAxis2Placement& in, ConversionData& conv)
{
	if(const IFC::IfcAxis2Placement3D* pl3 = in.ResolveSelectPtr<IFC::IfcAxis2Placement3D>(conv.db)) {
		ConvertAxisPlacement(out,*pl3,conv);
	}
	else if(const IFC::IfcAxis2Placement2D* pl2 = in.ResolveSelectPtr<IFC::IfcAxis2Placement2D>(conv.db)) {
		ConvertAxisPlacement(out,*pl2,conv);
	}
	else {
		IFCImporter::LogWarn("skipping unknown IfcAxis2Placement entity");
	}
}

// ------------------------------------------------------------------------------------------------
void SetCoordinateSpace(ConversionData& conv)
{
	const IFC::IfcRepresentationContext* fav = NULL;
	BOOST_FOREACH(const IFC::IfcRepresentationContext& v, conv.proj.RepresentationContexts) {
		fav = &v;
		// Model should be the most suitable type of context, hence ignore the others 
		if (v.ContextType && v.ContextType.Get() == "Model") { 
			break;
		}
	}
	if (fav) {
		if(const IFC::IfcGeometricRepresentationContext* const geo = fav->ToPtr<IFC::IfcGeometricRepresentationContext>()) {
			ConvertAxisPlacement(conv.wcs, *geo->WorldCoordinateSystem, conv);
			IFCImporter::LogDebug("got world coordinate system");
		}
	}
}

// ------------------------------------------------------------------------------------------------
void ConvertTransformOperator(aiMatrix4x4& out, const IFC::IfcCartesianTransformationOperator& op)
{
	aiVector3D loc;
	ConvertCartesianPoint(loc,op.LocalOrigin);

	aiVector3D x(1.f,0.f,0.f),y(0.f,1.f,0.f),z(0.f,0.f,1.f);
	if (op.Axis1) {
		ConvertDirection(x,*op.Axis1.Get());
	}
	if (op.Axis2) {
		ConvertDirection(y,*op.Axis2.Get());
	}
	if (const IFC::IfcCartesianTransformationOperator3D* op2 = op.ToPtr<IFC::IfcCartesianTransformationOperator3D>()) {
		if(op2->Axis3) {
			ConvertDirection(z,*op2->Axis3.Get());
		}
	}

	aiMatrix4x4 locm;
	aiMatrix4x4::Translation(loc,locm);	
	AssignMatrixAxes(out,x,y,z);

	const float sc = op.Scale?op.Scale.Get():1.f;

	aiMatrix4x4 s;
	aiMatrix4x4::Scaling(aiVector3D(sc,sc,sc),s);

	out = locm * out * s;
}

// ------------------------------------------------------------------------------------------------
bool ProcessPolyloop(const IFC::IfcPolyLoop& loop, TempMesh& meshout, ConversionData& conv)
{
	size_t cnt = 0;
	BOOST_FOREACH(const IFC::IfcCartesianPoint& c, loop.Polygon) {
		aiVector3D tmp;
		ConvertCartesianPoint(tmp,c);

		meshout.verts.push_back(tmp);
		++cnt;
	}
	// zero- or one- vertex polyloops simply ignored
	if (cnt >= 1) { 
		meshout.vertcnt.push_back(cnt);
		return true;
	}
	
	if (cnt==1) {
		meshout.vertcnt.pop_back();
	}
	return false;
}

// ------------------------------------------------------------------------------------------------
void ProcessConnectedFaceSet(const IFC::IfcConnectedFaceSet& fset, TempMesh& result, ConversionData& conv)
{
	BOOST_FOREACH(const IFC::IfcFace& face, fset.CfsFaces) {
		TempMesh meshout;

		size_t ob = face.Bounds.size(), cnt = 0;
		BOOST_FOREACH(const IFC::IfcFaceBound& bound, face.Bounds) {
			
			if(const IFC::IfcPolyLoop* const polyloop = bound.Bound->ToPtr<IFC::IfcPolyLoop>()) {
				if(ProcessPolyloop(*polyloop, meshout, conv)) {
					if(bound.ToPtr<IFC::IfcFaceOuterBound>()) {
						ob = cnt;
					}
					++cnt;
				}
			}
			else {
				IFCImporter::LogWarn("skipping unknown IfcFaceBound entity, type is " + bound.Bound->GetClassName());
				continue;
			}

			if(!IsTrue(bound.Orientation)) {
				size_t c = 0;
				BOOST_FOREACH(unsigned int& i, meshout.vertcnt) {
					std::reverse(meshout.verts.begin() + cnt,meshout.verts.begin() + cnt + c);
					cnt += c;
				}
			}
			
		}

		result.vertcnt.reserve(meshout.vertcnt.size()+result.vertcnt.size());
		if (meshout.vertcnt.size() <= 1) {
			result.verts.reserve(meshout.verts.size()+result.verts.size());

			std::copy(meshout.verts.begin(),meshout.verts.end(),std::back_inserter(result.verts));
			std::copy(meshout.vertcnt.begin(),meshout.vertcnt.end(),std::back_inserter(result.vertcnt));
			continue;
		}

		IFCImporter::LogDebug("fixing polygon with holes for triangulation via ear-cutting");

		// each hole results in two extra vertices
		result.verts.reserve(meshout.verts.size()+cnt*2+result.verts.size());

		// handle polygons with holes. our built in triangulation won't handle them as is, but
		// the ear cutting algorithm is solid enough to deal with them if we join the inner
		// holes with the outer boundaries by dummy connections.
		size_t outer_polygon_start = 0;
		
		// see if one of the polygons is a IfcFaceOuterBound - treats this as the outer boundary.
		// sadly we can't rely on it, the docs say 'At most one of the bounds shall be of the type IfcFaceOuterBound' 
		std::vector<unsigned int>::iterator outer_polygon = meshout.vertcnt.end(), begin=meshout.vertcnt.begin(),  iit;
		if (ob < face.Bounds.size()) {
			outer_polygon = begin + ob;
			outer_polygon_start = std::accumulate(begin,outer_polygon,0);
		}
		else {
			float area_outer_polygon = 1e-10f;

			// find the polygon with the largest area, it must be the outer bound. 
			size_t max_vcount = 0;
			for(iit = begin; iit != meshout.vertcnt.end(); ++iit) {
				ai_assert(*iit);
				max_vcount = std::max(max_vcount,static_cast<size_t>(*iit));
			}
			std::vector<float> temp((max_vcount+2)*4);
			size_t vidx = 0;
			for(iit = begin; iit != meshout.vertcnt.end(); vidx += *iit++) {

				for(size_t vofs = 0, cnt = 0; vofs < *iit; ++vofs) {
					const aiVector3D& v = meshout.verts[vidx+vofs];
					temp[cnt++] = v.x;
					temp[cnt++] = v.y;
					temp[cnt++] = v.z;
#ifdef _DEBUG
					temp[cnt] = std::numeric_limits<float>::quiet_NaN();
#endif
					++cnt;
				}
				
				aiVector3D nor;
				NewellNormal<4,4,4>(nor,*iit,&temp[0],&temp[1],&temp[2]);
				const float area = nor.SquareLength();

				if (area > area_outer_polygon) {
					area_outer_polygon = area;
					outer_polygon = iit;
					outer_polygon_start = vidx;
				}
			}
		}

		ai_assert(outer_polygon != meshout.vertcnt.end());	

		typedef boost::tuple<unsigned int, unsigned int, unsigned int> InsertionPoint;
		std::vector< InsertionPoint > insertions(*outer_polygon,boost::make_tuple(0u,0u,0u));

		// iterate through all other polyloops and find points in the outer polyloop that are close
		size_t vidx = 0;
		for(iit = begin; iit != meshout.vertcnt.end(); vidx += *iit++) {
			if (iit == outer_polygon) {
				continue;
			}

			size_t best_ofs,best_outer;
			float best_dist = 1e10;
			for(size_t vofs = 0; vofs < *iit; ++vofs) {
				const aiVector3D& v = meshout.verts[vidx+vofs];

				for(size_t outer = 0; outer < *outer_polygon; ++outer) {
					if (insertions[outer].get<0>()) {
						continue;
					}
					const aiVector3D& o = meshout.verts[outer_polygon_start+outer];
					const float d = (o-v).SquareLength();
										
					if (d < best_dist) {
						best_dist = d;
						best_ofs = vofs;
						best_outer = outer;
					}
				}		
			}
		
			// we will later insert a hidden connection line right after the closest point in the outer polygon
			insertions[best_outer] = boost::make_tuple(*iit,vidx,best_ofs);
		}

		// now that we collected all vertex connections to be added, build the output polygon
		cnt = *outer_polygon;
		for(size_t outer = 0; outer < *outer_polygon; ++outer) {
			const aiVector3D& o = meshout.verts[outer_polygon_start+outer];
			result.verts.push_back(o);

			const InsertionPoint& ins = insertions[outer];
			if (!ins.get<0>()) {
				continue;
			}

			for(size_t i = ins.get<2>(); i < ins.get<0>(); ++i) {
				result.verts.push_back(meshout.verts[ins.get<1>() + i]);
			}
			for(size_t i = 0; i < ins.get<2>(); ++i) {
				result.verts.push_back(meshout.verts[ins.get<1>() + i]);
			}

			// we need the first vertex of the inner polygon twice as we return to the
			// outer loop through the very same connection through which we got there.
			result.verts.push_back(meshout.verts[ins.get<1>() + ins.get<2>()]);

			// also append a copy of the initial insertion point to be able to continue the outer polygon
			result.verts.push_back(o);
			cnt += ins.get<0>()+2;
		}
		result.vertcnt.push_back(cnt);
	}
}

// ------------------------------------------------------------------------------------------------
void ProcessPolyLine(const IFC::IfcPolyline& def, TempMesh& meshout, ConversionData& conv)
{
	// this won't produce a valid mesh, it just spits out a list of vertices
	aiVector3D t;
	BOOST_FOREACH(const IFC::IfcCartesianPoint& cp, def.Points) {
		ConvertCartesianPoint(t,cp);
		meshout.verts.push_back(t);
	}
}

// ------------------------------------------------------------------------------------------------
void ProcessClosedProfile(const IFC::IfcArbitraryClosedProfileDef& def, TempMesh& meshout, ConversionData& conv)
{
	if(const IFC::IfcPolyline* poly = def.OuterCurve->ToPtr<IFC::IfcPolyline>()) {
		ProcessPolyLine(*poly,meshout,conv);
		if(meshout.verts.size()>2 && meshout.verts.front() == meshout.verts.back()) {
			meshout.verts.pop_back(); // duplicate element, first==last
		}
	}
	else {
		IFCImporter::LogWarn("skipping unknown IfcCurve entity, type is " + def.OuterCurve->GetClassName());
		return;
	}
}

// ------------------------------------------------------------------------------------------------
void ProcessOpenProfile(const IFC::IfcArbitraryOpenProfileDef& def, TempMesh& meshout, ConversionData& conv)
{
	if(const IFC::IfcPolyline* poly = def.Curve->ToPtr<IFC::IfcPolyline>()) {
		ProcessPolyLine(*poly,meshout,conv);
	}
	else {
		IFCImporter::LogWarn("skipping unknown IfcBoundedCurve entity, type is " + def.Curve->GetClassName());
		return;
	}
}

// ------------------------------------------------------------------------------------------------
void ProcessParametrizedProfile(const IFC::IfcParameterizedProfileDef& def, TempMesh& meshout, ConversionData& conv)
{
	if(const IFC::IfcRectangleProfileDef* const cprofile = def.ToPtr<IFC::IfcRectangleProfileDef>()) {
		const float x = cprofile->XDim*0.5f, y = cprofile->YDim*0.5f;

		meshout.verts.reserve(meshout.verts.size()+4);
		meshout.verts.push_back( aiVector3D( x, y, 0.f ));
		meshout.verts.push_back( aiVector3D(-x, y, 0.f ));
		meshout.verts.push_back( aiVector3D(-x,-y, 0.f ));
		meshout.verts.push_back( aiVector3D( x,-y, 0.f ));
		meshout.vertcnt.push_back(4);
	}
	else if( const IFC::IfcCircleProfileDef* const circle = def.ToPtr<IFC::IfcCircleProfileDef>()) {
		if( const IFC::IfcCircleHollowProfileDef* const hollow = def.ToPtr<IFC::IfcCircleHollowProfileDef>()) {
			// TODO
		}
		const size_t segments = 32;
		const float delta = AI_MATH_TWO_PI_F/segments, radius = circle->Radius;

		meshout.verts.reserve(segments);

		float angle = 0.f;
		for(size_t i = 0; i < segments; ++i, angle += delta) {
			meshout.verts.push_back( aiVector3D( cos(angle)*radius, sin(angle)*radius, 0.f ));
		}
	
		meshout.vertcnt.push_back(segments);
	}
	else {
		IFCImporter::LogWarn("skipping unknown IfcParameterizedProfileDef entity, type is " + def.GetClassName());
		return;
	}

	aiMatrix4x4 trafo;
	ConvertAxisPlacement(trafo, *def.Position,conv);

	BOOST_FOREACH(aiVector3D& v, meshout.verts) {
		v *= trafo;
	}
}

// ------------------------------------------------------------------------------------------------
bool ProcessProfile(const IFC::IfcProfileDef& prof, TempMesh& meshout, ConversionData& conv) 
{
	if(const IFC::IfcArbitraryClosedProfileDef* const cprofile = prof.ToPtr<IFC::IfcArbitraryClosedProfileDef>()) {
		ProcessClosedProfile(*cprofile,meshout,conv);
	}
	else if(const IFC::IfcArbitraryOpenProfileDef* const copen = prof.ToPtr<IFC::IfcArbitraryOpenProfileDef>()) {
		ProcessOpenProfile(*copen,meshout,conv);
	}
	else if(const IFC::IfcParameterizedProfileDef* const cparam = prof.ToPtr<IFC::IfcParameterizedProfileDef>()) {
		ProcessParametrizedProfile(*cparam,meshout,conv);
	}
	else {
		IFCImporter::LogWarn("skipping unknown IfcProfileDef entity, type is " + prof.GetClassName());
		return false;
	}
	return true;
}

// ------------------------------------------------------------------------------------------------
void FixupFaceOrientation(TempMesh& result)
{
	aiVector3D vavg;
	BOOST_FOREACH(aiVector3D& v, result.verts) {
		vavg += v;
	}

	// fixup face orientation.
	vavg /= static_cast<float>( result.verts.size() );

	size_t c = 0;
	BOOST_FOREACH(unsigned int cnt, result.vertcnt) {
		if (cnt>2){
			const aiVector3D& thisvert = result.verts[c];
			const aiVector3D normal((thisvert-result.verts[c+1])^(thisvert-result.verts[c+2]));
			if (normal*(thisvert-vavg) < 0) {
				std::reverse(result.verts.begin()+c,result.verts.begin()+cnt+c);
			}
		}
		c += cnt;
	}
}

// ------------------------------------------------------------------------------------------------
void ProcessRevolvedAreaSolid(const IFC::IfcRevolvedAreaSolid& solid, TempMesh& result, ConversionData& conv)
{
	TempMesh meshout;

	// first read the profile description
	if(!ProcessProfile(*solid.SweptArea,meshout,conv) || meshout.verts.size()<=1) {
		return;
	}

	aiVector3D axis, pos;
	ConvertAxisPlacement(axis,pos,solid.Axis,conv);

	aiMatrix4x4 tb0,tb1;
	aiMatrix4x4::Translation(pos,tb0);
	aiMatrix4x4::Translation(-pos,tb1);

	const std::vector<aiVector3D>& in = meshout.verts;
	const size_t size=in.size();
	
	bool has_area = solid.SweptArea->ProfileType == "AREA" && size>2;
	const float max_angle = solid.Angle*conv.angle_scale;
	if(fabs(max_angle) < 1e-3) {
		if(has_area) {
			result = meshout;
		}
		return;
	}

	const unsigned int cnt_segments = std::max(2u,static_cast<unsigned int>(16 * fabs(max_angle)/AI_MATH_HALF_PI_F));
	const float delta = max_angle/cnt_segments;

	has_area = has_area && fabs(max_angle) < AI_MATH_TWO_PI_F*0.99;
	
	result.verts.reserve(size*((cnt_segments+1)*4+(has_area?2:0)));
	result.vertcnt.reserve(size*cnt_segments+2);

	aiMatrix4x4 rot;
	rot = tb0 * aiMatrix4x4::Rotation(delta,axis,rot) * tb1;

	size_t base = 0;
	std::vector<aiVector3D>& out = result.verts;

	// dummy data to simplify later processing
	for(size_t i = 0; i < size; ++i) {
		out.insert(out.end(),4,in[i]);
	}

	for(unsigned int seg = 0; seg < cnt_segments; ++seg) {
		for(size_t i = 0; i < size; ++i) {
			const size_t next = (i+1)%size;

			result.vertcnt.push_back(4);
			const aiVector3D& base_0 = out[base+i*4+3],base_1 = out[base+next*4+3];

			out.push_back(base_0);
			out.push_back(base_1);
			out.push_back(rot*base_1);
			out.push_back(rot*base_0);
		}
		base += size*4;
	}

	out.erase(out.begin(),out.begin()+size*4);

	if(has_area) {
		// leave the triangulation of the profile area to the ear cutting 
		// implementation in aiProcess_Triangulate - for now we just
		// feed in two huge polygons.
		base -= size*8;
		for(size_t i = size; i--; ) {
			out.push_back(out[base+i*4+3]);
		}
		for(size_t i = 0; i < size; ++i ) {
			out.push_back(out[i*4]);
		}
		result.vertcnt.push_back(size);
		result.vertcnt.push_back(size);
	}

	aiMatrix4x4 trafo;
	ConvertAxisPlacement(trafo, solid.Position,conv);
	BOOST_FOREACH(aiVector3D& v, out) {
		v *= trafo;
	}

	FixupFaceOrientation(result);
	IFCImporter::LogDebug("generate mesh procedurally by radial extrusion (IfcRevolvedAreaSolid)");
}

// ------------------------------------------------------------------------------------------------
void ProcessExtrudedAreaSolid(const IFC::IfcExtrudedAreaSolid& solid, TempMesh& result, ConversionData& conv)
{
	TempMesh meshout;
	
	// first read the profile description
	if(!ProcessProfile(*solid.SweptArea,meshout,conv) || meshout.verts.size()<=1) {
		return;
	}

	aiVector3D dir;
	ConvertDirection(dir,solid.ExtrudedDirection);

	dir *= solid.Depth;

	// assuming that `meshout.verts` is now a list of vertex points forming 
	// the underlying profile, extrude along the given axis, forming new
	// triangles.
	
	const std::vector<aiVector3D>& in = meshout.verts;
	const size_t size=in.size();

	const bool has_area = solid.SweptArea->ProfileType == "AREA" && size>2;
	if(solid.Depth < 1e-3) {
		if(has_area) {
			meshout = result;
		}
		return;
	}

	result.verts.reserve(size*(has_area?4:2));
	result.vertcnt.reserve(meshout.vertcnt.size()+2);

	for(size_t i = 0; i < size; ++i) {
		const size_t next = (i+1)%size;

		result.vertcnt.push_back(4);

		result.verts.push_back(in[i]);
		result.verts.push_back(in[next]);
		result.verts.push_back(in[next]+dir);
		result.verts.push_back(in[i]+dir);
	}

	if(has_area) {
		// leave the triangulation of the profile area to the ear cutting 
		// implementation in aiProcess_Triangulate - for now we just
		// feed in two huge polygons.
		for(size_t i = size; i--; ) {
			result.verts.push_back(in[i]+dir);
		}
		for(size_t i = 0; i < size; ++i ) {
			result.verts.push_back(in[i]);
		}
		result.vertcnt.push_back(size);
		result.vertcnt.push_back(size);
	}

	aiMatrix4x4 trafo;
	ConvertAxisPlacement(trafo, solid.Position,conv);

	BOOST_FOREACH(aiVector3D& v, result.verts) {
		v *= trafo;
	}

	FixupFaceOrientation(result);
	IFCImporter::LogDebug("generate mesh procedurally by extrusion (IfcExtrudedAreaSolid)");
}

// ------------------------------------------------------------------------------------------------
void ProcessSweptAreaSolid(const IFC::IfcSweptAreaSolid& swept, TempMesh& meshout, ConversionData& conv)
{
	if(const IFC::IfcExtrudedAreaSolid* const solid = swept.ToPtr<IFC::IfcExtrudedAreaSolid>()) {
		ProcessExtrudedAreaSolid(*solid,meshout,conv);
	}
	else if(const IFC::IfcRevolvedAreaSolid* const rev = swept.ToPtr<IFC::IfcRevolvedAreaSolid>()) {
		ProcessRevolvedAreaSolid(*rev,meshout,conv);
	}
	else {
		IFCImporter::LogWarn("skipping unknown IfcSweptAreaSolid entity, type is " + swept.GetClassName());
	}
}

// ------------------------------------------------------------------------------------------------
enum Intersect {
	Intersect_No,
	Intersect_LiesOnPlane,
	Intersect_Yes
};

// ------------------------------------------------------------------------------------------------
Intersect IntersectSegmentPlane(const aiVector3D& p,const aiVector3D& n, const aiVector3D& e0, const aiVector3D& e1, aiVector3D& out) 
{
	const aiVector3D pdelta = e0 - p, seg = e1-e0;
	const float dotOne = n*seg, dotTwo = -(n*pdelta);

	if (fabs(dotOne) < 1e-6) {
		return fabs(dotTwo) < 1e-6f ? Intersect_LiesOnPlane : Intersect_No;
	}

	const float t = dotTwo/dotOne;
	// t must be in [0..1] if the intersection point is within the given segment
	if (t > 1.f || t < 0.f) {
		return Intersect_No;
	}
	out = e0+t*seg;
	return Intersect_Yes;
}

// ------------------------------------------------------------------------------------------------
void ProcessBoolean(const IFC::IfcBooleanResult& boolean, TempMesh& result, ConversionData& conv)
{
	if(const IFC::IfcBooleanClippingResult* const clip = boolean.ToPtr<IFC::IfcBooleanClippingResult>()) {
		if(clip->Operator != "DIFFERENCE") {
			IFCImporter::LogWarn("encountered unsupported boolean operator: " + (std::string)clip->Operator);
			return;
		}

		TempMesh meshout;
		const IFC::IfcHalfSpaceSolid* const hs = clip->SecondOperand->ResolveSelectPtr<IFC::IfcHalfSpaceSolid>(conv.db);
		if(!hs) {
			IFCImporter::LogError("expected IfcHalfSpaceSolid as second clipping operand");
			return;
		}

		const IFC::IfcPlane* const plane = hs->BaseSurface->ToPtr<IFC::IfcPlane>();
		if(!plane) {
			IFCImporter::LogError("expected IfcPlane as base surface for the IfcHalfSpaceSolid");
			return;
		}
		
		if(const IFC::IfcBooleanResult* const op0 = clip->FirstOperand->ResolveSelectPtr<IFC::IfcBooleanResult>(conv.db)) {
			ProcessBoolean(*op0,meshout,conv);
		}
		else if (const IFC::IfcSweptAreaSolid* const swept = clip->FirstOperand->ResolveSelectPtr<IFC::IfcSweptAreaSolid>(conv.db)) {
			ProcessSweptAreaSolid(*swept,meshout,conv);
		}
		else {
			IFCImporter::LogError("expected IfcSweptAreaSolid or IfcBooleanResult as first clipping operand");
			return;
		}

		// extract plane base position vector and normal vector
		aiVector3D p,n(0.f,0.f,1.f);
		if (plane->Position->Axis) {
			ConvertDirection(n,plane->Position->Axis.Get());
		}
		ConvertCartesianPoint(p,plane->Position->Location);

		if(!IsTrue(hs->AgreementFlag)) {
			n *= -1.f;
		}

		// clip the current contents of `meshout` against the plane we obtained from the second operand
		const std::vector<aiVector3D>& in = meshout.verts;
		std::vector<aiVector3D>& outvert = result.verts;
		std::vector<unsigned int>::const_iterator outer_polygon = meshout.vertcnt.end(), begin=meshout.vertcnt.begin(),  iit;
		
		unsigned int vidx = 0;
		for(iit = begin; iit != meshout.vertcnt.end(); vidx += *iit++) {

			unsigned int newcount = 0;
			for(unsigned int i = 0; i < *iit; ++i) {
				const aiVector3D& e0 = in[vidx+i], e1 = in[vidx+(i+1)%*iit];

				// does the next segment intersect the plane?
				aiVector3D isectpos;
				const Intersect isect = IntersectSegmentPlane(p,n,e0,e1,isectpos);
				if (isect == Intersect_No || isect == Intersect_LiesOnPlane) {
					if ( (e0-p).Normalize()*n > 0 ) {
						outvert.push_back(e0);
						++newcount;
					}
				}
				else if (isect == Intersect_Yes) {
					if ( (e0-p).Normalize()*n > 0 ) {
						// e0 is on the right side, so keep it
						outvert.push_back(e0);
						outvert.push_back(isectpos);
						newcount += 2;
					}
					else {
						// e0 is on the wrong side, so drop it and keep e1 instead
						outvert.push_back(isectpos);
						++newcount;
					}
				}
			}	

			if(newcount) {
				result.vertcnt.push_back(newcount);
			}
		}
		IFCImporter::LogDebug("generating CSG geometry by plane clipping (IfcBooleanClippingResult)");
	}
	else {
		IFCImporter::LogWarn("skipping unknown IfcBooleanResult entity, type is " + boolean.GetClassName());
	}
}

// ------------------------------------------------------------------------------------------------
int ConvertShadingMode(const std::string& name)
{
	if (name == "BLINN") {
		return aiShadingMode_Blinn;
	}
	else if (name == "FLAT" || name == "NOTDEFINED") {
		return aiShadingMode_NoShading;
	}
	else if (name == "PHONG") {
		return aiShadingMode_Phong;
	}
	IFCImporter::LogWarn("shading mode "+name+" not recognized by Assimp, using Phong instead");
	return aiShadingMode_Phong;
}

// ------------------------------------------------------------------------------------------------
unsigned int ProcessMaterials(const IFC::IfcRepresentationItem& item, ConversionData& conv)
{
	aiString name;
	aiColor4D col;

	if (conv.materials.empty()) {
		std::auto_ptr<MaterialHelper> mat(new MaterialHelper());

		name.Set("<IFCDefault>");
		mat->AddProperty(&name,AI_MATKEY_NAME);

		col = aiColor4D(0.6f,0.6f,0.6f,1.0f);
		mat->AddProperty(&col,1, AI_MATKEY_COLOR_DIFFUSE);

		conv.materials.push_back(mat.release());
	}

	STEP::DB::RefMapRange range = conv.db.GetRefs().equal_range(item.GetID());
	for(;range.first != range.second; ++range.first) {
		if(const IFC::IfcStyledItem* const styled = conv.db.GetObject((*range.first).second)->ToPtr<IFC::IfcStyledItem>()) {
			BOOST_FOREACH(const IFC::IfcPresentationStyleAssignment& as, styled->Styles) {
				BOOST_FOREACH(const IFC::IfcPresentationStyleSelect* sel, as.Styles) {
			
					if (const IFC::IfcSurfaceStyle* surf =  sel->ResolveSelectPtr<IFC::IfcSurfaceStyle>(conv.db)) {
						const std::string side = static_cast<std::string>(surf->Side);
						if (side != "BOTH") {
							IFCImporter::LogWarn("ignoring surface side marker on IFC::IfcSurfaceStyle: " + side);
						}

						std::auto_ptr<MaterialHelper> mat(new MaterialHelper());

						name.Set((surf->Name? surf->Name.Get() : "IfcSurfaceStyle_Unnamed"));
						mat->AddProperty(&name,AI_MATKEY_NAME);

						// now see which kinds of surface information are present
						BOOST_FOREACH(const IFC::IfcSurfaceStyleElementSelect* sel2, surf->Styles) {

							if (const IFC::IfcSurfaceStyleShading* shade = sel2->ResolveSelectPtr<IFC::IfcSurfaceStyleShading>(conv.db)) {
								aiColor4D col_base;

								ConvertColor(col_base, shade->SurfaceColour);
								mat->AddProperty(&col,1, AI_MATKEY_COLOR_DIFFUSE);

								if (const IFC::IfcSurfaceStyleRendering* ren = shade->ToPtr<IFC::IfcSurfaceStyleRendering>()) {
									
									if (ren->DiffuseColour) {
										ConvertColor(col, ren->DiffuseColour.Get(),conv,&col_base);
										mat->AddProperty(&col,1, AI_MATKEY_COLOR_DIFFUSE);
									}

									if (ren->SpecularColour) {
										ConvertColor(col, ren->SpecularColour.Get(),conv,&col_base);
										mat->AddProperty(&col,1, AI_MATKEY_COLOR_SPECULAR);
									}

									if (ren->TransmissionColour) {
										ConvertColor(col, ren->TransmissionColour.Get(),conv,&col_base);
										mat->AddProperty(&col,1, AI_MATKEY_COLOR_TRANSPARENT);
									}

									if (ren->ReflectionColour) {
										ConvertColor(col, ren->ReflectionColour.Get(),conv,&col_base);
										mat->AddProperty(&col,1, AI_MATKEY_COLOR_REFLECTIVE);
									}

									const int shading = (ren->SpecularHighlight && ren->SpecularColour)?ConvertShadingMode(ren->ReflectanceMethod):aiShadingMode_Gouraud;
									mat->AddProperty(&shading,1, AI_MATKEY_SHADING_MODEL);

									if (ren->SpecularHighlight) {
										if(const EXPRESS::REAL* rt = ren->SpecularHighlight.Get()->ToPtr<EXPRESS::REAL>()) {
											// at this point we don't distinguish between the two distinct ways of
											// specifying highlight intensities. leave this to the user.
											const float e = *rt;
											mat->AddProperty(&e,1,AI_MATKEY_SHININESS);
										}
										else {
											IFCImporter::LogWarn("unexpected type error, SpecularHighlight should be a REAL");
										}
									}
								}
							}
							else if (const IFC::IfcSurfaceStyleWithTextures* tex = sel2->ResolveSelectPtr<IFC::IfcSurfaceStyleWithTextures>(conv.db)) {
								// XXX
							}
						}

						conv.materials.push_back(mat.release());
						return conv.materials.size()-1;
					}
				}
			}
		}
	}
	return 0;
}

// ------------------------------------------------------------------------------------------------
bool ProcessTopologicalItem(const IFC::IfcTopologicalRepresentationItem& topo, std::vector<unsigned int>& mesh_indices, ConversionData& conv)
{
	TempMesh meshtmp;
	if(const IFC::IfcConnectedFaceSet* fset = topo.ToPtr<IFC::IfcConnectedFaceSet>()) {
		ProcessConnectedFaceSet(*fset,meshtmp,conv);
	}
	else {
		IFCImporter::LogWarn("skipping unknown IfcTopologicalRepresentationItem entity, type is " + topo.GetClassName());
		return false;
	}

	aiMesh* const mesh = meshtmp.ToMesh();
	if(mesh) {
		mesh->mMaterialIndex = ProcessMaterials(topo,conv);
		mesh_indices.push_back(conv.meshes.size());
		conv.meshes.push_back(mesh);
		return true;
	}
	return false;
}

// ------------------------------------------------------------------------------------------------
bool ProcessGeometricItem(const IFC::IfcGeometricRepresentationItem& geo, std::vector<unsigned int>& mesh_indices, ConversionData& conv)
{
	TempMesh meshtmp;
	if(const IFC::IfcShellBasedSurfaceModel* shellmod = geo.ToPtr<IFC::IfcShellBasedSurfaceModel>()) {
		BOOST_FOREACH(const IFC::IfcShell* shell,shellmod->SbsmBoundary) {
			try {
				const EXPRESS::ENTITY& e = shell->To<IFC::ENTITY>();
				const IFC::IfcConnectedFaceSet& fs = conv.db.MustGetObject(e).To<IFC::IfcConnectedFaceSet>(); 

				ProcessConnectedFaceSet(fs,meshtmp,conv);
			}
			catch(std::bad_cast&) {
				IFCImporter::LogWarn("unexpected type error, IfcShell ought to inherit from IfcConnectedFaceSet");
			}
		}
	}
	else if(const IFC::IfcSweptAreaSolid* swept = geo.ToPtr<IFC::IfcSweptAreaSolid>()) {
		ProcessSweptAreaSolid(*swept,meshtmp,conv);
	}
	else if(const IFC::IfcManifoldSolidBrep* brep = geo.ToPtr<IFC::IfcManifoldSolidBrep>()) {
		ProcessConnectedFaceSet(brep->Outer,meshtmp,conv);
	}
	else if(const IFC::IfcFaceBasedSurfaceModel* surf = geo.ToPtr<IFC::IfcFaceBasedSurfaceModel>()) {
		BOOST_FOREACH(const IFC::IfcConnectedFaceSet& fc, surf->FbsmFaces) {
			ProcessConnectedFaceSet(fc,meshtmp,conv);
		}
	}
	else if(const IFC::IfcBooleanResult* boolean = geo.ToPtr<IFC::IfcBooleanResult>()) {
		ProcessBoolean(*boolean,meshtmp,conv);
	}
	else if(const IFC::IfcBoundingBox* bb = geo.ToPtr<IFC::IfcBoundingBox>()) {
		// silently skip over bounding boxes
		return false; 
	}
	else {
		IFCImporter::LogWarn("skipping unknown IfcGeometricRepresentationItem entity, type is " + geo.GetClassName());
		return false;
	}

	aiMesh* const mesh = meshtmp.ToMesh();

	if(mesh) {
		mesh->mMaterialIndex = ProcessMaterials(geo,conv);
		mesh_indices.push_back(conv.meshes.size());
		conv.meshes.push_back(mesh);
		return true;
	}
	return false;
}

// ------------------------------------------------------------------------------------------------
void AssignAddedMeshes(std::vector<unsigned int>& mesh_indices,aiNode* nd,ConversionData& conv)
{
	if (!mesh_indices.empty()) {

		// make unique
		std::sort(mesh_indices.begin(),mesh_indices.end());
		std::vector<unsigned int>::iterator it_end = std::unique(mesh_indices.begin(),mesh_indices.end());
		
		const size_t size = std::distance(mesh_indices.begin(),it_end);

		nd->mNumMeshes = size;
		nd->mMeshes = new unsigned int[nd->mNumMeshes];
		for(unsigned int i = 0; i < nd->mNumMeshes; ++i) {
			nd->mMeshes[i] = mesh_indices[i];
		}
	}
}

// ------------------------------------------------------------------------------------------------
bool TryQueryMeshCache(const IFC::IfcRepresentationItem& item, std::vector<unsigned int>& mesh_indices, ConversionData& conv) 
{
	ConversionData::MeshCache::const_iterator it = conv.cached_meshes.find(&item);
	if (it != conv.cached_meshes.end()) {
		std::copy((*it).second.begin(),(*it).second.end(),std::back_inserter(mesh_indices));
		return true;
	}
	return false;
}

// ------------------------------------------------------------------------------------------------
void PopulateMeshCache(const IFC::IfcRepresentationItem& item, const std::vector<unsigned int>& mesh_indices, ConversionData& conv)
{
	conv.cached_meshes[&item] = mesh_indices;
}

// ------------------------------------------------------------------------------------------------
bool ProcessRepresentationItem(const IFC::IfcRepresentationItem& item, std::vector<unsigned int>& mesh_indices, ConversionData& conv)
{
	if(const IFC::IfcTopologicalRepresentationItem* const topo = item.ToPtr<IFC::IfcTopologicalRepresentationItem>()) {
		if (!TryQueryMeshCache(item,mesh_indices,conv)) {
			if(ProcessTopologicalItem(*topo,mesh_indices,conv)) {
				PopulateMeshCache(item,mesh_indices,conv);
			}
			else return false;
		}
		return true;
	}
	else if(const IFC::IfcGeometricRepresentationItem* const geo = item.ToPtr<IFC::IfcGeometricRepresentationItem>()) {
		if (!TryQueryMeshCache(item,mesh_indices,conv)) {
			if(ProcessGeometricItem(*geo,mesh_indices,conv)) {
				PopulateMeshCache(item,mesh_indices,conv);
				
			} 
			else return false;
		}
		return true;
	}
	return false;
}

// ------------------------------------------------------------------------------------------------
void ResolveObjectPlacement(aiMatrix4x4& m, const IFC::IfcObjectPlacement& place, ConversionData& conv)
{
	if (const IFC::IfcLocalPlacement* const local = place.ToPtr<IFC::IfcLocalPlacement>()){
		ConvertAxisPlacement(m, *local->RelativePlacement, conv);

		if (local->PlacementRelTo) {
			aiMatrix4x4 tmp;
			ResolveObjectPlacement(tmp,local->PlacementRelTo.Get(),conv);
			m = tmp * m;
		}
	}
	else {
		IFCImporter::LogWarn("skipping unknown IfcObjectPlacement entity, type is " + place.GetClassName());
	}
}

// ------------------------------------------------------------------------------------------------
void GetAbsTransform(aiMatrix4x4& out, const aiNode* nd, ConversionData& conv)
{
	aiMatrix4x4 t;
	if (nd->mParent) {
		GetAbsTransform(t,nd->mParent,conv);
	}
	out = t*nd->mTransformation;
}

// ------------------------------------------------------------------------------------------------
void ProcessMappedItem(const IFC::IfcMappedItem& mapped, aiNode* nd_src, std::vector< aiNode* >& subnodes_src, ConversionData& conv)
{
	// insert a custom node here, the cartesian transform operator is simply a conventional transformation matrix
	std::auto_ptr<aiNode> nd(new aiNode());
	nd->mName.Set("MappedItem");
	
	std::vector<unsigned int> meshes;

	const IFC::IfcRepresentation& repr = mapped.MappingSource->MappedRepresentation;
	BOOST_FOREACH(const IFC::IfcRepresentationItem& item, repr.Items) {
		if(!ProcessRepresentationItem(item,meshes,conv)) {
			IFCImporter::LogWarn("skipping unknown mapped entity, type is " + item.GetClassName());
		}
	}
	AssignAddedMeshes(meshes,nd.get(),conv);
		
	// handle the cartesian operator
	aiMatrix4x4 m;
	ConvertTransformOperator(m, *mapped.MappingTarget);

	aiMatrix4x4 msrc;
	ConvertAxisPlacement(msrc,*mapped.MappingSource->MappingOrigin,conv);

	aiMatrix4x4 minv = msrc;
	minv.Inverse();

	//aiMatrix4x4 correct;
	//GetAbsTransform(correct,nd_src,conv);

	nd->mTransformation = nd_src->mTransformation * minv * m * msrc;
	subnodes_src.push_back(nd.release());
}

// ------------------------------------------------------------------------------------------------
void ProcessProductRepresentation(const IFC::IfcProduct& el, aiNode* nd, std::vector< aiNode* >& subnodes, ConversionData& conv)
{
	if(!el.Representation) {
		return;
	}

	if(conv.settings.skipSpaceRepresentations) {
		if(const IFC::IfcSpace* const space = el.ToPtr<IFC::IfcSpace>()) {
			IFCImporter::LogWarn("skipping IfcSpace entity due to importer settings");
			return;
		}
	}

	std::vector<unsigned int> meshes;
	
	BOOST_FOREACH(const IFC::IfcRepresentation& repr, el.Representation.Get()->Representations) {
		if (conv.settings.skipCurveRepresentations && repr.RepresentationType && repr.RepresentationType.Get() == "Curve2D") {
			IFCImporter::LogWarn("skipping Curve2D representation item due to importer settings");
			continue;
		}
		BOOST_FOREACH(const IFC::IfcRepresentationItem& item, repr.Items) {
			if(const IFC::IfcMappedItem* const geo = item.ToPtr<IFC::IfcMappedItem>()) {
				ProcessMappedItem(*geo,nd,subnodes,conv);		
			}
			else {
				ProcessRepresentationItem(item,meshes,conv);
			}
		}
	}

	AssignAddedMeshes(meshes,nd,conv);
}

// ------------------------------------------------------------------------------------------------
aiNode* ProcessSpatialStructure(aiNode* parent, const IFC::IfcProduct& el, ConversionData& conv)
{
	const STEP::DB::RefMap& refs = conv.db.GetRefs();

	// add an output node for this spatial structure
	std::auto_ptr<aiNode> nd(new aiNode());
	nd->mName.Set(el.GetClassName()+"_"+(el.Name?el.Name:el.GlobalId));
	nd->mParent = parent;

	if(el.ObjectPlacement) {
		ResolveObjectPlacement(nd->mTransformation,el.ObjectPlacement.Get(),conv);
	}

	// convert everything contained directly within this structure,
	// this may result in more nodes.
	std::vector< aiNode* > subnodes;
	try {

		ProcessProductRepresentation(el,nd.get(),subnodes,conv);

		// locate aggregates and 'contained-in-here'-elements of this spatial structure and add them in recursively
		STEP::DB::RefMapRange range = refs.equal_range(el.GetID());

		for(STEP::DB::RefMapRange range2=range;range2.first != range.second; ++range2.first) {
			if(const IFC::IfcRelContainedInSpatialStructure* const cont = conv.db.GetObject((*range2.first).second)->
				ToPtr<IFC::IfcRelContainedInSpatialStructure>()) {
				
				BOOST_FOREACH(const IFC::IfcProduct& pro, cont->RelatedElements) {		
					subnodes.push_back( ProcessSpatialStructure(nd.get(),pro,conv) );
				}
				break;
			}
		}

		for(;range.first != range.second; ++range.first) {
			if(const IFC::IfcRelAggregates* const aggr = conv.db.GetObject((*range.first).second)->ToPtr<IFC::IfcRelAggregates>()) {

				// move aggregate elements to a separate node since they are semantically different than elements that are merely 'contained'
				std::auto_ptr<aiNode> nd_aggr(new aiNode());
				nd_aggr->mName.Set("$Aggregates");
				nd_aggr->mParent = nd.get();

				nd_aggr->mChildren = new aiNode*[aggr->RelatedObjects.size()]();
				BOOST_FOREACH(const IFC::IfcObjectDefinition& def, aggr->RelatedObjects) {
					if(const IFC::IfcProduct* const prod = def.ToPtr<IFC::IfcProduct>()) {
						nd_aggr->mChildren[nd_aggr->mNumChildren++] = ProcessSpatialStructure(nd_aggr.get(),*prod,conv);
					}
				}
			
				subnodes.push_back( nd_aggr.release() );
				break;
			}
		}

		if (subnodes.size()) {
			nd->mChildren = new aiNode*[subnodes.size()]();
			BOOST_FOREACH(aiNode* nd2, subnodes) {
				nd->mChildren[nd->mNumChildren++] = nd2;
				nd2->mParent = nd.get();
			}
		}
	}
	catch(...) {
		// it hurts, but I don't want to pull boost::ptr_vector into -noboost only for these few spots here
		std::for_each(subnodes.begin(),subnodes.end(),delete_fun<aiNode>());
		throw;
	}

	return nd.release();
}

// ------------------------------------------------------------------------------------------------
void ProcessSpatialStructures(ConversionData& conv)
{
	// process all products in the file. it is reasonable to assume that a
	// file that is relevant for us contains at least a site or a building.
	const STEP::DB::ObjectMapByType& map = conv.db.GetObjectsByType();
	STEP::DB::ObjectMapRange range = map.equal_range("ifcsite");

	if (range.first == map.end()) {
		range = map.equal_range("ifcbuilding");
		if (range.first == map.end()) {
			// no site, no building - try all ids. this will take ages, but it should rarely happen.
			range = STEP::DB::ObjectMapRange(map.begin(),map.end());
		}
	}

	
	for(;range.first != range.second; ++range.first) {
		const IFC::IfcSpatialStructureElement* const prod = (*range.first).second->ToPtr<IFC::IfcSpatialStructureElement>();
		if(!prod) {
			continue;
		}
		IFCImporter::LogDebug("looking at spatial structure `" + (prod->Name ? prod->Name.Get() : "unnamed") + "`" + (prod->ObjectType? " which is of type " + prod->ObjectType.Get():""));
	
		// the primary site is referenced by an IFCRELAGGREGATES element which assigns it to the IFCPRODUCT
		const STEP::DB::RefMap& refs = conv.db.GetRefs();
		STEP::DB::RefMapRange range = refs.equal_range(conv.proj.GetID());
		for(;range.first != range.second; ++range.first) {
			if(const IFC::IfcRelAggregates* const aggr = conv.db.GetObject((*range.first).second)->ToPtr<IFC::IfcRelAggregates>()) {
			
				BOOST_FOREACH(const IFC::IfcObjectDefinition& def, aggr->RelatedObjects) {
					// comparing pointer values is not sufficient, we would need to cast them to the same type first
					// as there is multiple inheritance in the game.
					if (def.GlobalId == prod->GlobalId) { 
						IFCImporter::LogDebug("selecting this spatial structure as root structure");
						// got it, this is the primary site.
						conv.out->mRootNode = ProcessSpatialStructure(NULL,*prod,conv);
						return;
					}
				}

			}
		}
	}


	IFCImporter::ThrowException("Failed to determine primary site element");
}

// ------------------------------------------------------------------------------------------------
void MakeTreeRelative(aiNode* start, const aiMatrix4x4& combined)
{
	// combined is the parent's absolute transformation matrix
	aiMatrix4x4 old = start->mTransformation;

	if (!combined.IsIdentity()) {
		start->mTransformation = aiMatrix4x4(combined).Inverse() * start->mTransformation;
	}

	// All nodes store absolute transformations right now, so we need to make them relative
	for (unsigned int i = 0; i < start->mNumChildren; ++i) {
		MakeTreeRelative(start->mChildren[i],old);
	}
}

// ------------------------------------------------------------------------------------------------
void MakeTreeRelative(ConversionData& conv)
{
	MakeTreeRelative(conv.out->mRootNode,aiMatrix4x4());
}

} // !anon



#endif