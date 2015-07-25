/*
 ---------------------------------------------------------------------------
 Open Asset Import Library (assimp)
 ---------------------------------------------------------------------------

 Copyright (c) 2006-2015, assimp team

 All rights reserved.

 Redistribution and use of this software in source and binary forms,
 with or without modification, are permitted provided that the following
 conditions are met:

 * Redistributions of source code must retain the above
 copyright notice, this list of conditions and the
 following disclaimer.

 * Redistributions in binary form must reproduce the above
 copyright notice, this list of conditions and the
 following disclaimer in the documentation and/or other
 materials provided with the distribution.

 * Neither the name of the assimp team, nor the names of its
 contributors may be used to endorse or promote products
 derived from this software without specific prior
 written permission of the assimp team.

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
 ---------------------------------------------------------------------------
 */

/** @file  MMDLoader.cpp
 *  @brief Implementation of the MMD importer class
 */


#ifndef ASSIMP_BUILD_NO_MMD_IMPORTER

// internal headers
#include "MMDLoader.h"
#include "StringComparison.h"
#include "SGSpatialSort.h"
#include "ByteSwapper.h"
#include "ProcessHelper.h"
#include "ConvertToLHProcess.h"
#include <boost/scoped_ptr.hpp>
#include "../include/assimp/IOSystem.hpp"
#include <sstream>
#include <iomanip>


using namespace Assimp;

#define AI_MMD_FOURCC_PMD "Pmd"
#define AI_MMD_FOURCC_PMX "PMX "
#define AI_MMD_FOURCC_VMD "Vocaloid Motion Data"


static const aiImporterDesc desc = {
    "MMD Importer",
    "",
    "",
    "http://www.geocities.jp/higuchuu4/",
    aiImporterFlags_SupportTextFlavour,
    0,
    0,
    0,
    0,
    "pmd pmx vmd"
};

// ------------------------------------------------------------------------------------------------
// Constructor to be privately used by Importer
MMDLoader::MMDLoader()
    : mIsPMD(),
      mIsPMX() {
}

// ------------------------------------------------------------------------------------------------
// Destructor, private as well
MMDLoader::~MMDLoader() {
}

// ------------------------------------------------------------------------------------------------
// Returns whether the class can handle the format of the given file.
bool MMDLoader::CanRead( const std::string& pFile, IOSystem* pIOHandler, bool checkSig) const {
    const std::string extension = GetExtension(pFile);
    if (extension == "pmd"
            || extension == "pmx"
            || extension == "vmd"
       ) {
        return true;
    }

    // if check for extension is not enough, check for the magic tokens
    if (!extension.length() || checkSig) {
        return CheckMagicToken(pIOHandler, pFile, AI_MMD_FOURCC_PMD,3,0,3)
               || CheckMagicToken(pIOHandler, pFile, AI_MMD_FOURCC_PMX,3,0,4)
               || CheckMagicToken(pIOHandler, pFile, AI_MMD_FOURCC_PMD,3,0,20);
    }
    return false;
}

// ------------------------------------------------------------------------------------------------
// Setup configuration properties
void MMDLoader::SetupProperties(const Importer* pImp) {
}

// ------------------------------------------------------------------------------------------------
// Get list of file extensions
const aiImporterDesc* MMDLoader::GetInfo () const {
    return &desc;
}

// ------------------------------------------------------------------------------------------------
// Imports the given file into the given scene structure.
void MMDLoader::InternReadFile( const std::string& pFile,
                                aiScene* pScene,
                                IOSystem* pIOHandler) {
    boost::scoped_ptr<IOStream> file( pIOHandler->Open( pFile, "rb"));

    // Check whether we can read from the file
    if( file.get() == NULL)
        throw DeadlyImportError( "Failed to open MMD file " + pFile + ".");

    if((unsigned int)file->FileSize() < 20)
        throw DeadlyImportError("MMD: The file is too small to contain the IFF header");

    this->pScene = pScene;

    if (CheckMagicToken(pIOHandler, pFile, AI_MMD_FOURCC_PMD,3,0,3)) {
        auto pmdModel = pmd::PmdModel::LoadFromFile(pFile.c_str());
        CreateDataFromImport(pmdModel, pScene);
    } else if (CheckMagicToken(pIOHandler, pFile, AI_MMD_FOURCC_PMX,3,0,4)) {
        pmx::PmxModel pmxModel;
        std::ifstream stream = std::ifstream(pFile, std::ios_base::binary);
        pmxModel.Read(&stream);
        CreateDataFromImport(&pmxModel, pScene);
    } else if (CheckMagicToken(pIOHandler, pFile, AI_MMD_FOURCC_PMD,3,0,20)) {
        auto vmdModel = vmd::VmdMotion::LoadFromFile(pFile.c_str());
        CreateDataFromImport(vmdModel, pScene);
    }


}


// ------------------------------------------------------------------------------------------------
void MMDLoader::CreateDataFromImport(const pmd::PmdModel* pModel, aiScene* pScene) {
    if( 0L == pModel ) {
        return;
    }

    // Create the root node of the scene
    pScene->mRootNode = new aiNode;
    if ( !pModel->header.name.empty() ) {
        // Set the name of the scene
        pScene->mRootNode->mName.Set(pModel->header.name);
    } else if (!pModel->header.name_english.empty()) {
        pScene->mRootNode->mName.Set(pModel->header.name_english);
    } else {
        // This is a fatal error, so break down the application
        ai_assert(false);
    }

    // Create all materials
    std::vector<aiMaterial> materials;
    createMaterials(pModel, &materials);


    createNodes(pModel, pScene->mRootNode, pScene);
}


void
MMDLoader::createVertices(const pmd::PmdModel* pModel,
                          std::vector<aiVector3D>* positions,
                          std::vector<aiVector3D>* normals,
                          std::vector<aiVector2D>* texCoords) {
    ai_assert(pModel);
    ai_assert(positions);
    ai_assert(normals);
    ai_assert(texCoords);


    for (size_t index = 0; index < pModel->vertices.size(); index++) {
        const auto v = pModel->vertices[index];
        positions->push_back(aiVector3D(v.position[0],v.position[1],v.position[2]));
        normals->push_back(aiVector3D(v.normal[0],v.normal[1],v.normal[2]));
        texCoords->push_back(aiVector2D(v.uv[0], v.uv[1]));
    }

}

void
MMDLoader::createMaterials(const pmd::PmdModel* pModel, std::vector<aiMaterial>* materials) {
    ai_assert(pModel);
    ai_assert(materials);

    for (size_t index = 0; index < pModel->materials.size(); index++) {
        const auto m = pModel->materials[index];
        aiMaterial mat;

        int sm = aiShadingMode_Gouraud;
        mat.AddProperty<int>( &sm, 1, AI_MATKEY_SHADING_MODEL);

        // multiplying the specular exponent with 2 seems to yield better results
        // m.shineness *= 4.f;

        // Adding material colors

        mat.AddProperty( &m.ambient, 1, AI_MATKEY_COLOR_AMBIENT );
        mat.AddProperty( &m.diffuse, 1, AI_MATKEY_COLOR_DIFFUSE );
        mat.AddProperty( &m.specular, 1, AI_MATKEY_COLOR_SPECULAR );
        mat.AddProperty( &m.power, 1, AI_MATKEY_SHININESS );

        // Adding textures
        if ( 0 != m.texture_filename.empty() ) {
            aiString texture(m.texture_filename);
            int clampMode = aiTextureMapMode_Clamp;
            mat.AddProperty(&texture, AI_MATKEY_TEXTURE_DIFFUSE(0));
            mat.AddProperty<int>(&clampMode, 1, AI_MATKEY_MAPPINGMODE_U(aiTextureType_DIFFUSE, 0));
            mat.AddProperty<int>(&clampMode, 1, AI_MATKEY_MAPPINGMODE_V(aiTextureType_DIFFUSE, 0));
        }
    }
}



// ------------------------------------------------------------------------------------------------
//  Creates all nodes of the model
aiNode* MMDLoader::createNodes(const pmd::PmdModel* pModel, aiNode *pParent, aiScene* pScene) {
    ai_assert( NULL != pModel );
    aiNode *pNode = new aiNode;


    pNode->mName = (!pModel->header.name.empty()) ? pModel->header.name
                   : (!pModel->header.name_english.empty()) ? pModel->header.name_english
                   : "";

    if( pParent != NULL ) {
        appendChildToParentNode( pParent, pNode );
    }

    aiMesh *pMesh = createTopology(pModel);

    // allocate one mesh
    pScene->mNumMeshes = 1;
    pScene->mMeshes = new aiMesh*[1];
    pScene->mMeshes[0] = pMesh;
    pMesh->mMaterialIndex = 0;

    // allocate a single node
    pParent->mNumMeshes = 1;
    pParent->mMeshes = new unsigned int[1];
    pParent->mMeshes[0] = 0;

    return pNode;
}


// ------------------------------------------------------------------------------------------------
//  Appends this node to the parent node
void MMDLoader::appendChildToParentNode(aiNode *pParent, aiNode *pChild) {
    // Checking preconditions
    ai_assert( NULL != pParent );
    ai_assert( NULL != pChild );

    // Assign parent to child
    pChild->mParent = pParent;

    // If already children was assigned to the parent node, store them in a
    std::vector<aiNode*> temp;
    if (pParent->mChildren != NULL) {
        ai_assert( 0 != pParent->mNumChildren );
        for (size_t index = 0; index < pParent->mNumChildren; index++) {
            temp.push_back(pParent->mChildren [ index ] );
        }
        delete [] pParent->mChildren;
    }

    // Copy node instances into parent node
    pParent->mNumChildren++;
    pParent->mChildren = new aiNode*[ pParent->mNumChildren ];
    for (size_t index = 0; index < pParent->mNumChildren-1; index++) {
        pParent->mChildren[ index ] = temp [ index ];
    }
    pParent->mChildren[ pParent->mNumChildren-1 ] = pChild;
}



// ------------------------------------------------------------------------------------------------
//  Create topology data
aiMesh*
MMDLoader::createTopology(const pmd::PmdModel* pModel) {
    // Checking preconditions
    ai_assert( NULL != pModel );

    // Create faces
    aiMesh* pMesh = new aiMesh;
    if( !pModel->header.name.empty() ) {
        pMesh->mName.Set( pModel->header.name );
    }

    for (size_t index = 0; index < pModel->faces.size(); index++) {
        const auto inp = &(pModel->faces[ index ]);
        ai_assert( NULL != inp );

        pMesh->mNumFaces++;
        if (inp->vertices.size() == 3) {
            pMesh->mPrimitiveTypes |= aiPrimitiveType_TRIANGLE;
        } else {
            pMesh->mPrimitiveTypes |= aiPrimitiveType_POLYGON;
        }
    }

    unsigned int uiIdxCount( 0u );
    if ( pMesh->mNumFaces > 0 ) {
        pMesh->mFaces = new aiFace[ pMesh->mNumFaces ];

        // Copy all data from all stored meshes
        for (size_t index = 0; index < pModel->faces.size(); index++) {
            const auto inp = &(pModel->faces[ index ]);

            aiFace *pFace = &pMesh->mFaces[ index ];
            const unsigned int uiNumIndices = inp->vertices.size();
            uiIdxCount += pFace->mNumIndices = (unsigned int) uiNumIndices;
            if (pFace->mNumIndices > 0) {
                pFace->mIndices = new unsigned int[ uiNumIndices ];
            }
        }
    }

    // Create mesh vertices
    createVertexArray(pModel, pMesh, uiIdxCount);

    return pMesh;
}


// ------------------------------------------------------------------------------------------------
//  Creates a vertex array
void MMDLoader::createVertexArray(const pmd::PmdModel* pModel,
                                  aiMesh* pMesh,
                                  unsigned int numIndices) {
    // Checking preconditions
    ai_assert( NULL != pModel );
    ai_assert( NULL != pMesh );
    ai_assert( 0 != numIndices );



    // load vertices
    std::vector<aiVector3D> positions;
    std::vector<aiVector3D> normals;
    std::vector<aiVector2D> texCoords;
    createVertices(pModel, &positions, &normals, &texCoords);


    // Copy vertices of this mesh instance
    pMesh->mNumVertices = positions.size();
    pMesh->mVertices = new aiVector3D[ pMesh->mNumVertices ];

    // Allocate buffer for normal vectors
    if (!normals.empty())
        pMesh->mNormals = new aiVector3D[ pMesh->mNumVertices ];

    // Allocate buffer for texture coordinates
    if (!texCoords.empty()) {
        pMesh->mNumUVComponents[ 0 ] = 2;
        pMesh->mTextureCoords[0] = new aiVector3D[ pMesh->mNumVertices ];
    }

    // Copy vertices, normals and textures into aiMesh instance
    unsigned int newIndex = 0, outIndex = 0;
    for ( size_t index=0; index < pModel->faces.size(); index++ ) {
        // Get source face
        auto pFace = &(pModel->faces[ index ]);

        // Copy all index arrays
        for ( size_t vertexIndex = 0, outVertexIndex = 0; vertexIndex < pFace->vertices.size(); vertexIndex++ ) {
            const unsigned int vertex = pFace->vertices.at( vertexIndex ).vertex_index;
            if ( vertex >= pModel->vertices.size() )
                throw DeadlyImportError( "OBJ: vertex index out of range" );

            pMesh->mVertices[ newIndex ] = positions[ vertex ];

            // Copy all normals
            pMesh->mNormals[ newIndex ] = normals[ vertex ];

            // Copy all texture coordinates
            pMesh->mTextureCoords[ 0 ][ newIndex ] = aiVector3D(texCoords[ vertex ][0], texCoords[ vertex ][1], 0.0);

            if ( pMesh->mNumVertices <= newIndex ) {
                throw DeadlyImportError("OBJ: bad vertex index");
            }

            // Get destination face
            aiFace *pDestFace = &pMesh->mFaces[ outIndex ];

            const bool last = ( vertexIndex == pFace->vertices.size() - 1 );
            if (!last) {
                pDestFace->mIndices[ outVertexIndex ] = newIndex;
                outVertexIndex++;
            } else if (last) {
                outIndex++;
            }

            ++newIndex;
        }
    }
}


#endif // !! ASSIMP_BUILD_NO_MMD_IMPORTER
