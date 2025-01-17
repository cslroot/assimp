/*
 Open Asset Import Library (assimp)
 ----------------------------------------------------------------------

 Copyright (c) 2006-2015, assimp team
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

 ----------------------------------------------------------------------
 */

/** @file Declaration of the LWO importer class. */
#ifndef AI_MMDLOADER_H_INCLUDED
#define AI_MMDLOADER_H_INCLUDED

#include "../include/assimp/types.h"
#include "../include/assimp/material.h"
#include "../include/assimp/DefaultLogger.hpp"

#include "MMDHelper.h"
#include "BaseImporter.h"

struct aiTexture;
struct aiNode;
struct aiMaterial;

namespace Assimp    {
using namespace MMD;

// ---------------------------------------------------------------------------
/** Class to load LWO files.
 *
 *  @note  Methods named "xxxLWO2[xxx]" are used with the newer LWO2 format.
 *         Methods named "xxxLWOB[xxx]" are used with the older LWOB format.
 *         Methods named "xxxLWO[xxx]" are used with both formats.
 *         Methods named "xxx" are used to preprocess the loaded data -
 *         they aren't specific to one format version
 */
// ---------------------------------------------------------------------------
class MMDLoader : public BaseImporter {
  public:
    MMDLoader();
    ~MMDLoader();


  public:

    // -------------------------------------------------------------------
    /** Returns whether the class can handle the format of the given file.
     * See BaseImporter::CanRead() for details.
     */
    bool CanRead( const std::string& pFile, IOSystem* pIOHandler,
                  bool checkSig) const;


    // -------------------------------------------------------------------
    /** Called prior to ReadFile().
     * The function is a request to the importer to update its configuration
     * basing on the Importer's configuration property list.
     */
    void SetupProperties(const Importer* pImp);

  protected:

    // -------------------------------------------------------------------
    // Get list of supported extensions
    const aiImporterDesc* GetInfo () const;

    // -------------------------------------------------------------------
    /** Imports the given file into the given scene structure.
     * See BaseImporter::InternReadFile() for details
     */
    void InternReadFile( const std::string& pFile, aiScene* pScene,
                         IOSystem* pIOHandler);

  private:
    void CreateDataFromImport(const pmd::PmdModel* pModel, aiScene* pScene);
    void CreateDataFromImport(const pmx::PmxModel* pModel, aiScene* pScene);
    void CreateDataFromImport(const vmd::VmdMotion* pMotion, aiScene* pScene);

    aiNode* createNodes(const pmd::PmdModel* pModel, aiNode *pParent, aiScene* pScene);
    void appendChildToParentNode(aiNode *pParent, aiNode *pChild);
    void createVertices(const pmd::PmdModel* pModel,
                        std::vector<aiVector3D>* positions,
                        std::vector<aiVector3D>* normals,
                        std::vector<aiVector2D>* texCoords);
    void createMaterials(const pmd::PmdModel* pModel,
                         std::vector<aiMaterial>* materials);
    aiMesh* createTopology(const pmd::PmdModel* pModel);
    void createVertexArray(const pmd::PmdModel* pModel,
                           aiMesh* pMesh,
                           unsigned int numIndices);

  protected:
    /** Output scene */
    aiScene* pScene;

    bool mIsPMD;
    bool mIsPMX;
};
} // end of namespace Assimp

#endif // AI_MMDIMPORTER_H_INCLUDED
