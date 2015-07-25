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
#ifndef OBJ_FILEPARSER_H_INC
#define OBJ_FILEPARSER_H_INC

#include <vector>
#include <string>
#include <map>
#include <assimp/vector2.h>
#include <assimp/vector3.h>
#include <assimp/mesh.h>

#include "MMDFileData.h"

namespace Assimp {
    
    /// \class  MMDParser
    /// \brief  Parser for MMD(Miku Miku Dance) file
    class MMDParser
    {
    public:
        /// \brief  Constructor with data array.
        MMDParser(std::vector<char> &Data,const std::string &strModelName, IOSystem* io);
        /// \brief  Destructor
        ~MMDParser();
        /// \brief  Model getter.
        MMD::pmd::Model *GetModel() const;
        
    private:
        /// Parse the loaded file
        void parseFile();
        
    private:
        // Copy and assignment constructor should be private
        // because the class contains pointer to allocated memory
        MMDParser(const MMDParser& rhs);
        MMDParser& operator=(const MMDParser& rhs);
        
        //! Pointer to model instance
        Model *m_pModel;
        //! Current line (for debugging)
        unsigned int m_uiLine;
        //! Helper buffer
        char m_buffer[BUFFERSIZE];
        /// Pointer to IO system instance.
        IOSystem *m_pIO;
    };
    
}   // Namespace Assimp

#endif
