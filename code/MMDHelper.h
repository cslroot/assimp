/** Helper structures for the MMD loader */

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

#ifndef AI_MMDHELPER_H_INC
#define AI_MMDHELPER_H_INC

#include <string>
#include <map>
#include <vector>
#include <stdint.h>

#include <memory>
#include <iostream>
#include <fstream>


#include "../include/assimp/light.h"
#include "../include/assimp/mesh.h"
#include "../include/assimp/material.h"

struct aiMaterial;

namespace Assimp    {
namespace MMD       {

/** Collada file versions which evolved during the years ... */
enum FormatVersion {
    PMD_1_0,
    PMX_2_0,
    PMX_2_1,
    VMD
};

namespace pmd {
/// ヘッダ
class PmdHeader {
  public:
    /// モデル名
    std::string name;
    /// モデル名(英語)
    std::string name_english;
    /// コメント
    std::string comment;
    /// コメント(英語)
    std::string comment_english;

    bool Read(std::ifstream* stream) {
        char buffer[256];
        stream->read(buffer, 20);
        name = std::string(buffer);
        stream->read(buffer, 256);
        comment = std::string(buffer);
        return true;
    }

    bool ReadExtension(std::ifstream* stream) {
        char buffer[256];
        stream->read(buffer, 20);
        name_english = std::string(buffer);
        stream->read(buffer, 256);
        comment_english = std::string(buffer);
        return true;
    }
};

/// 頂点
class PmdVertex {
  public:
    /// 位置
    float position[3];

    /// 法線
    float normal[3];

    /// UV座標
    float uv[2];

    /// 関連ボーンインデックス
    uint16_t bone_index[2];

    /// ボーンウェイト
    uint8_t bone_weight;

    /// エッジ不可視
    bool edge_invisible;

    bool Read(std::ifstream* stream) {
        stream->read((char*) position, sizeof(float) * 3);
        stream->read((char*) normal, sizeof(float) * 3);
        stream->read((char*) uv, sizeof(float) * 2);
        stream->read((char*) bone_index, sizeof(uint16_t) * 2);
        stream->read((char*) &bone_weight, sizeof(uint8_t));
        stream->read((char*) &edge_invisible, sizeof(uint8_t));
        return true;
    }
};

/// 材質
class PmdMaterial {
  public:
    /// 減衰色
    float diffuse[4];
    /// 光沢度
    float power;
    /// 光沢色
    float specular[3];
    /// 環境色
    float ambient[3];
    /// トーンインデックス
    uint8_t toon_index;
    /// エッジ
    uint8_t edge_flag;
    /// インデックス数
    uint32_t index_count;
    /// テクスチャファイル名
    std::string texture_filename;
    /// スフィアファイル名
    std::string sphere_filename;

    bool Read(std::ifstream* stream) {
        char buffer[20];
        stream->read((char*) &diffuse, sizeof(float) * 4);
        stream->read((char*) &power, sizeof(float));
        stream->read((char*) &specular, sizeof(float) * 3);
        stream->read((char*) &ambient, sizeof(float) * 3);
        stream->read((char*) &toon_index, sizeof(uint8_t));
        stream->read((char*) &edge_flag, sizeof(uint8_t));
        stream->read((char*) &index_count, sizeof(uint32_t));
        stream->read((char*) &buffer, sizeof(char) * 20);
        char* pstar = strchr(buffer, '*');
        if (NULL == pstar) {
            texture_filename = std::string(buffer);
            sphere_filename.clear();
        } else {
            *pstar = NULL;
            texture_filename = std::string(buffer);
            sphere_filename = std::string(pstar+1);
        }
        return true;
    }
};

enum BoneType {
    BoneType_Rotation,
    BoneType_RotationAndMove,
    BoneType_IkEffector,
    BoneType_Unknown,
    BoneType_IkEffectable,
    BoneType_RotationEffectable,
    BoneType_IkTarget,
    BoneType_Invisible,
    BoneType_Twist,
    BoneType_RotationMovement
};

/// ボーン
class PmdBone {
  public:
    /// ボーン名
    std::string name;
    /// ボーン名(英語)
    std::string name_english;
    /// 親ボーン番号
    uint16_t parent_bone_index;
    /// 末端ボーン番号
    uint16_t tail_pos_bone_index;
    /// ボーン種類
    BoneType bone_type;
    /// IKボーン番号
    uint16_t ik_parent_bone_index;
    /// ボーンのヘッドの位置
    float bone_head_pos[3];

    void Read(std::istream *stream) {
        char buffer[20];
        stream->read(buffer, 20);
        name = std::string(buffer);
        stream->read((char*) &parent_bone_index, sizeof(uint16_t));
        stream->read((char*) &tail_pos_bone_index, sizeof(uint16_t));
        stream->read((char*) &bone_type, sizeof(uint8_t));
        stream->read((char*) &ik_parent_bone_index, sizeof(uint16_t));
        stream->read((char*) &bone_head_pos, sizeof(float) * 3);
    }

    void ReadExpantion(std::istream *stream) {
        char buffer[20];
        stream->read(buffer, 20);
        name_english = std::string(buffer);
    }
};

/// IK
class PmdIk {
  public:
    /// IKボーン番号
    uint16_t ik_bone_index;
    /// IKターゲットボーン番号
    uint16_t target_bone_index;
    /// 再帰回数
    uint16_t interations;
    /// 角度制限
    float angle_limit;
    /// 影響下ボーン番号
    std::vector<uint16_t> ik_child_bone_index;

    void Read(std::istream *stream) {
        stream->read((char *) &ik_bone_index, sizeof(uint16_t));
        stream->read((char *) &target_bone_index, sizeof(uint16_t));
        uint8_t ik_chain_length;
        stream->read((char*) &ik_chain_length, sizeof(uint8_t));
        stream->read((char *) &interations, sizeof(uint16_t));
        stream->read((char *) &angle_limit, sizeof(float));
        ik_child_bone_index.resize(ik_chain_length);
        for (int i = 0; i < ik_chain_length; i++) {
            stream->read((char *) &ik_child_bone_index[i], sizeof(uint16_t));
        }
    }
};

class PmdFaceVertex {
  public:
    int vertex_index;
    float position[3];

    void Read(std::istream *stream) {
        stream->read((char *) &vertex_index, sizeof(int));
        stream->read((char *) position, sizeof(float) * 3);
    }
};

enum FaceCategory {
    FaceCategory_Base,
    FaceCategory_Eyebrow,
    FaceCategory_Eye,
    FaceCategory_Mouth,
    FaceCategory_Other
};

class PmdFace {
  public:
    std::string name;
    FaceCategory type;
    std::vector<PmdFaceVertex> vertices;
    std::string name_english;

    void Read(std::istream *stream) {
        char buffer[20];
        stream->read(buffer, 20);
        name = std::string(buffer);
        int vertex_count;
        stream->read((char*) &vertex_count, sizeof(int));
        stream->read((char*) &type, sizeof(uint8_t));
        vertices.resize(vertex_count);
        for (int i = 0; i < vertex_count; i++) {
            vertices[i].Read(stream);
        }
    }

    void ReadExpantion(std::istream *stream) {
        char buffer[20];
        stream->read(buffer, 20);
        name_english = std::string(buffer);
    }
};

/// ボーン枠用の枠名
class PmdBoneDispName {
  public:
    std::string bone_disp_name;
    std::string bone_disp_name_english;

    void Read(std::istream *stream) {
        char buffer[50];
        stream->read(buffer, 50);
        bone_disp_name = std::string(buffer);
        bone_disp_name_english.clear();
    }
    void ReadExpantion(std::istream *stream) {
        char buffer[50];
        stream->read(buffer, 50);
        bone_disp_name_english = std::string(buffer);
    }
};

class PmdBoneDisp {
  public:
    uint16_t bone_index;
    uint8_t bone_disp_index;

    void Read(std::istream *stream) {
        stream->read((char*) &bone_index, sizeof(uint16_t));
        stream->read((char*) &bone_disp_index, sizeof(uint8_t));
    }
};

/// 衝突形状
enum RigidBodyShape {
    /// 球
    RigidBodyShape_Sphere = 0,
    /// 直方体
    RigidBodyShape_Box = 1,
    /// カプセル
    RigidBodyShape_Cupsel = 2
};

/// 剛体タイプ
enum RigidBodyType {
    /// ボーン追従
    RigidBodyType_BoneConnected = 0,
    /// 物理演算
    RigidBodyType_Physics = 1,
    /// 物理演算(Bone位置合せ)
    RigidBodyType_ConnectedPhysics = 2
};

/// 剛体
class PmdRigidBody {
  public:
    /// 名前
    std::string name;
    /// 関連ボーン番号
    uint16_t related_bone_index;
    /// グループ番号
    uint8_t group_index;
    /// マスク
    uint16_t mask;
    /// 形状
    RigidBodyShape shape;
    /// 大きさ
    float size[3];
    /// 位置
    float position[3];
    /// 回転
    float orientation[3];
    /// 質量
    float weight;
    /// 移動ダンピング
    float linear_damping;
    /// 回転ダンピング
    float anglar_damping;
    /// 反発係数
    float restitution;
    /// 摩擦係数
    float friction;
    /// 演算方法
    RigidBodyType rigid_type;

    void Read(std::istream *stream) {
        char buffer[20];
        stream->read(buffer, sizeof(char) * 20);
        name = (std::string(buffer));
        stream->read((char*) &related_bone_index, sizeof(uint16_t));
        stream->read((char*) &group_index, sizeof(uint8_t));
        stream->read((char*) &mask, sizeof(uint16_t));
        stream->read((char*) &shape, sizeof(uint8_t));
        stream->read((char*) size, sizeof(float) * 3);
        stream->read((char*) position, sizeof(float) * 3);
        stream->read((char*) orientation, sizeof(float) * 3);
        stream->read((char*) &weight, sizeof(float));
        stream->read((char*) &linear_damping, sizeof(float));
        stream->read((char*) &anglar_damping, sizeof(float));
        stream->read((char*) &restitution, sizeof(float));
        stream->read((char*) &friction, sizeof(float));
        stream->read((char*) &rigid_type, sizeof(char));
    }
};

/// 剛体の拘束
class PmdConstraint {
  public:
    /// 名前
    std::string name;
    /// 剛体Aのインデックス
    uint32_t rigid_body_index_a;
    /// 剛体Bのインデックス
    uint32_t rigid_body_index_b;
    /// 位置
    float position[3];
    /// 回転
    float orientation[3];
    /// 最小移動制限
    float linear_lower_limit[3];
    /// 最大移動制限
    float linear_upper_limit[3];
    /// 最小回転制限
    float angular_lower_limit[3];
    /// 最大回転制限
    float angular_upper_limit[3];
    /// 移動に対する復元力
    float linear_stiffness[3];
    /// 回転に対する復元力
    float angular_stiffness[3];

    void Read(std::istream *stream) {
        char buffer[20];
        stream->read(buffer, 20);
        name = std::string(buffer);
        stream->read((char *) &rigid_body_index_a, sizeof(uint32_t));
        stream->read((char *) &rigid_body_index_b, sizeof(uint32_t));
        stream->read((char *) position, sizeof(float) * 3);
        stream->read((char *) orientation, sizeof(float) * 3);
        stream->read((char *) linear_lower_limit, sizeof(float) * 3);
        stream->read((char *) linear_upper_limit, sizeof(float) * 3);
        stream->read((char *) angular_lower_limit, sizeof(float) * 3);
        stream->read((char *) angular_upper_limit, sizeof(float) * 3);
        stream->read((char *) linear_stiffness, sizeof(float) * 3);
        stream->read((char *) angular_stiffness, sizeof(float) * 3);
    }
};

/// PMDモデル
class PmdModel {
  public:
    float version;
    PmdHeader header;
    std::vector<PmdVertex> vertices;
    std::vector<uint16_t> indices;
    std::vector<PmdMaterial> materials;
    std::vector<PmdBone> bones;
    std::vector<PmdIk> iks;
    std::vector<PmdFace> faces;
    std::vector<uint16_t> faces_indices;
    std::vector<PmdBoneDispName> bone_disp_name;
    std::vector<PmdBoneDisp> bone_disp;
    std::vector<std::string> toon_filenames;
    std::vector<PmdRigidBody> rigid_bodies;
    std::vector<PmdConstraint> constraints;

    static PmdModel* LoadFromFile(const char *filename) {
        std::ifstream stream(filename, std::ios::binary);
        if (stream.fail()) {
            std::cerr << "could not open \"" << filename << "\"" << std::endl;
            return NULL;
        }
        auto result = LoadFromStream(&stream);
        stream.close();
        return result;
    }

    /// ファイルからPmdModelを生成する
    static PmdModel* LoadFromStream(std::ifstream *stream) {
        auto result = new PmdModel();
        char buffer[100];

        // magic
        char magic[3];
        stream->read(magic, 3);
        if (magic[0] != 'P' || magic[1] != 'm' || magic[2] != 'd') {
            std::cerr << "invalid file" << std::endl;
            return nullptr;
        }

        // version
        stream->read((char*) &(result->version), sizeof(float));
        if (result ->version != 1.0f) {
            std::cerr << "invalid version" << std::endl;
            return nullptr;
        }

        // header
        result->header.Read(stream);


        // vertices
        uint32_t vertex_num;
        stream->read((char*) &vertex_num, sizeof(uint32_t));
        result->vertices.resize(vertex_num);
        for (uint32_t i = 0; i < vertex_num; i++) {
            result->vertices[i].Read(stream);
        }

        // indices
        uint32_t index_num;
        stream->read((char*) &index_num, sizeof(uint32_t));
        result->indices.resize(index_num);
        for (uint32_t i = 0; i < index_num; i++) {
            stream->read((char*) &result->indices[i], sizeof(uint16_t));
        }

        // materials
        uint32_t material_num;
        stream->read((char*) &material_num, sizeof(uint32_t));
        result->materials.resize(material_num);
        for (uint32_t i = 0; i < material_num; i++) {
            result->materials[i].Read(stream);
        }

        // bones
        uint16_t bone_num;
        stream->read((char*) &bone_num, sizeof(uint16_t));
        result->bones.resize(bone_num);
        for (uint32_t i = 0; i < bone_num; i++) {
            result->bones[i].Read(stream);
        }

        // iks
        uint16_t ik_num;
        stream->read((char*) &ik_num, sizeof(uint16_t));
        result->iks.resize(ik_num);
        for (uint32_t i = 0; i < ik_num; i++) {
            result->iks[i].Read(stream);
        }

        // faces
        uint16_t face_num;
        stream->read((char*) &face_num, sizeof(uint16_t));
        result->faces.resize(face_num);
        for (uint32_t i = 0; i < face_num; i++) {
            result->faces[i].Read(stream);
        }

        // face frames
        uint8_t face_frame_num;
        stream->read((char*) &face_frame_num, sizeof(uint8_t));
        result->faces_indices.resize(face_frame_num);
        for (uint32_t i = 0; i < face_frame_num; i++) {
            stream->read((char*) &result->faces_indices[i], sizeof(uint16_t));
        }

        // bone names
        uint8_t bone_disp_num;
        stream->read((char*) &bone_disp_num, sizeof(uint8_t));
        result->bone_disp_name.resize(bone_disp_num);
        for (uint32_t i = 0; i < bone_disp_num; i++) {
            result->bone_disp_name[i].Read(stream);
        }

        // bone frame
        uint32_t bone_frame_num;
        stream->read((char*) &bone_frame_num, sizeof(uint32_t));
        result->bone_disp.resize(bone_frame_num);
        for (uint32_t i = 0; i < bone_frame_num; i++) {
            result->bone_disp[i].Read(stream);
        }

        // english name
        bool english;
        stream->read((char*) &english, sizeof(char));
        if (english) {
            result->header.ReadExtension(stream);
            for (uint32_t i = 0; i < bone_num; i++) {
                result->bones[i].ReadExpantion(stream);
            }
            for (uint32_t i = 0; i < face_num; i++) {
                if (result->faces[i].type == pmd::FaceCategory_Base) {
                    continue;
                }
                result->faces[i].ReadExpantion(stream);
            }
            for (uint32_t i = 0; i < result->bone_disp_name.size(); i++) {
                result->bone_disp_name[i].ReadExpantion(stream);
            }
        }

        // toon textures
        if (stream->peek() == std::ios::traits_type::eof()) {
            result->toon_filenames.clear();
        } else {
            result->toon_filenames.resize(10);
            for (uint32_t i = 0; i < 10; i++) {
                stream->read(buffer, 100);
                result->toon_filenames[i] = std::string(buffer);
            }
        }

        // physics
        if (stream->peek() == std::ios::traits_type::eof()) {
            result->rigid_bodies.clear();
            result->constraints.clear();
        } else {
            uint32_t rigid_body_num;
            stream->read((char*) &rigid_body_num, sizeof(uint32_t));
            result->rigid_bodies.resize(rigid_body_num);
            for (uint32_t i = 0; i < rigid_body_num; i++) {
                result->rigid_bodies[i].Read(stream);
            }
            uint32_t constraint_num;
            stream->read((char*) &constraint_num, sizeof(uint32_t));
            result->constraints.resize(constraint_num);
            for (uint32_t i = 0; i < constraint_num; i++) {
                result->constraints[i].Read(stream);
            }
        }

        if (stream->peek() != std::ios::traits_type::eof()) {
            std::cerr << "there is unknown data" << std::endl;
        }

        return result;
    }
};
}


namespace vmd {
/// ボーンフレーム
class VmdBoneFrame {
  public:
    /// ボーン名
    std::string name;
    /// フレーム番号
    int frame;
    /// 位置
    float position[3];
    /// 回転
    float orientation[4];
    /// 補間曲線
    char interpolation[4][4][4];

    void Read(std::istream* stream) {
        char buffer[15];
        stream->read((char*) buffer, sizeof(char)*15);
        name = std::string(buffer);
        stream->read((char*) &frame, sizeof(int));
        stream->read((char*) position, sizeof(float)*3);
        stream->read((char*) orientation, sizeof(float)*4);
        stream->read((char*) interpolation, sizeof(char) * 4 * 4 * 4);
    }

    void Write(std::ostream* stream) {
        stream->write((char*)name.c_str(), sizeof(char) * 15);
        stream->write((char*)&frame, sizeof(int));
        stream->write((char*)position, sizeof(float) * 3);
        stream->write((char*)orientation, sizeof(float) * 4);
        stream->write((char*)interpolation, sizeof(char) * 4 * 4 * 4);
    }
};

/// 表情フレーム
class VmdFaceFrame {
  public:
    /// 表情名
    std::string face_name;
    /// 表情の重み
    float weight;
    /// フレーム番号
    uint32_t frame;

    void Read(std::istream* stream) {
        char buffer[15];
        stream->read((char*) &buffer, sizeof(char) * 15);
        face_name = std::string(buffer);
        stream->read((char*) &frame, sizeof(int));
        stream->read((char*) &weight, sizeof(float));
    }

    void Write(std::ostream* stream) {
        stream->write((char*)face_name.c_str(), sizeof(char) * 15);
        stream->write((char*)&frame, sizeof(int));
        stream->write((char*)&weight, sizeof(float));
    }
};

/// カメラフレーム
class VmdCameraFrame {
  public:
    /// フレーム番号
    int frame;
    /// 距離
    float distance;
    /// 位置
    float position[3];
    /// 回転
    float orientation[3];
    /// 補間曲線
    char interpolation[6][4];
    /// 視野角
    float angle;
    /// 不明データ
    char unknown[3];

    void Read(std::istream *stream) {
        stream->read((char*) &frame, sizeof(int));
        stream->read((char*) &distance, sizeof(float));
        stream->read((char*) position, sizeof(float) * 3);
        stream->read((char*) orientation, sizeof(float) * 3);
        stream->read((char*) interpolation, sizeof(char) * 24);
        stream->read((char*) &angle, sizeof(float));
        stream->read((char*) unknown, sizeof(char) * 3);
    }

    void Write(std::ostream *stream) {
        stream->write((char*)&frame, sizeof(int));
        stream->write((char*)&distance, sizeof(float));
        stream->write((char*)position, sizeof(float) * 3);
        stream->write((char*)orientation, sizeof(float) * 3);
        stream->write((char*)interpolation, sizeof(char) * 24);
        stream->write((char*)&angle, sizeof(float));
        stream->write((char*)unknown, sizeof(char) * 3);
    }
};

/// ライトフレーム
class VmdLightFrame {
  public:
    /// フレーム番号
    int frame;
    /// 色
    float color[3];
    /// 位置
    float position[3];

    void Read(std::istream* stream) {
        stream->read((char*) &frame, sizeof(int));
        stream->read((char*) color, sizeof(float) * 3);
        stream->read((char*) position, sizeof(float) * 3);
    }

    void Write(std::ostream* stream) {
        stream->write((char*)&frame, sizeof(int));
        stream->write((char*)color, sizeof(float) * 3);
        stream->write((char*)position, sizeof(float) * 3);
    }
};

/// IKの有効無効
class VmdIkEnable {
  public:
    std::string ik_name;
    bool enable;
};

/// IKフレーム
class VmdIkFrame {
  public:
    int frame;
    bool display;
    std::vector<VmdIkEnable> ik_enable;

    void Read(std::istream *stream) {
        char buffer[20];
        stream->read((char*) &frame, sizeof(int));
        stream->read((char*) &display, sizeof(uint8_t));
        int ik_count;
        stream->read((char*) &ik_count, sizeof(int));
        ik_enable.resize(ik_count);
        for (int i = 0; i < ik_count; i++) {
            stream->read(buffer, 20);
            ik_enable[i].ik_name = std::string(buffer);
            stream->read((char*) &ik_enable[i].enable, sizeof(uint8_t));
        }
    }

    void Write(std::ostream *stream) {
        stream->write((char*)&frame, sizeof(int));
        stream->write((char*)&display, sizeof(uint8_t));
        int ik_count = static_cast<int>(ik_enable.size());
        stream->write((char*)&ik_count, sizeof(int));
        for (int i = 0; i < ik_count; i++) {
            const VmdIkEnable& ik_enable = this->ik_enable.at(i);
            stream->write(ik_enable.ik_name.c_str(), 20);
            stream->write((char*)&ik_enable.enable, sizeof(uint8_t));
        }
    }
};

/// VMDモーション
class VmdMotion {
  public:
    /// モデル名
    std::string model_name;
    /// バージョン
    int version;
    /// ボーンフレーム
    std::vector<VmdBoneFrame> bone_frames;
    /// 表情フレーム
    std::vector<VmdFaceFrame> face_frames;
    /// カメラフレーム
    std::vector<VmdCameraFrame> camera_frames;
    /// ライトフレーム
    std::vector<VmdLightFrame> light_frames;
    /// IKフレーム
    std::vector<VmdIkFrame> ik_frames;

    static VmdMotion* LoadFromFile(char const *filename) {
        std::ifstream stream(filename, std::ios::binary);
        auto result = LoadFromStream(&stream);
        stream.close();
        return result;
    }

    static VmdMotion* LoadFromStream(std::ifstream *stream) {

        char buffer[30];
        auto result = new VmdMotion();

        // magic and version
        stream->read((char*) buffer, 30);
        if (strncmp(buffer, "Vocaloid Motion Data", 20)) {
            std::cerr << "invalid vmd file." << std::endl;
            return nullptr;
        }
        result->version = std::atoi(buffer + 20);

        // name
        stream->read(buffer, 20);
        result->model_name = std::string(buffer);

        // bone frames
        int bone_frame_num;
        stream->read((char*) &bone_frame_num, sizeof(int));
        result->bone_frames.resize(bone_frame_num);
        for (int i = 0; i < bone_frame_num; i++) {
            result->bone_frames[i].Read(stream);
        }

        // face frames
        int face_frame_num;
        stream->read((char*) &face_frame_num, sizeof(int));
        result->face_frames.resize(face_frame_num);
        for (int i = 0; i < face_frame_num; i++) {
            result->face_frames[i].Read(stream);
        }

        // camera frames
        int camera_frame_num;
        stream->read((char*) &camera_frame_num, sizeof(int));
        result->camera_frames.resize(camera_frame_num);
        for (int i = 0; i < camera_frame_num; i++) {
            result->camera_frames[i].Read(stream);
        }

        // light frames
        int light_frame_num;
        stream->read((char*) &light_frame_num, sizeof(int));
        result->light_frames.resize(light_frame_num);
        for (int i = 0; i < light_frame_num; i++) {
            result->light_frames[i].Read(stream);
        }

        // unknown2
        stream->read(buffer, 4);

        // ik frames
        if (stream->peek() != std::ios::traits_type::eof()) {
            int ik_num;
            stream->read((char*) &ik_num, sizeof(int));
            result->ik_frames.resize(ik_num);
            for (int i = 0; i < ik_num; i++) {
                result->ik_frames[i].Read(stream);
            }
        }

        if (stream->peek() != std::ios::traits_type::eof()) {
            std::cerr << "vmd stream has unknown data." << std::endl;
        }

        return result;
    }

    bool SaveToFile(const std::string& filename) {
        std::ofstream stream(filename.c_str(), std::ios::binary);
        auto result = SaveToStream(&stream);
        stream.close();
        return result;
    }

    bool SaveToStream(std::ofstream *stream) {
        std::string magic = "Vocaloid Motion Data 0002\0";
        magic.resize(30);

        // magic and version
        stream->write(magic.c_str(), 30);

        // name
        stream->write(model_name.c_str(), 20);

        // bone frames
        const int bone_frame_num = static_cast<int>(bone_frames.size());
        stream->write(reinterpret_cast<const char*>(&bone_frame_num), sizeof(int));
        for (int i = 0; i < bone_frame_num; i++) {
            bone_frames[i].Write(stream);
        }

        // face frames
        const int face_frame_num = static_cast<int>(face_frames.size());
        stream->write(reinterpret_cast<const char*>(&face_frame_num), sizeof(int));
        for (int i = 0; i < face_frame_num; i++) {
            face_frames[i].Write(stream);
        }

        // camera frames
        const int camera_frame_num = static_cast<int>(camera_frames.size());
        stream->write(reinterpret_cast<const char*>(&camera_frame_num), sizeof(int));
        for (int i = 0; i < camera_frame_num; i++) {
            camera_frames[i].Write(stream);
        }

        // light frames
        const int light_frame_num = static_cast<int>(light_frames.size());
        stream->write(reinterpret_cast<const char*>(&light_frame_num), sizeof(int));
        for (int i = 0; i < light_frame_num; i++) {
            light_frames[i].Write(stream);
        }

        // self shadow datas
        const int self_shadow_num = 0;
        stream->write(reinterpret_cast<const char*>(&self_shadow_num), sizeof(int));

        // ik frames
        const int ik_num = static_cast<int>(ik_frames.size());
        stream->write(reinterpret_cast<const char*>(&ik_num), sizeof(int));
        for (int i = 0; i < ik_num; i++) {
            ik_frames[i].Write(stream);
        }

        return true;
    }
};
}

namespace pmx {
/// インデックス設定
class PmxSetting {
  public:
    PmxSetting()
        : encoding(0)
        , uv(0)
        , vertex_index_size(0)
        , texture_index_size(0)
        , material_index_size(0)
        , bone_index_size(0)
        , morph_index_size(0)
        , rigidbody_index_size(0) {
    }

    /// エンコード方式
    uint8_t encoding;
    /// 追加UV数
    uint8_t uv;
    /// 頂点インデックスサイズ
    uint8_t vertex_index_size;
    /// テクスチャインデックスサイズ
    uint8_t texture_index_size;
    /// マテリアルインデックスサイズ
    uint8_t material_index_size;
    /// ボーンインデックスサイズ
    uint8_t bone_index_size;
    /// モーフインデックスサイズ
    uint8_t morph_index_size;
    /// 剛体インデックスサイズ
    uint8_t rigidbody_index_size;
    void Read(std::istream *stream);
};

/// 頂点スキニングタイプ
enum PmxVertexSkinningType {
    PmxVertexSkinningType_BDEF1 = 0,
    PmxVertexSkinningType_BDEF2 = 1,
    PmxVertexSkinningType_BDEF4 = 2,
    PmxVertexSkinningType_SDEF = 3,
    PmxVertexSkinningType_QDEF = 4,
};

/// 頂点スキニング
class PmxVertexSkinning {
  public:
    virtual void Read(std::istream *stream, PmxSetting *setting) = 0;
};

class PmxVertexSkinningBDEF1 : public PmxVertexSkinning {
  public:
    PmxVertexSkinningBDEF1()
        : bone_index(0) {
    }

    int bone_index;
    void Read(std::istream *stresam, PmxSetting *setting);
};

class PmxVertexSkinningBDEF2 : public PmxVertexSkinning {
  public:
    PmxVertexSkinningBDEF2()
        : bone_index1(0)
        , bone_index2(0)
        , bone_weight(0.0f) {
    }

    int bone_index1;
    int bone_index2;
    float bone_weight;
    void Read(std::istream *stresam, PmxSetting *setting);
};

class PmxVertexSkinningBDEF4 : public PmxVertexSkinning {
  public:
    PmxVertexSkinningBDEF4()
        : bone_index1(0)
        , bone_index2(0)
        , bone_index3(0)
        , bone_index4(0)
        , bone_weight1(0.0f)
        , bone_weight2(0.0f)
        , bone_weight3(0.0f)
        , bone_weight4(0.0f) {
    }

    int bone_index1;
    int bone_index2;
    int bone_index3;
    int bone_index4;
    float bone_weight1;
    float bone_weight2;
    float bone_weight3;
    float bone_weight4;
    void Read(std::istream *stresam, PmxSetting *setting);
};

class PmxVertexSkinningSDEF : public PmxVertexSkinning {
  public:
    PmxVertexSkinningSDEF()
        : bone_index1(0)
        , bone_index2(0)
        , bone_weight(0.0f) {
        for (int i = 0; i < 3; ++i) {
            sdef_c[i] = 0.0f;
            sdef_r0[i] = 0.0f;
            sdef_r1[i] = 0.0f;
        }
    }

    int bone_index1;
    int bone_index2;
    float bone_weight;
    float sdef_c[3];
    float sdef_r0[3];
    float sdef_r1[3];
    void Read(std::istream *stresam, PmxSetting *setting);
};

class PmxVertexSkinningQDEF : public PmxVertexSkinning {
  public:
    PmxVertexSkinningQDEF()
        : bone_index1(0)
        , bone_index2(0)
        , bone_index3(0)
        , bone_index4(0)
        , bone_weight1(0.0f)
        , bone_weight2(0.0f)
        , bone_weight3(0.0f)
        , bone_weight4(0.0f) {
    }

    int bone_index1;
    int bone_index2;
    int bone_index3;
    int bone_index4;
    float bone_weight1;
    float bone_weight2;
    float bone_weight3;
    float bone_weight4;
    void Read(std::istream *stresam, PmxSetting *setting);
};

/// 頂点
class PmxVertex {
  public:
    PmxVertex()
        : edge(0.0f) {
        uv[0] = uv[1] = 0.0f;
        for (int i = 0; i < 3; ++i) {
            positon[i] = 0.0f;
            normal[i] = 0.0f;
        }
        for (int i = 0; i < 4; ++i) {
            for (int k = 0; k < 4; ++k) {
                uva[i][k] = 0.0f;
            }
        }
    }

    /// 位置
    float positon[3];
    /// 法線
    float normal[3];
    /// テクスチャ座標
    float uv[2];
    /// 追加テクスチャ座標
    float uva[4][4];
    /// スキニングタイプ
    PmxVertexSkinningType skinning_type;
    /// スキニング
    PmxVertexSkinning* skinning;
    /// エッジ倍率
    float edge;
    void Read(std::istream *stream, PmxSetting *setting);
};

/// マテリアル
class PmxMaterial {
  public:
    PmxMaterial()
        : specularlity(0.0f)
        , flag(0)
        , edge_size(0.0f)
        , diffuse_texture_index(0)
        , sphere_texture_index(0)
        , sphere_op_mode(0)
        , common_toon_flag(0)
        , toon_texture_index(0)
        , index_count(0) {
        for (int i = 0; i < 3; ++i) {
            specular[i] = 0.0f;
            ambient[i] = 0.0f;
            edge_color[i] = 0.0f;
        }
        for (int i = 0; i < 4; ++i) {
            diffuse[i] = 0.0f;
        }
    }

    /// モデル名
    std::string material_name;
    /// モデル英名
    std::string material_english_name;
    /// 減衰色
    float diffuse[4];
    /// 光沢色
    float specular[3];
    /// 光沢度
    float specularlity;
    /// 環境色
    float ambient[3];
    /// 描画フラグ
    uint8_t flag;
    /// エッジ色
    float edge_color[4];
    /// エッジサイズ
    float edge_size;
    /// アルベドテクスチャインデックス
    int diffuse_texture_index;
    /// スフィアテクスチャインデックス
    int sphere_texture_index;
    /// スフィアテクスチャ演算モード
    uint8_t sphere_op_mode;
    /// 共有トゥーンフラグ
    uint8_t common_toon_flag;
    /// トゥーンテクスチャインデックス
    int toon_texture_index;
    /// メモ
    std::string memo;
    /// 頂点インデックス数
    int index_count;
    void Read(std::istream *stream, PmxSetting *setting);
};

/// リンク
class PmxIkLink {
  public:
    PmxIkLink()
        : link_target(0)
        , angle_lock(0) {
        for (int i = 0; i < 3; ++i) {
            max_radian[i] = 0.0f;
            min_radian[i] = 0.0f;
        }
    }

    /// リンクボーンインデックス
    int link_target;
    /// 角度制限
    uint8_t angle_lock;
    /// 最大制限角度
    float max_radian[3];
    /// 最小制限角度
    float min_radian[3];
    void Read(std::istream *stream, PmxSetting *settingn);
};

/// ボーン
class PmxBone {
  public:
    PmxBone()
        : parent_index(0)
        , level(0)
        , bone_flag(0)
        , target_index(0)
        , grant_parent_index(0)
        , grant_weight(0.0f)
        , key(0)
        , ik_target_bone_index(0)
        , ik_loop(0)
        , ik_loop_angle_limit(0.0f)
        , ik_link_count(0) {
        for (int i = 0; i < 3; ++i) {
            position[i] = 0.0f;
            offset[i] = 0.0f;
            lock_axis_orientation[i] = 0.0f;
            local_axis_x_orientation[i] = 0.0f;
            local_axis_y_orientation[i] = 0.0f;
        }
    }

    /// ボーン名
    std::string bone_name;
    /// ボーン英名
    std::string bone_english_name;
    /// 位置
    float position[3];
    /// 親ボーンインデックス
    int parent_index;
    /// 階層
    int level;
    /// ボーンフラグ
    uint16_t bone_flag;
    /// 座標オフセット(has Target)
    float offset[3];
    /// 接続先ボーンインデックス(not has Target)
    int target_index;
    /// 付与親ボーンインデックス
    int grant_parent_index;
    /// 付与率
    float grant_weight;
    /// 固定軸の方向
    float lock_axis_orientation[3];
    /// ローカル軸のX軸方向
    float local_axis_x_orientation[3];
    /// ローカル軸のY軸方向
    float local_axis_y_orientation[3];
    /// 外部親変形のkey値
    int key;
    /// IKターゲットボーン
    int ik_target_bone_index;
    /// IKループ回数
    int ik_loop;
    /// IKループ計算時の角度制限(ラジアン)
    float ik_loop_angle_limit;
    /// IKリンク数
    int ik_link_count;
    /// IKリンク
    PmxIkLink* ik_links;
    void Read(std::istream *stream, PmxSetting *setting);
};

enum MorphType {
    MorphType_Group = 0,
    MorphType_Vertex = 1,
    MorphType_Bone = 2,
    MorphType_UV = 3,
    MorphType_AdditionalUV1 = 4,
    MorphType_AdditionalUV2 = 5,
    MorphType_AdditionalUV3 = 6,
    MorphType_AdditionalUV4 = 7,
    MorphType_Matrial = 8,
    MorphType_Flip = 9,
    MorphType_Implus = 10,
};

enum MorphCategory {
    MorphCategory_ReservedCategory = 0,
    MorphCategory_Eyebrow = 1,
    MorphCategory_Eye = 2,
    MorphCategory_Mouth = 3,
    MorphCategory_Other = 4,
};

class PmxMorphOffset {
  public:
    void virtual Read(std::istream *stream, PmxSetting *setting) = 0;
};

class PmxMorphVertexOffset : public PmxMorphOffset {
  public:
    PmxMorphVertexOffset()
        : vertex_index(0) {
        for (int i = 0; i < 3; ++i) {
            position_offset[i] = 0.0f;
        }
    }
    int vertex_index;
    float position_offset[3];
    void Read(std::istream *stream, PmxSetting *setting) override;
};

class PmxMorphUVOffset : public PmxMorphOffset {
  public:
    PmxMorphUVOffset()
        : vertex_index(0) {
        for (int i = 0; i < 4; ++i) {
            uv_offset[i] = 0.0f;
        }
    }
    int vertex_index;
    float uv_offset[4];
    void Read(std::istream *stream, PmxSetting *setting) override;
};

class PmxMorphBoneOffset : public PmxMorphOffset {
  public:
    PmxMorphBoneOffset()
        : bone_index(0) {
        for (int i = 0; i < 3; ++i) {
            translation[i] = 0.0f;
        }
        for (int i = 0; i < 4; ++i) {
            rotation[i] = 0.0f;
        }
    }
    int bone_index;
    float translation[3];
    float rotation[4];
    void Read(std::istream *stream, PmxSetting *setting) override;
};

class PmxMorphMaterialOffset : public PmxMorphOffset {
  public:
    PmxMorphMaterialOffset()
        : specularity(0.0f)
        , edge_size(0.0f) {
        for (int i = 0; i < 3; ++i) {
            specular[i] = 0.0f;
            ambient[i] = 0.0f;
        }
        for (int i = 0; i < 4; ++i) {
            diffuse[i] = 0.0f;
            edge_color[i] = 0.0f;
            texture_argb[i] = 0.0f;
            sphere_texture_argb[i] = 0.0f;
            toon_texture_argb[i] = 0.0f;
        }
    }
    int material_index;
    uint8_t offset_operation;
    float diffuse[4];
    float specular[3];
    float specularity;
    float ambient[3];
    float edge_color[4];
    float edge_size;
    float texture_argb[4];
    float sphere_texture_argb[4];
    float toon_texture_argb[4];
    void Read(std::istream *stream, PmxSetting *setting) override;
};

class PmxMorphGroupOffset : public PmxMorphOffset {
  public:
    PmxMorphGroupOffset()
        : morph_index(0)
        , morph_weight(0.0f) {
    }
    int morph_index;
    float morph_weight;
    void Read(std::istream *stream, PmxSetting *setting) override;
};

class PmxMorphFlipOffset : public PmxMorphOffset {
  public:
    PmxMorphFlipOffset()
        : morph_index(0)
        , morph_value(0.0f) {
    }
    int morph_index;
    float morph_value;
    void Read(std::istream *stream, PmxSetting *setting) override;
};

class PmxMorphImplusOffset : public PmxMorphOffset {
  public:
    PmxMorphImplusOffset()
        : rigid_body_index(0)
        , is_local(0) {
        for (int i = 0; i < 3; ++i) {
            velocity[i] = 0.0f;
            angular_torque[i] = 0.0f;
        }
    }
    int rigid_body_index;
    uint8_t is_local;
    float velocity[3];
    float angular_torque[3];
    void Read(std::istream *stream, PmxSetting *setting) override;
};

/// モーフ
class PmxMorph {
  public:
    PmxMorph()
        : offset_count(0) {
    }
    /// モーフ名
    std::string morph_name;
    /// モーフ英名
    std::string morph_english_name;
    /// カテゴリ
    MorphCategory category;
    /// モーフタイプ
    MorphType morph_type;
    /// オフセット数
    int offset_count;
    /// 頂点モーフ配列
    PmxMorphVertexOffset* vertex_offsets;
    /// UVモーフ配列
    PmxMorphUVOffset* uv_offsets;
    /// ボーンモーフ配列
    PmxMorphBoneOffset* bone_offsets;
    /// マテリアルモーフ配列
    PmxMorphMaterialOffset* material_offsets;
    /// グループモーフ配列
    PmxMorphGroupOffset* group_offsets;
    /// フリップモーフ配列
    PmxMorphFlipOffset* flip_offsets;
    /// インパルスモーフ配列
    PmxMorphImplusOffset* implus_offsets;
    void Read(std::istream *stream, PmxSetting *setting);
};

/// 枠内要素
class PmxFrameElement {
  public:
    PmxFrameElement()
        : element_target(0)
        , index(0) {
    }
    /// 要素対象
    uint8_t element_target;
    /// 要素対象インデックス
    int index;
    void Read(std::istream *stream, PmxSetting *setting);
};

/// 表示枠
class PmxFrame {
  public:
    PmxFrame()
        : frame_flag(0)
        , element_count(0) {
    }
    /// 枠名
    std::string frame_name;
    /// 枠英名
    std::string frame_english_name;
    /// 特殊枠フラグ
    uint8_t frame_flag;
    /// 枠内要素数
    int element_count;
    /// 枠内要素配列
    std::vector<PmxFrameElement> elements;
    void Read(std::istream *stream, PmxSetting *setting);
};

class PmxRigidBody {
  public:
    PmxRigidBody()
        : target_bone(0)
        , group(0)
        , mask(0)
        , shape(0)
        , mass(0.0f)
        , move_attenuation(0.0f)
        , rotation_attenuation(0.0f)
        , repulsion(0.0f)
        , friction(0.0f)
        , physics_calc_type(0) {
        for (int i = 0; i < 3; ++i) {
            size[i] = 0.0f;
            position[i] = 0.0f;
            orientation[i] = 0.0f;
        }
    }
    /// 剛体名
    std::string girid_body_name;
    /// 剛体英名
    std::string girid_body_english_name;
    /// 関連ボーンインデックス
    int target_bone;
    /// グループ
    uint8_t group;
    /// マスク
    uint16_t mask;
    /// 形状
    uint8_t shape;
    float size[3];
    float position[3];
    float orientation[3];
    float mass;
    float move_attenuation;
    float rotation_attenuation;
    float repulsion;
    float friction;
    uint8_t physics_calc_type;
    void Read(std::istream *stream, PmxSetting *setting);
};

enum PmxJointType {
    PmxJointType_Generic6DofSpring = 0,
    PmxJointType_Generic6Dof = 1,
    PmxJointType_Point2Point = 2,
    PmxJointType_ConeTwist = 3,
    PmxJointType_Slider = 5,
    PmxJointType_Hinge = 6
};

class PmxJointParam {
  public:
    PmxJointParam()
        : rigid_body1(0)
        , rigid_body2(0) {
        for (int i = 0; i < 3; ++i) {
            position[i] = 0.0f;
            orientaiton[i] = 0.0f;
            move_limitation_min[i] = 0.0f;
            move_limitation_max[i] = 0.0f;
            rotation_limitation_min[i] = 0.0f;
            rotation_limitation_max[i] = 0.0f;
            spring_move_coefficient[i] = 0.0f;
            spring_rotation_coefficient[i] = 0.0f;
        }
    }
    int rigid_body1;
    int rigid_body2;
    float position[3];
    float orientaiton[3];
    float move_limitation_min[3];
    float move_limitation_max[3];
    float rotation_limitation_min[3];
    float rotation_limitation_max[3];
    float spring_move_coefficient[3];
    float spring_rotation_coefficient[3];
    void Read(std::istream *stream, PmxSetting *setting);
};

class PmxJoint {
  public:
    std::string joint_name;
    std::string joint_english_name;
    PmxJointType joint_type;
    PmxJointParam param;
    void Read(std::istream *stream, PmxSetting *setting);
};

enum PmxSoftBodyFlag {
    BLink = 0x01,
    Cluster = 0x02,
    Link = 0x04
};

class PmxAncherRigidBody {
  public:
    PmxAncherRigidBody()
        : related_rigid_body(0)
        , related_vertex(0)
        , is_near(false) {
    }
    int related_rigid_body;
    int related_vertex;
    bool is_near;
    void Read(std::istream *stream, PmxSetting *setting);
};

class PmxSoftBody {
  public:
    PmxSoftBody()
        : shape(0)
        , target_material(0)
        , group(0)
        , mask(0)
        , blink_distance(0)
        , cluster_count(0)
        , mass(0.0)
        , collisioni_margin(0.0)
        , aero_model(0)
        , VCF(0.0f)
        , DP(0.0f)
        , DG(0.0f)
        , LF(0.0f)
        , PR(0.0f)
        , VC(0.0f)
        , DF(0.0f)
        , MT(0.0f)
        , CHR(0.0f)
        , KHR(0.0f)
        , SHR(0.0f)
        , AHR(0.0f)
        , SRHR_CL(0.0f)
        , SKHR_CL(0.0f)
        , SSHR_CL(0.0f)
        , SR_SPLT_CL(0.0f)
        , SK_SPLT_CL(0.0f)
        , SS_SPLT_CL(0.0f)
        , V_IT(0)
        , P_IT(0)
        , D_IT(0)
        , C_IT(0)
        , LST(0.0f)
        , AST(0.0f)
        , VST(0.0f)
        , anchor_count(0)
        , pin_vertex_count(0) {
    }
    std::string soft_body_name;
    std::string soft_body_english_name;
    uint8_t shape;
    int target_material;
    uint8_t group;
    uint16_t mask;
    PmxSoftBodyFlag flag;
    int blink_distance;
    int cluster_count;
    float mass;
    float collisioni_margin;
    int aero_model;
    float VCF;
    float DP;
    float DG;
    float LF;
    float PR;
    float VC;
    float DF;
    float MT;
    float CHR;
    float KHR;
    float SHR;
    float AHR;
    float SRHR_CL;
    float SKHR_CL;
    float SSHR_CL;
    float SR_SPLT_CL;
    float SK_SPLT_CL;
    float SS_SPLT_CL;
    int V_IT;
    int P_IT;
    int D_IT;
    int C_IT;
    float LST;
    float AST;
    float VST;
    int anchor_count;
    PmxAncherRigidBody** anchers;
    int pin_vertex_count;
    int** pin_vertices;
    void Read(std::istream *stream, PmxSetting *setting);
};

/// PMXモデル
class PmxModel {
  public:
    PmxModel()
        : version(0.0f)
        , vertex_count(0)
        , index_count(0)
        , texture_count(0)
        , material_count(0)
        , bone_count(0)
        , morph_count(0)
        , frame_count(0)
        , rigid_body_count(0)
        , joint_count(0)
        , soft_body_count(0) {
    }

    /// バージョン
    float version;
    /// 設定
    PmxSetting setting;
    /// モデル名
    std::string model_name;
    /// モデル英名
    std::string model_english_name;
    /// コメント
    std::string model_comment;
    /// 英語コメント
    std::string model_english_commnet;
    /// 頂点数
    int vertex_count;
    /// 頂点配列
    std::vector<PmxVertex> vertices;
    /// インデックス数
    int index_count;
    /// インデックス配列
    std::vector<int> indices;
    /// テクスチャ数
    int texture_count;
    /// テクスチャ配列
    std::vector<std::string> textures;
    /// マテリアル数
    int material_count;
    /// マテリアル
    std::vector<PmxMaterial> materials;
    /// ボーン数
    int bone_count;
    /// ボーン配列
    std::vector<PmxBone> bones;
    /// モーフ数
    int morph_count;
    /// モーフ配列
    std::vector<PmxMorph> morphs;
    /// 表示枠数
    int frame_count;
    /// 表示枠配列
    std::vector<PmxFrame> frames;
    /// 剛体数
    int rigid_body_count;
    /// 剛体配列
    std::vector<PmxRigidBody> rigid_bodies;
    /// ジョイント数
    int joint_count;
    /// ジョイント配列
    std::vector<PmxJoint> joints;
    /// ソフトボディ数
    int soft_body_count;
    /// ソフトボディ配列
    std::vector<PmxSoftBody> soft_bodies;
    /// モデル初期化
    void Init();
    /// モデル読み込み
    void Read(std::istream *stream);
    ///// ファイルからモデルの読み込み
    //static std::unique_ptr<PmxModel> ReadFromFile(const char *filename);
    ///// 入力ストリームからモデルの読み込み
    //static std::unique_ptr<PmxModel> ReadFromStream(std::istream *stream);
};

/// インデックス値を読み込む
int ReadIndex(std::istream *stream, int size) {
    switch (size) {
    case 1:
        uint8_t tmp8;
        stream->read((char*) &tmp8, sizeof(uint8_t));
        if (255 == tmp8) {
            return -1;
        } else {
            return (int) tmp8;
        }
    case 2:
        uint16_t tmp16;
        stream->read((char*) &tmp16, sizeof(uint16_t));
        if (65535 == tmp16) {
            return -1;
        } else {
            return (int) tmp16;
        }
    case 4:
        int tmp32;
        stream->read((char*) &tmp32, sizeof(int));
        return tmp32;
    default:
        return -1;
    }
}

/// 文字列を読み込む
// TODO: 文字コード対応
std::string ReadString(std::istream *stream, uint8_t endcoding) {
    int size;
    stream->read((char*) &size, sizeof(int));
    std::vector<char> buffer;
    if (size == 0) {
        return std::string("");
    }
    buffer.resize(size);
    stream->read((char*) buffer.data(), size);

    return std::string(buffer.data());
}

void PmxSetting::Read(std::istream *stream) {
    uint8_t count;
    stream->read((char*) &count, sizeof(uint8_t));
    if (count < 8) {
        throw;
    }
    stream->read((char*) &encoding, sizeof(uint8_t));
    stream->read((char*) &uv, sizeof(uint8_t));
    stream->read((char*) &vertex_index_size, sizeof(uint8_t));
    stream->read((char*) &texture_index_size, sizeof(uint8_t));
    stream->read((char*) &material_index_size, sizeof(uint8_t));
    stream->read((char*) &bone_index_size, sizeof(uint8_t));
    stream->read((char*) &morph_index_size, sizeof(uint8_t));
    stream->read((char*) &rigidbody_index_size, sizeof(uint8_t));
    uint8_t temp;
    for (int i = 8; i < count; i++) {
        stream->read((char*)&temp, sizeof(uint8_t));
    }
}

void PmxVertexSkinningBDEF1::Read(std::istream *stream, PmxSetting *setting) {
    this->bone_index = ReadIndex(stream, setting->bone_index_size);
}

void PmxVertexSkinningBDEF2::Read(std::istream *stream, PmxSetting *setting) {
    this->bone_index1 = ReadIndex(stream, setting->bone_index_size);
    this->bone_index2 = ReadIndex(stream, setting->bone_index_size);
    stream->read((char*) &this->bone_weight, sizeof(float));
}

void PmxVertexSkinningBDEF4::Read(std::istream *stream, PmxSetting *setting) {
    this->bone_index1 = ReadIndex(stream, setting->bone_index_size);
    this->bone_index2 = ReadIndex(stream, setting->bone_index_size);
    this->bone_index3 = ReadIndex(stream, setting->bone_index_size);
    this->bone_index4 = ReadIndex(stream, setting->bone_index_size);
    stream->read((char*) &this->bone_weight1, sizeof(float));
    stream->read((char*) &this->bone_weight2, sizeof(float));
    stream->read((char*) &this->bone_weight3, sizeof(float));
    stream->read((char*) &this->bone_weight4, sizeof(float));
}

void PmxVertexSkinningSDEF::Read(std::istream *stream, PmxSetting *setting) {
    this->bone_index1 = ReadIndex(stream, setting->bone_index_size);
    this->bone_index2 = ReadIndex(stream, setting->bone_index_size);
    stream->read((char*) &this->bone_weight, sizeof(float));
    stream->read((char*) this->sdef_c, sizeof(float) * 3);
    stream->read((char*) this->sdef_r0, sizeof(float) * 3);
    stream->read((char*) this->sdef_r1, sizeof(float) * 3);
}

void PmxVertexSkinningQDEF::Read(std::istream *stream, PmxSetting *setting) {
    this->bone_index1 = ReadIndex(stream, setting->bone_index_size);
    this->bone_index2 = ReadIndex(stream, setting->bone_index_size);
    this->bone_index3 = ReadIndex(stream, setting->bone_index_size);
    this->bone_index4 = ReadIndex(stream, setting->bone_index_size);
    stream->read((char*) &this->bone_weight1, sizeof(float));
    stream->read((char*) &this->bone_weight2, sizeof(float));
    stream->read((char*) &this->bone_weight3, sizeof(float));
    stream->read((char*) &this->bone_weight4, sizeof(float));
}

void PmxVertex::Read(std::istream *stream, PmxSetting *setting) {
    stream->read((char*) this->positon, sizeof(float) * 3);
    stream->read((char*) this->normal, sizeof(float) * 3);
    stream->read((char*) this->uv, sizeof(float) * 2);
    for (int i = 0; i < setting->uv; ++i) {
        stream->read((char*) this->uva[i], sizeof(float) * 4);
    }
    stream->read((char*) &this->skinning_type, sizeof(PmxVertexSkinningType));
    switch (this->skinning_type) {
    case PmxVertexSkinningType_BDEF1:
        this->skinning = new PmxVertexSkinningBDEF1();
        break;
    case PmxVertexSkinningType_BDEF2:
        this->skinning = new PmxVertexSkinningBDEF2();
        break;
    case PmxVertexSkinningType_BDEF4:
        this->skinning = new PmxVertexSkinningBDEF4();
        break;
    case PmxVertexSkinningType_SDEF:
        this->skinning = new PmxVertexSkinningSDEF();
        break;
    case PmxVertexSkinningType_QDEF:
        this->skinning = new PmxVertexSkinningQDEF();
        break;
    default:
        throw "invalid skinning type";
    }
    this->skinning->Read(stream, setting);
    stream->read((char*) &this->edge, sizeof(float));
}

void PmxMaterial::Read(std::istream *stream, PmxSetting *setting) {
    this->material_name = ReadString(stream, setting->encoding);
    this->material_english_name = ReadString(stream, setting->encoding);
    stream->read((char*) this->diffuse, sizeof(float) * 4);
    stream->read((char*) this->specular, sizeof(float) * 3);
    stream->read((char*) &this->specularlity, sizeof(float));
    stream->read((char*) this->ambient, sizeof(float) * 3);
    stream->read((char*) &this->flag, sizeof(uint8_t));
    stream->read((char*) this->edge_color, sizeof(float) * 4);
    stream->read((char*) &this->edge_size, sizeof(float));
    this->diffuse_texture_index = ReadIndex(stream, setting->texture_index_size);
    this->sphere_texture_index = ReadIndex(stream, setting->texture_index_size);
    stream->read((char*) &this->sphere_op_mode, sizeof(uint8_t));
    stream->read((char*) &this->common_toon_flag, sizeof(uint8_t));
    if (this->common_toon_flag) {
        stream->read((char*) &this->toon_texture_index, sizeof(uint8_t));
    } else {
        this->toon_texture_index = ReadIndex(stream, setting->texture_index_size);
    }
    this->memo = ReadString(stream, setting->encoding);
    stream->read((char*) &this->index_count, sizeof(int));
}

void PmxIkLink::Read(std::istream *stream, PmxSetting *setting) {
    this->link_target = ReadIndex(stream, setting->bone_index_size);
    stream->read((char*) &this->angle_lock, sizeof(uint8_t));
    if (angle_lock == 1) {
        stream->read((char*) this->max_radian, sizeof(float) * 3);
        stream->read((char*) this->min_radian, sizeof(float) * 3);
    }
}

void PmxBone::Read(std::istream *stream, PmxSetting *setting) {
    this->bone_name = ReadString(stream, setting->encoding);
    this->bone_english_name = ReadString(stream, setting->encoding);
    stream->read((char*) this->position, sizeof(float) * 3);
    this->parent_index = ReadIndex(stream, setting->bone_index_size);
    stream->read((char*) &this->level, sizeof(int));
    stream->read((char*) &this->bone_flag, sizeof(uint16_t));
    if (this->bone_flag & 0x0001) {
        this->target_index = ReadIndex(stream, setting->bone_index_size);
    } else {
        stream->read((char*)this->offset, sizeof(float) * 3);
    }
    if (this->bone_flag & (0x0100 | 0x0200)) {
        this->grant_parent_index = ReadIndex(stream, setting->bone_index_size);
        stream->read((char*) &this->grant_weight, sizeof(float));
    }
    if (this->bone_flag & 0x0400) {
        stream->read((char*)this->lock_axis_orientation, sizeof(float) * 3);
    }
    if (this->bone_flag & 0x0800) {
        stream->read((char*)this->local_axis_x_orientation, sizeof(float) * 3);
        stream->read((char*)this->local_axis_y_orientation, sizeof(float) * 3);
    }
    if (this->bone_flag & 0x2000) {
        stream->read((char*)this->key, sizeof(int));
    }
    if (this->bone_flag & 0x0020) {
        this->ik_target_bone_index = ReadIndex(stream, setting->bone_index_size);
        stream->read((char*) &ik_loop, sizeof(int));
        stream->read((char*) &ik_loop_angle_limit, sizeof(float));
        stream->read((char*) &ik_link_count, sizeof(int));
        this->ik_links = new PmxIkLink[ik_link_count];
        for (int i = 0; i < ik_link_count; i++) {
            ik_links[i].Read(stream, setting);
        }
    }
}

void PmxMorphVertexOffset::Read(std::istream *stream, PmxSetting *setting) {
    this->vertex_index = ReadIndex(stream, setting->vertex_index_size);
    stream->read((char*)this->position_offset, sizeof(float) * 3);
}

void PmxMorphUVOffset::Read(std::istream *stream, PmxSetting *setting) {
    this->vertex_index = ReadIndex(stream, setting->vertex_index_size);
    stream->read((char*)this->uv_offset, sizeof(float) * 4);
}

void PmxMorphBoneOffset::Read(std::istream *stream, PmxSetting *setting) {
    this->bone_index = ReadIndex(stream, setting->bone_index_size);
    stream->read((char*)this->translation, sizeof(float) * 3);
    stream->read((char*)this->rotation, sizeof(float) * 4);
}

void PmxMorphMaterialOffset::Read(std::istream *stream, PmxSetting *setting) {
    this->material_index = ReadIndex(stream, setting->material_index_size);
    stream->read((char*) &this->offset_operation, sizeof(uint8_t));
    stream->read((char*)this->diffuse, sizeof(float) * 4);
    stream->read((char*)this->specular, sizeof(float) * 3);
    stream->read((char*) &this->specularity, sizeof(float));
    stream->read((char*)this->ambient, sizeof(float) * 3);
    stream->read((char*)this->edge_color, sizeof(float) * 4);
    stream->read((char*) &this->edge_size, sizeof(float));
    stream->read((char*)this->texture_argb, sizeof(float) * 4);
    stream->read((char*)this->sphere_texture_argb, sizeof(float) * 4);
    stream->read((char*)this->toon_texture_argb, sizeof(float) * 4);
}

void PmxMorphGroupOffset::Read(std::istream *stream, PmxSetting *setting) {
    this->morph_index = ReadIndex(stream, setting->morph_index_size);
    stream->read((char*) &this->morph_weight, sizeof(float));
}

void PmxMorphFlipOffset::Read(std::istream *stream, PmxSetting *setting) {
    this->morph_index = ReadIndex(stream, setting->morph_index_size);
    stream->read((char*) &this->morph_value, sizeof(float));
}

void PmxMorphImplusOffset::Read(std::istream *stream, PmxSetting *setting) {
    this->rigid_body_index = ReadIndex(stream, setting->rigidbody_index_size);
    stream->read((char*) &this->is_local, sizeof(uint8_t));
    stream->read((char*)this->velocity, sizeof(float) * 3);
    stream->read((char*)this->angular_torque, sizeof(float) * 3);
}

void PmxMorph::Read(std::istream *stream, PmxSetting *setting) {
    this->morph_name = ReadString(stream, setting->encoding);
    this->morph_english_name = ReadString(stream, setting->encoding);
    stream->read((char*) &category, sizeof(MorphCategory));
    stream->read((char*) &morph_type, sizeof(MorphType));
    stream->read((char*) &this->offset_count, sizeof(int));
    switch (this->morph_type) {
    case MorphType_Group:
        group_offsets = new PmxMorphGroupOffset[this->offset_count];
        for (int i = 0; i < offset_count; i++) {
            group_offsets[i].Read(stream, setting);
        }
        break;
    case MorphType_Vertex:
        vertex_offsets = new PmxMorphVertexOffset[this->offset_count];
        for (int i = 0; i < offset_count; i++) {
            vertex_offsets[i].Read(stream, setting);
        }
        break;
    case MorphType_Bone:
        bone_offsets = new PmxMorphBoneOffset[this->offset_count];
        for (int i = 0; i < offset_count; i++) {
            bone_offsets[i].Read(stream, setting);
        }
        break;
    case MorphType_Matrial:
        material_offsets = new PmxMorphMaterialOffset[this->offset_count];
        for (int i = 0; i < offset_count; i++) {
            material_offsets[i].Read(stream, setting);
        }
        break;
    case MorphType_UV:
    case MorphType_AdditionalUV1:
    case MorphType_AdditionalUV2:
    case MorphType_AdditionalUV3:
    case MorphType_AdditionalUV4:
        uv_offsets = new PmxMorphUVOffset[this->offset_count];
        for (int i = 0; i < offset_count; i++) {
            uv_offsets[i].Read(stream, setting);
        }
        break;
    default:
        throw;
    }
}

void PmxFrameElement::Read(std::istream *stream, PmxSetting *setting) {
    stream->read((char*) &this->element_target, sizeof(uint8_t));
    if (this->element_target == 0x00) {
        this->index = ReadIndex(stream, setting->bone_index_size);
    } else {
        this->index = ReadIndex(stream, setting->morph_index_size);
    }
}

void PmxFrame::Read(std::istream *stream, PmxSetting *setting) {
    this->frame_name = ReadString(stream, setting->encoding);
    this->frame_english_name = ReadString(stream, setting->encoding);
    stream->read((char*) &this->frame_flag, sizeof(uint8_t));
    stream->read((char*) &this->element_count, sizeof(int));
    this->elements.resize(this->element_count);
    for (int i = 0; i < this->element_count; i++) {
        this->elements[i].Read(stream, setting);
    }
}

void PmxRigidBody::Read(std::istream *stream, PmxSetting *setting) {
    this->girid_body_name = ReadString(stream, setting->encoding);
    this->girid_body_english_name = ReadString(stream, setting->encoding);
    this->target_bone = ReadIndex(stream, setting->bone_index_size);
    stream->read((char*) &this->group, sizeof(uint8_t));
    stream->read((char*) &this->mask, sizeof(uint16_t));
    stream->read((char*) &this->shape, sizeof(uint8_t));
    stream->read((char*) this->size, sizeof(float) * 3);
    stream->read((char*) this->position, sizeof(float) * 3);
    stream->read((char*) this->orientation, sizeof(float) * 3);
    stream->read((char*) &this->mass, sizeof(float));
    stream->read((char*) &this->move_attenuation, sizeof(float));
    stream->read((char*) &this->rotation_attenuation, sizeof(float));
    stream->read((char*) &this->repulsion, sizeof(float));
    stream->read((char*) &this->friction, sizeof(float));
    stream->read((char*) &this->physics_calc_type, sizeof(uint8_t));
}

void PmxJointParam::Read(std::istream *stream, PmxSetting *setting) {
    this->rigid_body1 = ReadIndex(stream, setting->rigidbody_index_size);
    this->rigid_body2 = ReadIndex(stream, setting->rigidbody_index_size);
    stream->read((char*) this->position, sizeof(float) * 3);
    stream->read((char*) this->orientaiton, sizeof(float) * 3);
    stream->read((char*) this->move_limitation_min, sizeof(float) * 3);
    stream->read((char*) this->move_limitation_max, sizeof(float) * 3);
    stream->read((char*) this->rotation_limitation_min, sizeof(float) * 3);
    stream->read((char*) this->rotation_limitation_max, sizeof(float) * 3);
    stream->read((char*) this->spring_move_coefficient, sizeof(float) * 3);
    stream->read((char*) this->spring_rotation_coefficient, sizeof(float) * 3);
}

void PmxJoint::Read(std::istream *stream, PmxSetting *setting) {
    this->joint_name = ReadString(stream, setting->encoding);
    this->joint_english_name = ReadString(stream, setting->encoding);
    stream->read((char*) &this->joint_type, sizeof(uint8_t));
    this->param.Read(stream, setting);
}

void PmxAncherRigidBody::Read(std::istream *stream, PmxSetting *setting) {
    this->related_rigid_body = ReadIndex(stream, setting->rigidbody_index_size);
    this->related_vertex = ReadIndex(stream, setting->vertex_index_size);
    stream->read((char*) &this->is_near, sizeof(uint8_t));
}

void PmxSoftBody::Read(std::istream *stream, PmxSetting *setting) {
    // 未実装
    std::cerr << "Not Implemented Exception" << std::endl;
    throw;
}

void PmxModel::Init() {
    this->version = 0.0f;
    this->model_name.clear();
    this->model_english_name.clear();
    this->model_comment.clear();
    this->model_english_commnet.clear();
    this->vertex_count = 0;
    //this->vertices = nullptr;
    this->index_count = 0;
    //this->indices = nullptr;
    this->texture_count = 0;
    //this->textures = nullptr;
    this->material_count = 0;
    //this->materials = nullptr;
    this->bone_count = 0;
    //this->bones = nullptr;
    this->morph_count = 0;
    //this->morphs = nullptr;
    this->frame_count = 0;
    //this->frames = nullptr;
    this->rigid_body_count = 0;
    //this->rigid_bodies = nullptr;
    this->joint_count = 0;
    //this->joints = nullptr;
    this->soft_body_count = 0;
    //this->soft_bodies = nullptr;
}

void PmxModel::Read(std::istream *stream) {
    // マジック
    char magic[4];
    stream->read((char*) magic, sizeof(char) * 4);
    if (magic[0] != 0x50 || magic[1] != 0x4d || magic[2] != 0x58 || magic[3] != 0x20) {
        std::cerr << "invalid magic number." << std::endl;
        throw;
    }
    // バージョン
    stream->read((char*) &version, sizeof(float));
    if (version != 2.0f && version != 2.1f) {
        std::cerr << "this is not ver2.0 or ver2.1 but " << version << "." << std::endl;
        throw;
    }
    // ファイル設定
    this->setting.Read(stream);

    // モデル情報
    this->model_name = ReadString(stream, setting.encoding);
    this->model_english_name = ReadString(stream, setting.encoding);
    this->model_comment = ReadString(stream, setting.encoding);
    this->model_english_commnet = ReadString(stream, setting.encoding);

    // 頂点
    stream->read((char*) &vertex_count, sizeof(int));
    this->vertices.resize(vertex_count);
    for (int i = 0; i < vertex_count; i++) {
        vertices[i].Read(stream, &setting);
    }

    // 面
    stream->read((char*) &index_count, sizeof(int));
    this->indices.resize(index_count);
    for (int i = 0; i < index_count; i++) {
        this->indices[i] = ReadIndex(stream, setting.vertex_index_size);
    }

    // テクスチャ
    stream->read((char*) &texture_count, sizeof(int));
    this->textures.resize(texture_count);
    for (int i = 0; i < texture_count; i++) {
        this->textures[i] = ReadString(stream, setting.encoding);
    }

    // マテリアル
    stream->read((char*) &material_count, sizeof(int));
    this->materials.resize(material_count);
    for (int i = 0; i < material_count; i++) {
        this->materials[i].Read(stream, &setting);
    }

    // ボーン
    stream->read((char*) &this->bone_count, sizeof(int));
    this->bones.resize(this->bone_count);
    for (int i = 0; i < this->bone_count; i++) {
        this->bones[i].Read(stream, &setting);
    }

    // モーフ
    stream->read((char*) &this->morph_count, sizeof(int));
    this->morphs.resize(this->morph_count);
    for (int i = 0; i < this->morph_count; i++) {
        this->morphs[i].Read(stream, &setting);
    }

    // 表示枠
    stream->read((char*) &this->frame_count, sizeof(int));
    this->frames.resize(this->frame_count);
    for (int i = 0; i < this->frame_count; i++) {
        this->frames[i].Read(stream, &setting);
    }

    // 剛体
    stream->read((char*) &this->rigid_body_count, sizeof(int));
    this->rigid_bodies.resize(this->rigid_body_count);
    for (int i = 0; i < this->rigid_body_count; i++) {
        this->rigid_bodies[i].Read(stream, &setting);
    }

    // ジョイント
    stream->read((char*) &this->joint_count, sizeof(int));
    this->joints.resize(this->joint_count);
    for (int i = 0; i < this->joint_count; i++) {
        this->joints[i].Read(stream, &setting);
    }

    //// ソフトボディ
    //if (this->version == 2.1f)
    //{
    //	stream->read((char*) &this->soft_body_count, sizeof(int));
    //	this->soft_bodies = std::make_unique<PmxSoftBody []>(this->soft_body_count);
    //	for (int i = 0; i < this->soft_body_count; i++)
    //	{
    //		this->soft_bodies[i].Read(stream, &setting);
    //	}
    //}
}

//std::unique_ptr<PmxModel> ReadFromFile(const char *filename)
//{
//	auto stream = std::ifstream(filename, std::ios_base::binary);
//	auto pmx = PmxModel::ReadFromStream(&stream);
//	if (!stream.eof())
//	{
//		std::cerr << "don't reach the end of file." << std::endl;
//	}
//	stream.close();
//	return pmx;
//}

//std::unique_ptr<PmxModel> ReadFromStream(std::istream *stream)
//{
//	auto pmx = std::make_unique<PmxModel>();
//	pmx->Read(stream);
//	return pmx;
//}
}

} // end of namespace MMD
} // end of namespace Assimp

#endif // AI_MMDAHELPER_H_INC
