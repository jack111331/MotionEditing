//
// Created by edge on 2021/3/20.
//

#ifndef OGRETUTORIALSSAMPLE_BVHPARSER_H
#define OGRETUTORIALSSAMPLE_BVHPARSER_H

#include <string>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>
#include "lexertk.hpp"

class BVHNode {
public:
    void parseJointName(const lexertk::generator &ss, size_t &i);
    void parseOffset(const lexertk::generator &ss, size_t &i);
    size_t parseChannelLayout(const lexertk::generator &ss, size_t &i);
    void addChild(BVHNode *node) {
        m_childNodeList.push_back(node);
    }
    virtual ~BVHNode() {
        for(auto node : m_childNodeList) {
            delete node;
        }
    }

    std::string m_jointName;
    glm::vec3 m_offset;
    int m_channels = 0;
    int m_channelLayoutAssign[6];
    int m_inMotionStartIndex;
    std::vector<BVHNode *> m_childNodeList;
    enum class ChannelLayout{
        XPOSITION=0,
        YPOSITION=1,
        ZPOSITION=2,
        XROTATION=3,
        YROTATION=4,
        ZROTATION=5
    };
    std::string CHANNEL_TOKEN[6] = {
            "XPOSITION",
            "YPOSITION",
            "ZPOSITION",
            "XROTATION",
            "YROTATION",
            "ZROTATION"
    };
};

class Frame {
public:
    std::vector<float> m_motionParameter;
};


class Motion {
public:
    std::vector<Frame *> m_frameList;
    virtual ~Motion() {
        for(auto frame : m_frameList) {
            delete frame;
        }
    }
};

class BVHParser {
public:
    BVHParser(const std::string &filename);
    void parseMotion(std::stringstream &ss);
    virtual ~BVHParser() {
        delete m_root;
        for(auto motion : m_motionList) {
            delete motion;
        }
    }
private:
    BVHNode *m_root;
    size_t m_jointParameterAmount = 0;
    int m_frameAmount;
    float m_frameTime;
    std::vector<Motion *> m_motionList;

    // not case sensitive
    const std::string HEADER_TOKEN = "HIERARCHY";
    const std::string ROOT_TOKEN = "ROOT";
    const std::string LEFT_TOKEN = "{";
    const std::string RIGHT_TOKEN = "}";
    const std::string OFFSET_TOKEN = "OFFSET";
    const std::string CHANNEL_TOKEN = "CHANNELS";
    const std::string JOINT_TOKEN = "JOINT";
    const std::string JOINT_END_TOKEN = "END";
    const std::string JOINT_SITE_TOKEN = "SITE";
    const std::string MOTION_TOKEN = "MOTION";
    const std::string FRAME_TOKEN = "FRAMES:";
    const std::string FRAME_TIME_TOKEN = "FRAME TIME:";
};

#endif //OGRETUTORIALSSAMPLE_BVHPARSER_H
