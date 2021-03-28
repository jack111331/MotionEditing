//
// Created by edge on 2021/3/20.
//

#include "BVHParser.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <stack>

using namespace std;

BVHParser::BVHParser(const string &filename) {
    ifstream ifs;
    ifs.open(filename);
    string line;
    stack<BVHNode *> nodeStack;

    string allLine;
    while(getline(ifs, line)) {
        if(boost::iequals(line, MOTION_TOKEN)) {
            break;
        }
        allLine += line + " ";
    }

    lexertk::generator generator;
    if (!generator.process(allLine)) {
        cerr << "Failed to parse first part of BVH file" << std::endl;
        return;
    }

    for (size_t i = 0; i < generator.size(); ++i) {
        lexertk::token t = generator[i];
        if(t.type == lexertk::token::token_type::e_symbol) {
            if(boost::iequals(t.value,HEADER_TOKEN)) {
                cout << "This is a BVH file" << endl;
            } else if(boost::iequals(t.value, ROOT_TOKEN)) {
                m_root = new BVHNode;
                m_root->parseJointName(generator, i);
                nodeStack.push(m_root);
            } else if(boost::iequals(t.value, OFFSET_TOKEN)) {
                nodeStack.top()->parseOffset(generator, i);
            } else if(boost::iequals(t.value, CHANNEL_TOKEN)) {
                nodeStack.top()->parseOffset(generator, i);
                nodeStack.top()->m_inMotionStartIndex = m_jointParameterAmount;
                m_jointParameterAmount += nodeStack.top()->parseChannelLayout(generator, i);
            } else if(boost::iequals(t.value, JOINT_TOKEN)) {
                BVHNode *newBVH = new BVHNode;
                nodeStack.top()->addChild(newBVH);
                nodeStack.push(newBVH);
                newBVH->parseJointName(generator, i);
            } else if(boost::iequals(t.value, JOINT_END_TOKEN)) {
                lexertk::token tNext = generator[++i];
                if(boost::iequals(tNext.value, JOINT_SITE_TOKEN)) {
                    BVHNode *newBVH = new BVHNode;
                    nodeStack.top()->addChild(newBVH);
                    nodeStack.push(newBVH);
                    nodeStack.top()->m_jointName = "Dummy";
                } else {
                    cerr << "\"End\" keyword not concatenate with \"SITE\" keyword" << endl;
                    exit(1);
                }
            }
        } else if(t.type == lexertk::token::token_type::e_lcrlbracket) {
        } else if(t.type == lexertk::token::token_type::e_rcrlbracket) {
            nodeStack.pop();
        } else {
            cerr << "Unidentified token" << endl;
        }
    }
    // Stopped at read "MOTION"
    string token;
    do {
        stringstream ss(line);
        ss >> token;
        if(boost::iequals(token, MOTION_TOKEN)) {
            cout << "Start reading motion data" << endl;
        } else if(boost::iequals(token, FRAME_TOKEN)) {
            ss >> m_frameAmount;
        } else if(boost::iequals(token, FRAME_TIME_TOKEN)) {
            ss >> m_frameTime;
            getline(ifs, line);
            ss.clear();
            ss << line;
            parseMotion(ss);
        }
    } while(getline(ifs, line));
    ifs.close();
}

void BVHParser::parseMotion(stringstream &ss) {
    Motion *newMotion = new Motion;
    float value;
    for(size_t i = 0;i < m_frameAmount;++i) {
        Frame *newFrame = new Frame;
        for(size_t j = 0;j < m_jointParameterAmount;++j) {
            ss >> value;
            newFrame->m_motionParameter.push_back(value);
        }
        newMotion->m_frameList.push_back(newFrame);
    }
    m_motionList.push_back(newMotion);
}

void BVHNode::parseJointName(const lexertk::generator &ss, size_t &i) {
    m_jointName = ss[i++].value;
}

void BVHNode::parseOffset(const lexertk::generator &ss, size_t &i) {
    m_offset.x = stof(ss[i++].value);
    m_offset.y = stof(ss[i++].value);
    m_offset.z = stof(ss[i++].value);
}

size_t BVHNode::parseChannelLayout(const lexertk::generator &ss, size_t &i) {
    m_channels = stoi(ss[i++].value);
    for (int i = 0;i < m_channels;++i) {
        string token = ss[i++].value;
        for(int j = 0;j < 6;++j) {
            if(boost::iequals(token,CHANNEL_TOKEN[j])) {
                m_channelLayoutAssign[i] = j;
            }
        }
    }
    return m_channels;
}