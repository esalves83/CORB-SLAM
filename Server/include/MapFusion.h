//
// Created by lifu on 6/6/17.
//

#ifndef PROJECT_MAPFUSION_H
#define PROJECT_MAPFUSION_H

//#include <include/KeyFrame.h>

#include "KeyFrame.h"
#include "LoopClosing.h"
//#include "ros/ros.h"
//#include "corbslam_client/corbslam_update.h"
//#include "corbslam_client/corbslam_insert.h"
//#include "corbslam_client/corbslam_message.h"
#include "ServerMap.h"
#include "TransPose.h"
#include "ORBmatcher.h"
#include "PnPsolver.h"
#include "MapDrawer.h"
#include "ServerMapView.h"
#include "PubToClient.h"
#include "GlobalOptimize.h"

using namespace ORB_SLAM2;

namespace CORBSLAM_SERVER{

    class MapFusion{

    public:

        MapFusion( std::string strSettingPath );

        void createSubServerMap( int mapId );

        void runUpdateFromClient(int connFd, int clientId);
        void runUpdateToClient();

        bool insertKeyFrameToMap(std::string data, int numAgent);
        bool insertMapPointToMap(std::string data, int numAgent);
        bool updateKeyFrameToMap(std::string data, int numAgent);
        bool updateMapPointToMap(std::string data, int numAgent);
        bool deleteMapPointToMap(std::string data, int numAgent);
        
        void pubTransMsToClients(int connFd, std::map<int, cv::Mat> mtransMs);
        void pubNewKFsToClients(int connFd, std::set<LightKeyFrame> newKFs );
        void pubNewMPsToClients(int connFd, std::set<LightMapPoint> newMPs );
        void pubUpdatedKFsToClients(int connFd, std::set<LightKeyFrame> updatedKFs );
        void pubUpdatedMPSToClients(int connFd, std::set<LightMapPoint> updatedMPs );

        void fuseSubMapToMap();

        void resentGlobalMapToClient();

        bool loadORBVocabulary(const string &strVocFile);

        void createKeyFrameDatabase();

        //bool mapFuse( ServerMap* sMapx, ServerMap* sMapy);

        bool mapFuseToGlobalMap( ServerMap * sMap );

        bool detectKeyFrameInServerMap( ServerMap * sMap, KeyFrame* tKF, cv::Mat &newPose, std::vector<KeyFrame *> &candidateKFs);

        void insertServerMapToGlobleMap( ServerMap * sMap, cv::Mat To2n);

    private:

        // ORB vocabulary used for place recognition and feature matching.
        ORBVocabulary *mpVocabulary;

        GlobalOptimize * mpGBA;

        std::string mpStrSettingPath;

        MapDrawer * mpGlobalMapDrawer;
        ServerMapView * mpSMView;
        std::thread * mptViewer;
        PubToClient * pubToClient;

        bool ifSubToGlobalMap[100];

        bool ifNullGlobalMap;

        bool resentGlobalMap;

        std::mutex resentGlobalMapMutex;
        std::mutex nullGlobalMapMutex;

        cv::Mat subMapTransM[100];

        // KeyFrame database for place recognition (relocalization and loop detection).
        KeyFrameDatabase *mpKeyFrameDatabase;

        std::map<int, ServerMap *> serverMap;

        ServerMap * globalMap;

        std::mutex mStreamInOutPutMutex;

        std::mutex mSubMapUpdatedMutex;
        
        // Socket id to connect to client
        int socketId;
        std::vector<int> connFds;
        
        std::map<int, cv::Mat> transMs;
        
        //LoopClosing* mpLoopCloser;
        //std::vector<LoopClosing *> mpLoopClosers;
        //std::thread* mptLoopClosing;

    };
}


#endif //PROJECT_MAPFUSION_H
