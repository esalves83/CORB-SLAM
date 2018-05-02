//
// Created by lifu on 6/6/17.
//

//#include "corbslam_client/corbslam_insertRequest.h"
//#include "corbslam_client/corbslam_message.h"
//#include "corbslam_client/corbslam_insert.h"
//#include "corbslam_client/corbslam_update.h"
//#include "corbslam_client/corbslam_updateRequest.h"
#include <thread>
#include "MapFusion.h"

//#include <boost/asio.hpp>

namespace CORBSLAM_SERVER{

    static std::vector< std::string > elementVec;

    MapFusion::MapFusion( std::string strSettingPath ){

        this->mpStrSettingPath = strSettingPath;

        memset( ifSubToGlobalMap, false, sizeof( ifSubToGlobalMap ));

        for( int i = 0 ; i < 100; i++ ) {
            subMapTransM[i] = cv::Mat::eye( 4,4, CV_32F );
        }

        ifNullGlobalMap = true;

    }
    
    void MapFusion::runUpdateFromClient(int connFd, int clientId){
        
        int socket_buffer_size = 7000000;
    
        //char *pch;
        
        char received[socket_buffer_size+1];
        bzero(received, socket_buffer_size+1);
        std::size_t foundReset;
        string temp;
        
        cout << "Connected to agent " << connFd << endl;
        connFds.push_back(connFd);
        
        while(true)
        {
            bzero(received, socket_buffer_size);        
            int recvd_size = read(connFd, received, socket_buffer_size);
            
            //cout << recvd_size << " bytes received" << endl;
            
            // Concaternar em um buffer de trabalho
            temp += received;
            
            /*foundReset = temp.find("#end#");
            if(foundReset!=std::string::npos)
                break;*/
            
            std::size_t found = temp.find_first_of("*()?[");
            while (found!=std::string::npos)
            {
                if(temp[found] == '*')
                {
                    std::string aux = temp.substr(0,found); //All characters before *
                
                    if(insertKeyFrameToMap(aux, clientId))
                        //cout << "--Server Client[" << clientId << "]: insert keyframe to map func. KFs count: " << temp.length() << " bytes"  << endl;
                    
                    temp.erase(0,found+1);
                }
                
                if(temp[found] == '(')
                {
                    std::string aux = temp.substr(0,found); //All characters before +
                
                    if(insertMapPointToMap(aux, clientId))
                        //cout << "--Server Client[" << clientId << "]: insert MapPoint to map func. MPs count: " << temp.length() << " bytes"  << endl;
                    
                    temp.erase(0,found+1);
                }
                
                if(temp[found] == ')')
                {
                    std::string aux = temp.substr(0,found); //All characters before !
                
                    if(updateKeyFrameToMap(aux, clientId))
                        //cout << "--Server Client[" << clientId << "]: update keyframe to map func. KFs count: " << temp.length() << " bytes"  << endl;
                    
                    temp.erase(0,found+1);
                }
                
                if(temp[found] == '?')
                {
                    std::string aux = temp.substr(0,found); //All characters before ?
                
                    if(updateMapPointToMap(aux, clientId))
                        //cout << "--Server Client[" << clientId << "]: update map points to map func. KFs count: " << temp.length() << " bytes"  << endl;
                    
                    temp.erase(0,found+1);
                }
                
                if(temp[found] == '[')
                {
                    std::string aux = temp.substr(0,found); //All characters before ?
                
                    if(deleteMapPointToMap(aux, clientId))
                        //cout << "--Server Client[" << clientId << "]: update map points to map func. KFs count: " << temp.length() << " bytes"  << endl;
                    
                    temp.erase(0,found+1);
                }
                
                found=temp.find_first_of("*()?[");
            }
        }
        
        /*std::size_t found;
        while(true)
        {    
            bzero(received, socket_buffer_size);        
            int recvd_size = read(connFd, received, socket_buffer_size);
            
            // Concaternar em um buffer de trabalho
            temp += received;
            
            found = temp.find("**");
            if(found!=std::string::npos)
            {
                std::string aux = temp.substr(0,found); //All characters before **
                
                if(insertKeyFrameToMap(aux, clientId))
                    //cout << "--Server Client[" << clientId << "]: insert keyframe to map func. KFs count: " << temp.length() << " bytes"  << endl;
                
                temp.erase(0,found+2); // Erase all characters up to **, including **               
            }
            
            found = temp.find("++");
            if(found!=std::string::npos)
            {
                std::string aux = temp.substr(0,found); //All characters before ++
                
                if(insertMapPointToMap(aux, clientId))
                    //cout << "--Server Client[" << clientId << "]: insert MapPoint to map func. MPs count: " << temp.length() << " bytes"  << endl;
                
                temp.erase(0,found+2); // Erase all characters up to ++, including ++            
            }
            
            found = temp.find("!!");
            if(found!=std::string::npos)
            {
                std::string aux = temp.substr(0,found); //All characters before !!
                
                if(updateKeyFrameToMap(aux, clientId))
                    //cout << "--Server Client[" << clientId << "]: update keyframe to map func. KFs count: " << temp.length() << " bytes"  << endl;
                
                temp.erase(0,found+2); // Erase all characters up to !!, including !!            
            }
            
            found = temp.find("??");
            if(found!=std::string::npos)
            {
                std::string aux = temp.substr(0,found); //All characters before ??
                
                if(updateMapPointToMap(aux, clientId))
                    //cout << "--Server Client[" << clientId << "]: update map points to map func. KFs count: " << temp.length() << " bytes"  << endl;
                
                temp.erase(0,found+2); // Erase all characters up to !!, including !!            
            }            
        }*/
        cout << "##############################################" << endl;
    }
    
    bool MapFusion::deleteMapPointToMap(std::string data, int numAgent){

        //ROS_INFO("--Server Client[%d]: insert MapPoint to map func. MPs count: %d " ,req.CID, req.COUNT );

        int clientId = numAgent;
        
        unique_lock<mutex> lock( mStreamInOutPutMutex );
        elementVec.clear();
        std::stringstream is( data );
        boost::archive::text_iarchive ia(is);
        ia >> elementVec;
        //cout << "Received " << elementVec.size() << " new MPs" << endl;
        int count = 0;
        {
            std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
            if( ifSubToGlobalMap[ clientId ] ) {
                for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                    try {
                        long unsigned int mnId;
                        ORB_SLAM2::MapPoint *tMP = new ORB_SLAM2::MapPoint();
                        std::stringstream iis( elementVec[mit] );
                        boost::archive::text_iarchive iia(iis);
                        iia >> mnId;
                        
                        if(globalMap->pCacher->getMapPointById( mnId )){
                            
                            tMP = globalMap->pCacher->getMapPointById( mnId );
                            globalMap->pCacher->EraseMapPointFromMap(tMP);
                            count++;
                        }
                        
                        /*if( tMP && globalMap->pCacher->getMapPointById( tMP->mnId ) ) {*/
                            
                            /*tMP->setCache( globalMap->pCacher );
                            cv::Mat tPose = tMP->GetWorldPos();
                            cv::Mat Tn2o = cv::Mat::eye(4,4, tPose.type());
                            cv::Mat tcw = subMapTransM[clientId].rowRange(0, 3).col(3);
                            cv::Mat Rwc = subMapTransM[clientId].rowRange(0, 3).colRange(0, 3).t();
                            tPose = Rwc * ( tPose - tcw );
                            tMP->SetWorldPos( tPose );
                            tMP->ifFixed = false;
                            globalMap->pCacher->AddMapPointToMap( tMP );*/
                            
                            //count++;
                        //}
                        
                    } catch ( ... ) {
                        cout << endl << "Exception in delete MapPoint: " << mit << endl;
                    }
                }
            }
        }
        
        cout << "Deleted " << count << " MPs" << endl;

        elementVec.clear();

        return true;
    }

    bool MapFusion::insertKeyFrameToMap(std::string data, int numAgent){

        int clientId = numAgent;

        unique_lock<mutex> lock( mStreamInOutPutMutex );

        elementVec.clear();
        std::stringstream is( data );
        boost::archive::text_iarchive ia(is);
        ia >> elementVec;
        {
            std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
            if( ifSubToGlobalMap[ clientId ] ) {

                for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                    try {
                        ORB_SLAM2::KeyFrame *tKF = new ORB_SLAM2::KeyFrame();
                        std::stringstream iis( elementVec[mit] );
                        boost::archive::text_iarchive iia(iis);
                        iia >> tKF;
                        if( tKF && !globalMap->pCacher->getKeyFrameById( tKF->mnId)) {
                            tKF->setCache( globalMap->pCacher );
                            cv::Mat Tcw = tKF->GetPose();
                            Tcw = Tcw * subMapTransM[ clientId ];
                            tKF->SetPose( Tcw );
                            tKF->ifFixed = false;
                            globalMap->pCacher->AddKeyFrameToMap( tKF );
                            globalMap->pCacher->addKeyFrametoDB( tKF );
                        }
                        
                    } catch( ... ) {
                        cout << "error in insertKeyFrameToMap : " << mit << endl;
                    }
                }
                
                //cout << "Insert KFs in submap " << clientId << endl;

            } else {

                if( serverMap.find( clientId ) == serverMap.end() ) {
                    createSubServerMap( clientId );
                    cout << "Create submap " << clientId << endl;
                }

                ServerMap * clientMap = serverMap[ clientId ];
                clientMap->pCacher->pClientId = clientId;

                for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                    try {
                        ORB_SLAM2::KeyFrame *tKF = new ORB_SLAM2::KeyFrame();
                        std::stringstream iis( elementVec[mit] );
                        boost::archive::text_iarchive iia(iis);
                        iia >> tKF;
                        if( tKF && !clientMap->pCacher->getKeyFrameById( tKF->mnId)) {
                            tKF->setCache( clientMap->pCacher );
                            tKF->ifFixed = false;
                            clientMap->pCacher->AddKeyFrameToMap( tKF );
                            clientMap->pCacher->addKeyFrametoDB( tKF );
                            
                            //mpLoopCloser->InsertKeyFrame(tKF);
                        }
                    } catch( ... ) {
                        cout << elementVec[mit] << endl;
                        cout << "error in insertKeyFrameToMap : " << mit << endl;
                    }
                    //delete tKF;
                }
                //cout << "keyframes in map from insert function " << serverMap[ clientId ]->pCacher->getKeyFramesInMap() << endl;
            }
        }


        elementVec.clear();

        return true;

    }

    bool MapFusion::insertMapPointToMap(std::string data, int numAgent){

        //ROS_INFO("--Server Client[%d]: insert MapPoint to map func. MPs count: %d " ,req.CID, req.COUNT );

        int clientId = numAgent;
        
        unique_lock<mutex> lock( mStreamInOutPutMutex );
        elementVec.clear();
        std::stringstream is( data );
        boost::archive::text_iarchive ia(is);
        ia >> elementVec;
        cout << "Received " << elementVec.size() << " new MPs from client" << endl;
        int count = 0;
        {
            std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
            if( ifSubToGlobalMap[ clientId ] ) {
                for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                    try {
                        ORB_SLAM2::MapPoint *tMP = new ORB_SLAM2::MapPoint();
                        std::stringstream iis( elementVec[mit] );
                        boost::archive::text_iarchive iia(iis);
                        iia >> tMP;
                        if( tMP && !globalMap->pCacher->getMapPointById( tMP->mnId ) ) {
                            tMP->setCache( globalMap->pCacher );
                            cv::Mat tPose = tMP->GetWorldPos();
                            cv::Mat Tn2o = cv::Mat::eye(4,4, tPose.type());
                            cv::Mat tcw = subMapTransM[clientId].rowRange(0, 3).col(3);
                            cv::Mat Rwc = subMapTransM[clientId].rowRange(0, 3).colRange(0, 3).t();
                            tPose = Rwc * ( tPose - tcw );
                            tMP->SetWorldPos( tPose );
                            tMP->ifFixed = false;
                            globalMap->pCacher->AddMapPointToMap( tMP );
                            count++;
                        }
                        
                    } catch ( ... ) {
                        cout << endl << "Exception in insert MapPoint: " << mit << endl;
                    }
                }
                cout << "Inserted " << count << "new MPs" << endl;
                std::vector<MapPoint *> MPs = globalMap->pCacher->GetAllMapPointsFromMap();
                cout << "Global Map has " << MPs.size() << " map points" << endl;
            } else {

                if( serverMap.find( clientId ) == serverMap.end() ) {
                    createSubServerMap( clientId );
                }
                ServerMap * clientMap = serverMap[ clientId ];
                clientMap->pCacher->pClientId = clientId;

                for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                    try {
                        ORB_SLAM2::MapPoint *tMP = new ORB_SLAM2::MapPoint();
                        std::stringstream iis( elementVec[mit] );
                        boost::archive::text_iarchive iia(iis);
                        iia >> tMP;
                        if( tMP && !clientMap->pCacher->getMapPointById( tMP->mnId )) {
                            tMP->setCache( clientMap->pCacher );
                            tMP->ifFixed = false;
                            clientMap->pCacher->AddMapPointToMap( tMP );
                        }
                        //delete tMP;
                    } catch ( ... ) {
                        cout << endl << "Exception in insert MapPoint: " << mit << endl;
                    }
                }
            }
        }

        elementVec.clear();

        return true;
    }

    bool MapFusion::updateKeyFrameToMap(std::string data, int numAgent) {

        //ROS_INFO("--Server client[%d]: update keyframe to map func. KFs count:%d" , req.CID, req.COUNT );
        //cout << "INIT UPDATE" << endl;

        int clientId = numAgent; //cout << "int clientId = numAgent" << endl;

        unique_lock<mutex> lock( mStreamInOutPutMutex );

        if ( clientId > 0 ) {

            elementVec.clear(); //cout << "elementVec.clear()" << endl;
            std::stringstream is( data ); //cout << "std::stringstream is( data )" << endl;
            boost::archive::text_iarchive ia(is); //cout << "boost::archive::text_iarchive ia(is)" << endl;
            ia >> elementVec; //cout << "ia >> elementVec" << endl;

            {
                std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
                if( ifSubToGlobalMap[ clientId ] ) {
                    for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                        try {
                            ORB_SLAM2::KeyFramePose *tKFP = new ORB_SLAM2::KeyFramePose();
                            std::stringstream iis( elementVec[mit] );
                            boost::archive::text_iarchive iia(iis);
                            iia >> tKFP;
                            if( tKFP ) {
                                ORB_SLAM2::KeyFrame * tKF = globalMap->pCacher->getKeyFrameById( tKFP->KFPId );
                                if( tKF ) {
                                    cv::Mat Tcw = tKFP->KFPose;
                                    Tcw = Tcw * subMapTransM[ clientId ];
                                    tKF->SetPose( Tcw );
                                    globalMap->pCacher->addUpdateKeyframe( tKF );
                                }
                            }
                            
                        } catch( ... ) {
                            cout << "error in updateKeyFrameToMap : " << mit << endl;
                        }
                        //delete tKFP;
                    }
                } else {
                    if( serverMap.find( clientId ) == serverMap.end() ) {
                        return false;
                    }

                    ServerMap * clientMap = serverMap[ clientId ];

                    for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                        try {
                            ORB_SLAM2::KeyFramePose *tKFP = new ORB_SLAM2::KeyFramePose();
                            std::stringstream iis( elementVec[mit] );
                            boost::archive::text_iarchive iia(iis);
                            iia >> tKFP;
                            if( tKFP ) {
                                ORB_SLAM2::KeyFrame * tKF = clientMap->pCacher->getKeyFrameById( tKFP->KFPId );
                                if( tKF ){
                                    tKF->SetPose( tKFP->KFPose );
                                }
                            }
                            //delete tKFP;
                        } catch( ... ) {
                            cout << "error in updateKeyFrameToMap : " << mit << endl;
                        }
                    }
                }
            }


            elementVec.clear();
        }

        return true;
    }

    bool MapFusion::updateMapPointToMap(std::string data, int numAgent) {

        //ROS_INFO("--Server Client[%d]: update MapPoint to map func. MPs count:%d", req.CID, req.COUNT );

        int clientId = numAgent;

        unique_lock<mutex> lock( mStreamInOutPutMutex );

        if ( clientId > 0 ) {

            elementVec.clear();
            std::stringstream is( data );
            boost::archive::text_iarchive ia(is);
            ia >> elementVec;
            int count = 0;
            {
                std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
                if( ifSubToGlobalMap[ clientId ] ) {
                    for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                        try {
                            ORB_SLAM2::MapPointPose *tMPP = new ORB_SLAM2::MapPointPose();
                            std::stringstream iis( elementVec[mit] );
                            boost::archive::text_iarchive iia(iis);
                            iia >> tMPP;
                            if( tMPP ) {
                                ORB_SLAM2::MapPoint * tMP = globalMap->pCacher->getMapPointById( tMPP->MPPId );
                                if( tMP ) {
                                    cv::Mat tPose = tMPP->MPPose;
                                    cv::Mat tcw = subMapTransM[clientId].rowRange(0, 3).col(3);
                                    cv::Mat Rwc = subMapTransM[clientId].rowRange(0, 3).colRange(0, 3).t();
                                    tPose = Rwc * ( tPose - tcw );
                                    tMP->SetWorldPos( tPose );
                                    globalMap->pCacher->addUpdateMapPoint( tMP );
                                    count++;
                                }
                            }                            
                        } catch( ... ) {
                            cout << "error in updateMapPointToMap : " << mit << endl;
                        }
                    }
                    cout << "Inserted " << count << "updated MPs" << endl;
                    std::vector<MapPoint *> MPs = globalMap->pCacher->GetAllMapPointsFromMap();
                    cout << "Global Map has " << MPs.size() << " map points" << endl;
                } else {
                    if( serverMap.find( clientId ) == serverMap.end() ) {
                        return false;
                    }
                    ServerMap * clientMap = serverMap[ clientId ];

                    for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                        try {
                            ORB_SLAM2::MapPointPose *tMPP = new ORB_SLAM2::MapPointPose();
                            std::stringstream iis( elementVec[mit] );
                            boost::archive::text_iarchive iia(iis);
                            iia >> tMPP;
                            if( tMPP ) {
                                ORB_SLAM2::MapPoint * tMP = clientMap->pCacher->getMapPointById( tMPP->MPPId );
                                if( tMP ){
                                    tMP->SetWorldPos( tMPP->MPPose );
                                }
                            }
                            //delete tMPP;
                        } catch( ... ) {
                            cout << "error in updateMapPointToMap : " << mit << endl;
                        }
                    }
                }
            }


            elementVec.clear();
        }

        return true;

    }

    void MapFusion::runUpdateToClient() {

        //ROS_INFO("Thread runPubInsertTopic start...");
        cout << "Thread runUpdateToClient start... " << endl;
        
        //ros::Rate loop_rate(0.5);

        //while( ros::ok() ) {
        while (true){
            //if not resent the global map to the clients, publish the new or updated to clients
            {

                std::unique_lock<mutex> lock( resentGlobalMapMutex );

                std::set<LightKeyFrame> tnewInsertedKFs ;
                std::set<LightMapPoint> tnewInsertedMPs ;
                std::set<LightKeyFrame> tupdateKFs ;
                std::set<LightMapPoint> tupdateMPs ;
                
                std::map<int, cv::Mat> mtransMs;
                
                //cout << "Thread runUpdateToClient get the lock... " << endl;

                {
                    std::unique_lock<mutex> lock(mSubMapUpdatedMutex);

                    tnewInsertedKFs = globalMap->pCacher->newInsertedKFs;
                    tnewInsertedMPs = globalMap->pCacher->newInsertedMPs;
                    tupdateKFs = globalMap->pCacher->updateKFs;
                    tupdateMPs = globalMap->pCacher->updateMPs;
                    
                    mtransMs = transMs;
                    
                    //std::vector<KeyFrame *> KFs = globalMap->pCacher->getAllKeyFramesInMap();
                    //cout << "Keyframes: " << KFs.size() << endl;
                    
                    //std::vector<MapPoint *> MPs = globalMap->pCacher->GetAllMapPointsFromMap();
                    //cout << "Map Points: " << MPs.size() << endl;

                    globalMap->pCacher->newInsertedMPs.clear();
                    globalMap->pCacher->newInsertedKFs.clear();
                    globalMap->pCacher->updateKFs.clear();
                    globalMap->pCacher->updateMPs.clear();
                }

                
                int count = (int)( tnewInsertedKFs.size() + tnewInsertedMPs.size() + tupdateKFs.size() + tupdateMPs.size() );

                if( count > 0 ) {
                    
                    //cout << "******************************************" << endl;
                    
                    for (std::map<int, ServerMap *>::iterator tmpx = serverMap.begin(); tmpx != serverMap.end(); tmpx++) {
                        
                        if( ifSubToGlobalMap[ (*tmpx).first ] ){
                            
                            pubTransMsToClients(connFds[(*tmpx).first-1], mtransMs);
                            usleep(50000);
                            pubNewKFsToClients(connFds[(*tmpx).first-1], tnewInsertedKFs);
                            usleep(50000);
                            pubNewMPsToClients(connFds[(*tmpx).first-1], tnewInsertedMPs);
                            usleep(50000);
                            pubUpdatedKFsToClients(connFds[(*tmpx).first-1], tupdateKFs);
                            usleep(50000);
                            pubUpdatedMPSToClients(connFds[(*tmpx).first-1], tupdateMPs);
                        }
                    }
                    
                    //cout << "******************************************" << endl;
                }
            }

            usleep(500000);
            
            /*if(CheckrunUpdateToClientFinish())
                break;*/
        }

    }
    
    void MapFusion::pubTransMsToClients(int connFd, std::map<int, cv::Mat> mtransMs){
        
        std::ostringstream oos;
        boost::archive::text_oarchive ooa(oos);
        ooa << mtransMs;
            
        std::string ss = oos.str();
        //ss.append("TransMs");
        ss.append("#");
          
        int n = write(connFd, ss.c_str(), ss.length());
        if (n < 0) cout << "ERROR writing to client" << endl;
            
        ss.clear();
        oos.clear();
           
        //cout << "inserted " << n << " bytes" << " TransM new keyframes to client " << connFd << endl;
    }
    
    void MapFusion::pubNewKFsToClients(int connFd, std::set<LightKeyFrame> newKFs ) {

        //corbslam_client::corbslam_message msg;
        std::vector< std::string > KFsData;
        KFsData.clear();

        for( std::set<LightKeyFrame>::iterator mit = newKFs.begin(); mit != newKFs.end(); mit++ ) {

            KeyFrame * tKF = (*mit).getKeyFrame();
            if( tKF ) {
                std::ostringstream os;
                boost::archive::text_oarchive oa(os);
                oa << tKF;
                KFsData.push_back( os.str() );
                os.clear();
            }
        }
        
        if(KFsData.size() > 0)
        {
            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << KFsData;
            //msg.DATA = os.str();
            
            std::string ss = os.str();
            //ss.append("newKFs");
            ss.append("$");
            
            int n = write(connFd, ss.c_str(), ss.length());
            if (n < 0) cout << "ERROR writing to client" << endl;
            
            os.clear();
            ss.clear();
            KFsData.clear();
            
            //cout << "inserted " << n << " bytes" << " new keyframes to client " << connFd << endl;
        }
        //insertKeyFramesPub.publish(msg);

    }
    
    void MapFusion::pubNewMPsToClients(int connFd, std::set<LightMapPoint> newMPs ) {

        //corbslam_client::corbslam_message msg;

        std::vector< std::string > MPsData;
        MPsData.clear();
        for( std::set<LightMapPoint>::iterator mit = newMPs.begin(); mit != newMPs.end(); mit++ ) {

            MapPoint * tMP = (*mit).getMapPoint();
            if( tMP ) {
                std::ostringstream os;
                boost::archive::text_oarchive oa(os);
                oa << tMP;
                MPsData.push_back( os.str() );
                os.clear();
            }
        }
        
        cout << "Send " << MPsData.size() << " new MPs to client" << endl;
        
        if(MPsData.size() > 0)
        {
            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << MPsData;
            //msg.DATA = os.str();
            
            std::string ss = os.str();
            //ss.append("newMPs");
            ss.append("@");
            
            int n = write(connFd, ss.c_str(), ss.length());
            if (n < 0) cout << "ERROR writing to client" << endl;
            
            os.clear();
            ss.clear();
            MPsData.clear();
            
            //cout << "inserted " << n << " bytes" << " new map points to client " << connFd << endl;
        }
    }
    
    void MapFusion::pubUpdatedKFsToClients(int connFd, std::set<LightKeyFrame> updatedKFs ) {

        //corbslam_client::corbslam_message msg;

        std::vector< std::string > KFsData;
        KFsData.clear();

        for( std::set<LightKeyFrame>::iterator mit = updatedKFs.begin(); mit != updatedKFs.end(); mit++ ) {

            KeyFrame * tKF = (*mit).getKeyFrame();
            if( tKF ) {
                KeyFramePose* tKFP = new KeyFramePose( tKF );
                std::ostringstream os;
                boost::archive::text_oarchive oa(os);
                oa << tKFP ;
                KFsData.push_back( os.str() );
                os.clear();
                
                //delete tKFP;
            }
        }
        
        if(KFsData.size() > 0){
            
            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << KFsData;
            //msg.DATA = os.str();
            
            std::string ss = os.str();
            ss.append("=");
            
            int n = write(connFd, ss.c_str(), ss.length());
            if (n < 0) cout << "ERROR writing to client" << endl;
            
            os.clear();
            ss.clear();
            KFsData.clear();
            
            //cout << "inserted " << n << " bytes" << " updated keyframes to client " << endl;
        }
    }
    
    void MapFusion::pubUpdatedMPSToClients(int connFd, std::set<LightMapPoint> updatedMPs ) {

        //corbslam_client::corbslam_message msg;

        std::vector< std::string > MPsData;
        MPsData.clear();

        for( std::set<LightMapPoint>::iterator mit = updatedMPs.begin(); mit != updatedMPs.end(); mit++ ) {
            MapPoint *tMP = (*mit).getMapPoint();
            if( tMP ) {
                MapPointPose * tMPP = new MapPointPose( tMP );
                std::ostringstream os;
                boost::archive::text_oarchive oa(os);
                oa << tMPP;
                std::string ss = os.str();
                MPsData.push_back(ss);
                os.clear();
                ss.clear();
                
                //delete tMPP;
            }
        }
        
        cout << "Send " << MPsData.size() << " updated MPs to client" << endl;
        
        if(MPsData.size() > 0){
            
            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << MPsData;
            //msg.DATA = os.str();
            
            std::string ss = os.str();
            ss.append("*");
            
            int n = write(connFd, ss.c_str(), ss.length());
            if (n < 0) cout << "ERROR writing to client" << endl;
            
            os.clear();
            ss.clear();
            MPsData.clear();
            
            //cout << "inserted " << n << " bytes" << " updated map points to client " << endl;
        }
    }

    void MapFusion::resentGlobalMapToClient() {

        std::unique_lock<mutex> lock( resentGlobalMapMutex );
        
        cout << "Thread resentGlobalMapToClient get the lock... " << endl;

        //ROS_INFO( "Resent to client start");
        

        std::vector<KeyFrame * > allKFs ;
        std::vector<MapPoint * > allMPs ;
        
        std::map<int, cv::Mat> mtransMs;

        {
            std::unique_lock<mutex> lock1(mSubMapUpdatedMutex);

            globalMap->pCacher->newInsertedMPs.clear();
            globalMap->pCacher->newInsertedKFs.clear();
            globalMap->pCacher->updateKFs.clear();
            globalMap->pCacher->updateMPs.clear();

            allKFs = globalMap->pMap->GetAllKeyFrames();
            allMPs = globalMap->pMap->GetAllMapPoints();
            
            mtransMs = transMs;

        }

        int mit = 0;

        //ros::Rate loop_rate(0.5);

        //time_t start_t = clock();

        //while( ros::ok() )
        //usleep(4000000);
        while( true ) {

            std::set<LightKeyFrame> tmpLKFs;
            std::set<LightMapPoint> tmpLMPs;

            for( int i = mit * 50 ; i < min( (int)allKFs.size(), (mit + 1 ) * 50 ); i++ ) {
                tmpLKFs.insert( LightKeyFrame( allKFs[i]));
            }

            for( int i = mit * 2000 ; i < min( (int)allMPs.size(), (mit + 1 ) * 2000 ); i++ ) {
                tmpLMPs.insert( LightMapPoint( allMPs[i]));
            }

            if( tmpLKFs.size() == 0 && tmpLMPs.size() == 0) {
                break;
            } else {
                
                /*if( ifSubToGlobalMap[1] ){
                        
                        pubTransMsToClients(connFds[0], mtransMs);
                        usleep(50000);
                        pubNewKFsToClients(connFds[0], tmpLKFs);
                        usleep(50000);
                        pubNewMPsToClients(connFds[0], tmpLMPs);
                        usleep(50000);
                    }*/
                
                for (std::map<int, ServerMap *>::iterator tmpx = serverMap.begin(); tmpx != serverMap.end(); tmpx++) {
                        
                    if( ifSubToGlobalMap[ (*tmpx).first ] ){
                        
                        pubTransMsToClients(connFds[(*tmpx).first-1], mtransMs);
                        usleep(50000);
                        pubNewKFsToClients(connFds[(*tmpx).first-1], tmpLKFs);
                        usleep(50000);
                        pubNewMPsToClients(connFds[(*tmpx).first-1], tmpLMPs);
                        usleep(50000);
                    }
                }

                //pubToClient->pubNewKFsToClients( tmpLKFs );
                //ros::spinOnce();
                //pubToClient->pubNewMPsToClients( tmpLMPs );
                //ros::spinOnce();
                mit++;
            }

            //loop_rate.sleep();
            usleep(500000);
            //cout << "Resent to client start" << endl;

        }
        
        cout << "******************* Finish resentGlobalMapToClient *********************" << endl;
        cout << "************************************************************************" << endl;
        //runUpdateToClient();

        //time_t end_t = clock();

        //ROS_INFO( "Resent to client end, use time: %lfs", (double)(end_t - start_t)/(double)CLOCKS_PER_SEC );

    }

    void MapFusion::fuseSubMapToMap(){
        
        while(1){
            
            for (std::map<int, ServerMap *>::iterator tmpx = serverMap.begin(); tmpx != serverMap.end(); tmpx++) {

                //TODO the map fusion logic need to be fixed
                if( !ifSubToGlobalMap[ (*tmpx).first ] ) {
                    
                    //cout << "Calling mapFuseToGlobalMap" << endl;

                    mapFuseToGlobalMap( (*tmpx).second );

                }
            }
            
            usleep(500000);
        }
    }

    bool MapFusion::mapFuseToGlobalMap(ServerMap *sMap) {

        time_t start_t = clock();
        fstream detectf;
        detectf.open( "detectMap.txt" );

        // if global map is null, this submap is inserted into the global map and does not do transform
        {
            std::unique_lock<mutex> lock( nullGlobalMapMutex );
            if( ifNullGlobalMap ) {

                cv::Mat Tnorm = cv::Mat::eye(4,4, CV_32F);
                std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
                insertServerMapToGlobleMap(sMap, Tnorm);
                ifSubToGlobalMap[(*sMap).pCacher->pClientId] = true;
                subMapTransM[(*sMap).pCacher->pClientId] = Tnorm;
                //pubToClient->transMs[ (*sMap).pCacher->pClientId ] = Tnorm;
                transMs[ (*sMap).pCacher->pClientId ] = Tnorm;
                sMap->clear();
                ifNullGlobalMap = false;

                cout <<"Global Map is not null!\n";
                return true;
            }
        }
        
        bool flag = false;
        std::vector<KeyFrame*> allKeyFramesInMapy = sMap->pMap->GetAllKeyFrames();

        bool bOK = false;
        std::vector<KeyFrame *> candidateKFs;
        KeyFrame * currentKF;
        
        for( int mit = 0; mit < (int)allKeyFramesInMapy.size(); mit++ ) {

            KeyFrame * tKF = allKeyFramesInMapy[mit];

            cv::Mat oldTwc = tKF->GetPoseInverse();
            cv::Mat oldTcw = tKF->GetPose();
            cv::Mat newTcw = cv::Mat::eye(4,4, oldTcw.type());

            candidateKFs.clear();
            currentKF = tKF;

            bOK = detectKeyFrameInServerMap( globalMap, tKF, newTcw, candidateKFs);

            if( bOK ) {

                mpGBA->setCurentKeyFrame( currentKF );

                mpGBA->setCandidates( candidateKFs );

                if( mpGBA->ComputeSim3() ) {

                    //ROS_INFO("Detect In serverMap[%d], from keyframe id[%d]", (int)sMap->pCacher->pClientId, (int)tKF->mnId);
                    // if this keyframe is deteted in the serverMapx

                    cv::Mat To2n = oldTwc * newTcw;
                    cv::Mat Tnorm = cv::Mat::eye(4,4, newTcw.type());

                    flag = true;

                    {
                        std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
                        subMapTransM[ (*sMap).pCacher->pClientId ] = To2n;
                        //pubToClient->transMs[ (*sMap).pCacher->pClientId ] = To2n;
                        transMs[ (*sMap).pCacher->pClientId ] = To2n;
                        ifSubToGlobalMap[ (*sMap).pCacher->pClientId ] = true;
                        insertServerMapToGlobleMap( sMap, To2n );
                        sMap->clear();
                        
                        mpGBA->CorrectLoop();
                    }                    

                    //mpGBA->CorrectLoop();

                    time_t end_t = clock();

                    cout << "mapfuse " << globalMap->pMap->KeyFramesInMap() << " " << allKeyFramesInMapy.size() << " " << (double)( end_t - start_t )/(double)CLOCKS_PER_SEC << endl;

                    break;

                }

            }

        }

        if( flag ){
            //pubTransMsToClients(connFds);
            resentGlobalMapToClient();
        }
        return flag;

    }

    /*bool MapFusion::mapFuse( ServerMap* sMapx, ServerMap* sMapy) {

        bool flag = false;
        std::vector<KeyFrame*> allKeyFramesInMapy = sMapy->pMap->GetAllKeyFrames();

        bool bOK = false;
        std::vector<KeyFrame *>  candidateKFs;
        KeyFrame * currentKF;

        for( int mit = 0; mit < (int)allKeyFramesInMapy.size(); mit++ ) {

            KeyFrame * tKF = allKeyFramesInMapy[mit];

            cv::Mat oldTwc = tKF->GetPoseInverse();
            cv::Mat oldTcw = tKF->GetPose();
            cv::Mat newTcw = cv::Mat::eye(4,4, newTcw.type());

            time_t start_t = clock();

            candidateKFs.clear();
            currentKF = tKF;

            bOK = detectKeyFrameInServerMap( sMapx, tKF, newTcw, candidateKFs);

            if( bOK ) {

                mpGBA->setCurentKeyFrame( currentKF );

                mpGBA->setCandidates( candidateKFs );

                if( mpGBA->ComputeSim3() ) {

                    ROS_INFO("Detect In serverMap[%d], from keyframe id[%d]", (int)sMapx->pCacher->pClientId, (int)tKF->mnId);
                    // if this keyframe is deteted in the serverMapx

                    start_t = clock();

                    cv::Mat To2n = oldTwc * newTcw;
                    cv::Mat Tnorm = cv::Mat::eye(4,4, newTcw.type());

                    flag = true;
                    {
                        std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
                        insertServerMapToGlobleMap(sMapx, Tnorm);
                        ifSubToGlobalMap[(*sMapx).pCacher->pClientId] = true;
                        subMapTransM[(*sMapx).pCacher->pClientId] = Tnorm;
                        pubToClient->transMs[ (*sMapx).pCacher->pClientId ] = Tnorm;
                        sMapx->clear();
                    }
                    {
                        std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
                        insertServerMapToGlobleMap( sMapy, To2n );
                        ifSubToGlobalMap[ (*sMapy).pCacher->pClientId ] = true;
                        subMapTransM[ (*sMapy).pCacher->pClientId ] = To2n;
                        pubToClient->transMs[ (*sMapy).pCacher->pClientId ] = To2n;
                        sMapy->clear();
                    }

                    time_t end_t = clock();
                    cout << "Fusing map use time is : " << ( double )( end_t - start_t ) / (double)CLOCKS_PER_SEC << "s \n";

                    mpGBA->CorrectLoop();

                    break;

                }

            }

        }

        return flag;

    }*/

    void MapFusion::insertServerMapToGlobleMap(ServerMap *sMap, cv::Mat To2n) {

        vector<KeyFrame * > allKFs = sMap->pMap->GetAllKeyFrames();

        vector<MapPoint * > allMPs = sMap->pMap->GetAllMapPoints();

        for( int mit = 0; mit < (int)allKFs.size(); mit ++ ) {

            KeyFrame * tKF = allKFs[ mit ];
            cv::Mat Tcw = tKF->GetPose();

            Tcw = Tcw * To2n;

            tKF->SetPose( Tcw );
            tKF->setCache( globalMap->pCacher );
            globalMap->pCacher->AddKeyFrameToMap( tKF );
            globalMap->pCacher->addKeyFrametoDB( tKF );

        }

        for( int mit = 0 ; mit < (int)allMPs.size(); mit ++ ) {

            MapPoint * tMP = allMPs[ mit ];

            cv::Mat tPose = tMP->GetWorldPos();
            cv::Mat Tn2o = cv::Mat::eye(4,4, To2n.type());
            cv::Mat tcw = To2n.rowRange(0, 3).col(3);
            cv::Mat Rwc = To2n.rowRange(0, 3).colRange(0, 3).t();
            tPose = Rwc * ( tPose - tcw );

            tMP->SetWorldPos( tPose );
            tMP->setCache( globalMap->pCacher );
            globalMap->pCacher->AddMapPointToMap( tMP );

        }

    }

    bool MapFusion::detectKeyFrameInServerMap(ServerMap *sMap, KeyFrame *tKF, cv::Mat &newPose, std::vector<KeyFrame *> &candidateKFs) {

        std::vector<KeyFrame*> vpCandidateKFs = sMap->DetectMapFusionCandidatesFromDB( tKF );

        if(vpCandidateKFs.empty())
            return false;

        const int nKFs = (int) vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75,true);

        vector<PnPsolver*> vpPnPsolvers;
        vpPnPsolvers.resize(nKFs);

        vector<vector<MapPoint*> > vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates=0;

        for(int i=0; i<nKFs; i++)
        {
            KeyFrame* pKF = vpCandidateKFs[i];
            if(pKF->isBad())
                vbDiscarded[i] = true;
            else
            {
                int nmatches = matcher.SearchByBoWInServer(pKF,tKF,vvpMapPointMatches[i]);
                if(nmatches<15)
                {
                    vbDiscarded[i] = true;
                    continue;
                }
                else
                {
                    PnPsolver* pSolver = new PnPsolver( (*tKF) ,vvpMapPointMatches[i]);

                    pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                    vpPnPsolvers[i] = pSolver;
                    nCandidates++;
                    candidateKFs.push_back( pKF );
                    
                    //delete pSolver;
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9,true);

        while(nCandidates>0 && !bMatch)
        {
            for(int i=0; i<nKFs; i++)
            {
                if(vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                PnPsolver* pSolver = vpPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if(bNoMore)
                {
                    vbDiscarded[i]=true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if(!Tcw.empty())
                {
                    Tcw.copyTo(newPose);

                    bMatch = true;
                    break;
                }
            }
        }

        if(!bMatch)
        {
            return false;
        }
        else
        {
            return true;
        }

    }


    void MapFusion::createSubServerMap(int mapId) {

        Cache * pCache = new Cache();

        pCache->mpVocabulary = mpVocabulary;

        pCache->createKeyFrameDatabase();

        pCache->createMap();

        ServerMap * subMap = new ServerMap( pCache, pCache->getMpMap() );

        serverMap[ mapId ] = subMap;
        
        //mpLoopCloser = new LoopClosing(serverMap[ mapId ]->pCacher, true);
        //mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);
    }

    bool MapFusion::loadORBVocabulary(const string &strVocFile) {

        //Load ORB Vocabulary

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if (!bVocLoad) {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }

        //ROS_INFO( "Vocabulary loaded!" );
        cout << "Vocabulary loaded" << endl;

        return bVocLoad;

    }

    void MapFusion::createKeyFrameDatabase() {
        //create new global server Map

        this->mpKeyFrameDatabase = new KeyFrameDatabase( *mpVocabulary );

        Cache * pCache = new Cache();

        pCache->mpVocabulary = mpVocabulary;

        pCache->createKeyFrameDatabase();

        pCache->createMap();

        pCache->pClientId = 0;

        globalMap = new ServerMap( pCache, pCache->getMpMap() );

        //pubToClient = new PubToClient( globalMap->pCacher );

        //global map view

        mpGlobalMapDrawer = new MapDrawer( pCache->getMpMap(), mpStrSettingPath );

        mpSMView = new ServerMapView(mpGlobalMapDrawer, mpStrSettingPath);

        mptViewer = new thread(&ServerMapView::Run, mpSMView);

        mpGBA = new GlobalOptimize( globalMap );

    }


}
