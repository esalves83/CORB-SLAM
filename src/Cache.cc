//
// Created by lifu on 17-1-31.
//

#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>

#include "System.h"
#include "Cache.h"
#include "Converter.h"
#include "DataDriver.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <time.h>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

//#include "ros/ros.h"

namespace ORB_SLAM2 {

    //construct function
    Cache::Cache() {

        mpMap = nullptr;
        mpKeyFrameDatabase = nullptr;
        mpVocabulary = nullptr;
        lKFToKFmap.clear();
        lMPToMPmap.clear();
        mbFinished = false;
        mbFinishedRunSubFromServer = false;
        mlNewKeyFrames.clear();
        mbStopRequested = false;
        mbNotStop = true;
        mbStopped = false;
        mbFinishRequested = false;
        mbFinishRequestedRunSubFromServer = false;
        kfStatus.clear();

        pClientId = -1;

    }

    bool Cache::loadORBVocabulary(const string &strVocFile) {

        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if (!bVocLoad) {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }

        cout << "Vocabulary loaded!" << endl << endl;

        return bVocLoad;

    }

    void Cache::createKeyFrameDatabase() {
        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    }

    void Cache::createMap() {
        //Create the Map
        mpMap = new Map();
    }

    void Cache::AddKeyFrameToMap(KeyFrame *pKF) {

            {
                // add KeyFrame to the LinghtKeyFrame to KeyFrame map
                unique_lock<mutex> lock(mMutexlKFToKFmap);
                //unique_lock<mutex> lock2(mpMap->mMutexMapUpdate);
                mpMap->AddKeyFrame(pKF);
                lKFToKFmap[pKF->mnId] = pKF;
                kfStatus[pKF->mnId] = KF_IN_CACHE;
                tmpKFMap.erase(pKF->mnId);

            }
            {
                unique_lock<mutex> lock(mMutexNewKFs);
                mlNewKeyFrames.push_back(pKF);
            }
            {
                unique_lock<mutex> lock(updateMPsToServerMutex);
                newInsertedKFs.insert(LightKeyFrame(pKF));
            }

    }

    void Cache::AddKeyFrameFromServer(KeyFrame *pKF) {
        {
            // add KeyFrame to the LinghtKeyFrame to KeyFrame map
            unique_lock<mutex> lock(mMutexlKFToKFmap);
            //unique_lock<mutex> lock2(mpMap->mMutexMapUpdate);
            mpMap->AddKeyFrame(pKF);
            lKFToKFmap[pKF->mnId] = pKF;
            kfStatus[pKF->mnId] = KF_IN_CACHE;
            tmpKFMap.erase(pKF->mnId);

        }
        {
            unique_lock<mutex> lock(mMutexNewKFs);
            mlNewKeyFrames.push_back(pKF);
        }

    }

    void Cache::AddMapPointFromServer(MapPoint *pMP) {

        mpMap->AddMapPoint(pMP);

        // add MapPoint to MapPoint map
        unique_lock<mutex> lock(mMutexMPToMPmap);
        lMPToMPmap[pMP->mnId] = pMP;

    }

    void Cache::EraseKeyFrameFromMap(KeyFrame *pKF) {

        // erase KeyFrame to the LinghtKeyFrame to KeyFrame
        {
            //unique_lock<mutex> lock2(mpMap->mMutexMapUpdate);
            unique_lock<mutex> lock(mMutexlKFToKFmap);
            lKFToKFmap.erase(pKF->mnId);
            kfStatus.erase(pKF->mnId);
        }

        mpMap->EraseKeyFrame(pKF);

    }

    void Cache::AddMapPointToMap(MapPoint *pMP) {

            mpMap->AddMapPoint(pMP);

            // add MapPoint to MapPoint map
            unique_lock<mutex> lock(mMutexMPToMPmap);
            lMPToMPmap[pMP->mnId] = pMP;

            {
                unique_lock<mutex> lock(updateMPsToServerMutex);
                newInsertedMPs.insert(LightMapPoint(pMP));
            }

    }

    void Cache::EraseMapPointFromMap(MapPoint *pMP) {

        {
            unique_lock<mutex> lock(mMutexMPToMPmap);
            //cout << "erase mp : " << pMP->mnId << "\n";
            lMPToMPmap.erase(pMP->mnId);
        }

        mpMap->EraseMapPoint(pMP);

    }

    std::map<long unsigned int, MapPoint *> Cache::getlMPToMPmap() {
        return lMPToMPmap;
    }

    KeyFrame *Cache::getKeyFrameById(long unsigned int pId) {

        if (pId <= 0) return nullptr;

        if (kfStatus.find(pId) == kfStatus.end()) return nullptr;

        KeyFrame *pKF = nullptr;

        {
            if (tmpKFMap.find(pId) != tmpKFMap.end()) {
                pKF = tmpKFMap[pId];
            } else if (lKFToKFmap.find(pId) != lKFToKFmap.end())
                pKF = lKFToKFmap[pId];

        }
        return pKF;
    }

    bool Cache::KeyFrameInCache(long unsigned int pID) {

        if (kfStatus.find(pID) != kfStatus.end())
            return kfStatus[pID] == KF_IN_CACHE;

        return false;

    }

    MapPoint *Cache::getMapPointById(long unsigned int pId) {

        if (pId <= 0) return nullptr;

        MapPoint *pMP = nullptr;
        {
            //unique_lock<mutex> lock(mMutexMPToMPmap);
            if (lMPToMPmap.find(pId) != lMPToMPmap.end()) {
                unique_lock<mutex> lock(mMutexMPToMPmap);
                pMP = lMPToMPmap[pId];
            }
        }

        return pMP;

    }

    std::vector<MapPoint *> Cache::GetAllMapPointsFromMap() {
        return mpMap->GetAllMapPoints();
    }

    void Cache::SetReferenceMapPointsToMap(std::vector<MapPoint *> pLocalMapPoints) {
        mpMap->SetReferenceMapPoints(pLocalMapPoints);
    }

    void Cache::SetmvpKeyFrameOrigins(KeyFrame *pKF) {
        mpMap->mvpKeyFrameOrigins.push_back(pKF);
    }

    vector<KeyFrame *> Cache::getmvpKeyFrameOrigins() {
        return mpMap->mvpKeyFrameOrigins;
    }

    void Cache::addUpdateKeyframe(KeyFrame *tKF) {
        if (tKF) {
            if (tKF->getFixed())
                return;
            else {
                unique_lock<mutex> lock(updateMPsToServerMutex);
                LightKeyFrame tLKF(tKF);
                if (newInsertedKFs.find(tLKF) == newInsertedKFs.end())
                    updateKFs.insert(tLKF);
            }
        }
    }

    void Cache::addUpdateMapPoint(MapPoint *tMP) {

        if (tMP) {
            if (tMP->getFixed()) {
                return;
            } else {
                unique_lock<mutex> lock(updateMPsToServerMutex);
                LightMapPoint tLMP(tMP);
                if (newInsertedMPs.find(tLMP) == newInsertedMPs.end())
                    updateMPs.insert(tLMP);
            }
        }
    }

    const int Cache::getKeyFramesInMap() {
        return mpMap->KeyFramesInMap();
    }

    vector<KeyFrame *> Cache::getAllKeyFramesInMap() {
        return mpMap->GetAllKeyFrames();
    }

    long unsigned int Cache::GetMaxKFidInMap() {
        return mpMap->GetMaxKFid();
    }

    void Cache::clearMap() {

        mpMap->clear();
        {
            unique_lock<mutex> lock(mMutexlKFToKFmap);
            lKFToKFmap.clear();
        }
        lMPToMPmap.clear();
        mlNewKeyFrames.clear();
        tmpKFMap.clear();
        kfStatus.clear();
    }

    void Cache::EraseKeyFrameFromDB(KeyFrame *pKF) {
        mpKeyFrameDatabase->erase(pKF);
    }

    vector<KeyFrame *> Cache::DetectRelocalizationCandidatesFromDB(Frame *F) {

        return mpKeyFrameDatabase->DetectRelocalizationCandidates(F);

    }

    vector<KeyFrame *> Cache::DetectLoopCandidatesFromDB(KeyFrame *pKF, float minScore) {
        return mpKeyFrameDatabase->DetectLoopCandidates(pKF, minScore);
    }

    void Cache::clearKeyframeDatabase() {

        mpKeyFrameDatabase->clear();
    }


    void Cache::addKeyFrametoDB(KeyFrame *pKF) {

        mpKeyFrameDatabase->add(pKF);

    }

    void Cache::SaveMap(const string &filename) {
        // save Map to files
        std::ofstream ofs("savetest.txt");
        boost::archive::text_oarchive oa(ofs);
        oa << mpMap;
        ofs.close();

    }

    void Cache::LoadMap(const string &filename) {
        // loadMap to files

        std::ifstream ifs("savetest.txt");
        boost::archive::text_iarchive ia(ifs);
        ia >> mpMap;
        ifs.close();

    }

    /*  cache organize function   */
    
    /*void Cache::RequestServerReset(){
        
        int n = write(socketId, "#end#", 5);
        if (n < 0) cout << "ERROR writing RequestServerReset" << endl;
    }*/

    void Cache::runUpdateToServer() {
        
        // Creates an endpoint for communication and returns a descriptor
        //int socketId = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
        socketId = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (socketId < 0)
            std::cerr << "Cannot open socket" << std::endl;
        
        struct sockaddr_in serverAddr;
        socklen_t addrSize = sizeof(serverAddr);
        bzero((char*)&serverAddr, sizeof(serverAddr)); // zero the struct
        
        serverAddr.sin_family       = AF_INET;
        serverAddr.sin_port         = htons(8080); // set destination port number
        serverAddr.sin_addr.s_addr  = htonl(INADDR_LOOPBACK); // set destination IP number - localhost, 127.0.0.1
        
        // Initiate a connection on a socket
        int isConnect = connect(socketId, (struct sockaddr*)&serverAddr, addrSize);
        if (isConnect < 0) 
            std::cerr << "Cannot connect to server!" << std::endl;

        while (true) {

            std::set<LightKeyFrame> tnewInsertedKFs;
            std::set<LightMapPoint> tnewInsertedMPs;
            std::set<LightKeyFrame> tupdateKFs;
            std::set<LightMapPoint> tupdateMPs;
            std::set<LightMapPoint> tdeletedMPs;
            {
                unique_lock<mutex> lock(updateMPsToServerMutex);

                tnewInsertedKFs = newInsertedKFs;
                //cout << "tnewInsertedKFs has " << tnewInsertedKFs.size() << " elements" << endl;
                newInsertedKFs.clear();
                
                tnewInsertedMPs = newInsertedMPs;
                //cout << "Going to send " << newInsertedMPs.size() << " elements to server" << endl;
                //cout << "newInsertedMPs has " << newInsertedMPs.size() << " elements" << endl;
                newInsertedMPs.clear();
                
                tupdateKFs = updateKFs;
                updateKFs.clear();
                
                tupdateMPs = updateMPs;
                updateMPs.clear();
                
                tdeletedMPs = deletedMPs;
                deletedMPs.clear();
            }
            
            //std::vector<KeyFrame *> KFs = getAllKeyFramesInMap();
            //cout << "Keyframes: " << KFs.size() << endl;
            
            //std::vector<MapPoint *> MPs = GetAllMapPointsFromMap();
            //cout << "Client map has " << MPs.size() << " map points" << endl;

            insertNewKeyFramesToServer(tnewInsertedKFs);
            
            usleep(500000);

            insertNewMapPointsToServer(tnewInsertedMPs);
            
            usleep(500000);
            
            deletedMapPointsToServer(tdeletedMPs);
            
            usleep(500000);

            updateKeyFramePosesToServer(tupdateKFs);
            
            usleep(500000);

            updateMapPointPosesToServer(tupdateMPs);


            if (isStopped()) {
                // Safe area to stop
                while (isStopped() && !CheckFinish()) {
                    usleep(3000);
                }
                if (CheckFinish())
                    break;
            }

            if (CheckFinish())
                break;

            usleep(2000000);
        }

        SetFinish();
        cout << "runUpdateToServer CLOSED" << endl;
        //close(socketId);
    }
    
    void Cache::deletedMapPointsToServer(std::set<LightMapPoint> tdeletedMPs) {

        //cout << "CacheUpdate : update MPs to server" << endl;
        
        std::vector< std::string > MPsData;
        
        MPsData.clear();

        for( std::set<LightMapPoint>::iterator mit = tdeletedMPs.begin(); mit != tdeletedMPs.end(); mit++ ) {

            //MapPoint *tMP = (*mit).getMapPoint();
            long unsigned int mnId = (*mit).mnMapPointId;

            

                std::ostringstream os;

                boost::archive::text_oarchive oa(os);

                oa << mnId;

                std::string ss = os.str();

                MPsData.push_back(ss);

                os.clear();
                ss.clear();
            

        }
        
        cout << "Send " << MPsData.size() << " deleted MPs to server" << endl;
        if(MPsData.size() > 0)
        {
            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << MPsData;
            
            std::string ss = os.str();
            ss.append("[");
        
            //std::string s(ss.length(),'2');
            //s.append("$");
        
            int n = write(socketId, ss.c_str(), ss.length());
            if (n < 0) cout << "ERROR writing to socket" << endl;
            
            os.clear();
            ss.clear();
            MPsData.clear();
            
            //cout << "inserted " << n << " bytes" << " new map points to server " << endl;
        }

        //DataDriver DB(this);

        //DB.insertNewMapPointsToServer(tInsertMPs);

    }
    
    

    void Cache::insertNewKeyFramesToServer(std::set<LightKeyFrame> tInsertKFs) {
        
        //cout << "CacheUpdate : update KFs to server" << endl;
        
        std::vector< std::string > KFsData;

        KFsData.clear();
        
        int count = 0;

        for( std::set<LightKeyFrame>::iterator mit = tInsertKFs.begin(); mit != tInsertKFs.end(); mit++ ) {

            KeyFrame * tKF = (*mit).getKeyFrame();

            if( tKF && !tKF->getFixed() ) {
                std::ostringstream os;

                boost::archive::text_oarchive oa(os);
                oa << tKF;

                std::string ss = os.str();

                KFsData.push_back( ss );

                os.clear();
                ss.clear();
            }
            count++;

        }
        
        //cout << "Size of KFsData: " << KFsData.size() << endl;
        if(KFsData.size() > 0)
        {
            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << KFsData;
            std::string ss = os.str();
            ss.append("*");
            
            //std::string s(ss.length(),'1');
            //s.append("#");
            
            /*std::size_t found = ss.find("#");
            if (found!=std::string::npos)
                cout << "Found # at: " << found << endl;*/
            
            int n = write(socketId, ss.c_str(), ss.length());
            if (n < 0) cout << "ERROR writing to socket" << endl;
            
            os.clear();
            ss.clear();
            KFsData.clear();
            
            //cout << "inserted " << n << " bytes" << " new keyframes to server " << endl;
        }

        //DataDriver DB(this);

        //DB.insertNewKeyFramesToServer(tInsertKFs);

    }

    void Cache::insertNewMapPointsToServer(std::set<LightMapPoint> tInsertMPs) {

        //cout << "CacheUpdate : update MPs to server" << endl;
        
        std::vector< std::string > MPsData;
        
        MPsData.clear();

        for( std::set<LightMapPoint>::iterator mit = tInsertMPs.begin(); mit != tInsertMPs.end(); mit++ ) {

            MapPoint *tMP = (*mit).getMapPoint();

            if( tMP && !tMP->getFixed() ) {

                std::ostringstream os;

                boost::archive::text_oarchive oa(os);

                oa << tMP;

                std::string ss = os.str();

                MPsData.push_back(ss);

                os.clear();
                ss.clear();
            }

        }
        
        cout << "Send " << MPsData.size() << " new MPs to server" << endl;
        if(MPsData.size() > 0)
        {
            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << MPsData;
            
            std::string ss = os.str();
            ss.append("(");
        
            //std::string s(ss.length(),'2');
            //s.append("$");
            
            /*std::size_t found = ss.find("$");
            if (found!=std::string::npos)
                cout << "Found $ at: " << found << endl;*/
        
            int n = write(socketId, ss.c_str(), ss.length());
            if (n < 0) cout << "ERROR writing to socket" << endl;
            
            os.clear();
            ss.clear();
            MPsData.clear();
            
            //cout << "inserted " << n << " bytes" << " new map points to server " << endl;
        }

        //DataDriver DB(this);

        //DB.insertNewMapPointsToServer(tInsertMPs);

    }

    void Cache::updateKeyFramePosesToServer(std::set<LightKeyFrame> tUpdateKFs) {
        
        //cout << "CacheUpdate : update poses of KFs to server" << endl;

        std::vector< std::string > KFsData;

        KFsData.clear();

        for( std::set<LightKeyFrame>::iterator mit = tUpdateKFs.begin(); mit != tUpdateKFs.end(); mit++ ) {

            KeyFrame * tKF = (*mit).getKeyFrame();

            if( tKF && !tKF->getFixed() ) {
                KeyFramePose* tKFP = new KeyFramePose( tKF );

                std::ostringstream os;

                boost::archive::text_oarchive oa(os);
                oa << tKFP ;

                std::string ss = os.str();

                KFsData.push_back( ss );

                os.clear();
                ss.clear();
                
                //delete tKFP;
            }
        }

        //cout << "Size of Updated KFsData: " << KFsData.size() << endl;
        if(KFsData.size() > 0)
        {
            std::ostringstream os;

            boost::archive::text_oarchive oa(os);

            oa << KFsData;

            std::string ss = os.str();
            ss.append(")");
            
            //std::string s(ss.length(),'3');
            //s.append("&");
            
            /*std::size_t found = ss.find("!!");
                if (found!=std::string::npos)
                    cout << "Found !! at: " << found << endl;*/
                
            int n = write(socketId, ss.c_str(), ss.length());
            if (n < 0) cout << "ERROR writing to socket" << endl;
            
            os.clear();
            ss.clear();
            KFsData.clear();
                
            //cout << "updated " << n << " bytes" << " keyframes to server " << endl;
        }        

        //ROS_INFO("CacheUpdate : update poses of KFs to server");

        //DataDriver DB(this);

        //DB.updateKeyFramePosesToServer(tUpdateKFs);

    }

    void Cache::updateMapPointPosesToServer(std::set<LightMapPoint> tUpdateMPs) {
        
        //cout << "CacheUpdate : update poses of MPs to server" << endl;

        std::vector< std::string > MPsData;

        MPsData.clear();

        for( std::set<LightMapPoint>::iterator mit = tUpdateMPs.begin(); mit != tUpdateMPs.end(); mit++ ) {

            MapPoint *tMP = (*mit).getMapPoint();

            if( tMP && !tMP->getFixed()) {

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
        
        cout << "Send " << MPsData.size() << " updated MPs to server" << endl;
        if(MPsData.size() > 0)
        {
            std::ostringstream os;

            boost::archive::text_oarchive oa(os);

            oa << MPsData;

            std::string ss = os.str();
            ss.append("?");
            
            /*std::size_t found = ss.find("&&");
                if (found!=std::string::npos)
                    cout << "Found && at: " << found << endl;*/
                    
            int n = write(socketId, ss.c_str(), ss.length());
            if (n < 0) cout << "ERROR writing to socket" << endl;
            
            os.clear();
            ss.clear();
            MPsData.clear();
            
            //cout << "updated " << n << " bytes" << " map points to server " << endl;
        }
        

        //ROS_INFO("CacheUpdate : update poses of MPs to server");

        //DataDriver DB(this);

        //DB.updateMapPointPosesToServer(tUpdateMPs);


    }

    void Cache::runSubFromServer() {        
        
        int socket_buffer_size = 7000000;    
        //char *pch;        
        char received[socket_buffer_size+1];
        bzero(received, socket_buffer_size+1);        
        std::string temp;        
        //std::size_t found;
        
        while(true)
        {    
            bzero(received, socket_buffer_size);        
            int recvd_size = read(socketId, received, socket_buffer_size);
            
            //cout << recvd_size << " bytes received" << endl;
            
            // Concaternar em um buffer de trabalho
            temp += received;
            
            std::size_t found = temp.find_first_of("#$@=*");
            while (found!=std::string::npos)
            {
                if(temp[found] == '#')
                {
                    std::string aux = temp.substr(0,found);
                    //cout << "Received " << aux.length() << " bytes of TransMs from server" << endl;
                    
                    std::map<int, cv::Mat> mtransMs; // para deserializar
                    {
                        std::stringstream iiis(aux);
                        boost::archive::text_iarchive iiia(iiis);
                        iiia >> mtransMs;
                        iiis.clear();
                    }
                    transMs = mtransMs;
                    
                    temp.erase(0,found+1);
                }
                
                if(temp[found] == '$')
                {
                    std::string aux = temp.substr(0,found);
                    //cout << "Received " << aux.length() << " bytes of newKFs from server" << endl;
                    
                    subNewInsertKFs(aux);
                    
                    temp.erase(0,found+1);
                }
                
                if(temp[found] == '@')
                {
                    std::string aux = temp.substr(0,found);
                    //cout << "Received " << aux.length() << " bytes of newMPs from server" << endl;
                    
                    subNewInsertMPs(aux);
                    
                    temp.erase(0,found+1);
                }
                
                if(temp[found] == '=')
                {
                    std::string aux = temp.substr(0,found);
                    //cout << "Received " << aux.length() << " bytes of upKFs from server" << endl;
                    
                    subUpInsertKFs(aux);
                    
                    temp.erase(0,found+1);
                }
                
                if(temp[found] == '*')
                {
                    std::string aux = temp.substr(0,found);
                    //cout << "Received " << aux.length() << " bytes of upMPs from server" << endl;
                    
                    subUpInsertMPs(aux);
                    
                    temp.erase(0,found+1);
                }                
                
                found=temp.find_first_of("#$@=*");
            }
            
            /*std::vector<std::string> v;
            boost::split(v, temp, boost::is_any_of("#$@=*"));
            
            cout << "Size of vector " << v.size() << endl;
            v.clear();
            temp.clear();*/
            
            
            /*found = temp.find("##");
            if(found!=std::string::npos)
            {                
                std::string aux = temp.substr(0,found); //All characters before ##
                
                cout << "Received " << aux.length() << " bytes of TransMs from server" << endl;
                
                std::map<int, cv::Mat> mtransMs; // para deserializar
                {
                    std::stringstream iiis(aux);
                    boost::archive::text_iarchive iiia(iiis);
                    iiia >> mtransMs;
                    iiis.clear();
                }
                transMs = mtransMs;
                
                temp.erase(0,found+2); // Erase all characters up to ##, including ##
            }
            
            found = temp.find("$$");
            if(found!=std::string::npos)
            {
                std::string aux = temp.substr(0,found); //All characters before $$
                cout << "Received " << aux.length() << " bytes of newKFs from server" << endl;
                
                subNewInsertKFs(aux);                
                
                //temp.clear();
                temp.erase(0,found+2);
            }
            
            found = temp.find("@@");
            if(found!=std::string::npos)
            {
                std::string aux = temp.substr(0,found); //All characters before $$
                cout << "Received " << aux.length() << " bytes of newMPs from server" << endl;                
                
                subNewInsertMPs(aux);
                
                //temp.clear();
                temp.erase(0,found+2);
            }
            
            found = temp.find("==");
            if(found!=std::string::npos)
            {
                std::string aux = temp.substr(0,found); //All characters before $$
                cout << "Received " << aux.length() << " bytes of update KFs from server" << endl;                
                
                subUpInsertKFs(aux);
                
                //temp.clear();
                temp.erase(0,found+2);
            }
            
            found = temp.find("--");
            if(found!=std::string::npos)
            {
                std::string aux = temp.substr(0,found); //All characters before $$
                cout << "Received " << aux.length() << " bytes of update MPs from server" << endl;
                
                subUpInsertMPs(aux);
                
                temp.erase(0,found+2);
            }*/
        }
    }
    
    void Cache::subNewInsertKFs(std::string str)
    {
        unique_lock<mutex> lock(mMutexNewKFsQueue);
        mlNewKFsQueue.push_back(str);
    }
    
    void Cache::subNewInsertMPs(std::string str)
    {
        unique_lock<mutex> lock(mMutexNewMPsQueue);
        mlNewMPsQueue.push_back(str);
    }
    
    void Cache::subUpInsertKFs(std::string str)
    {
        unique_lock<mutex> lock(mMutexUpKFsQueue);
        mlUpKFsQueue.push_back(str);
    }
    
    void Cache::subUpInsertMPs(std::string str)
    {
        unique_lock<mutex> lock(mMutexUpMPsQueue);
        mlUpMPsQueue.push_back(str);
    }
    
    bool Cache::CheckNewKFs()
    {
        unique_lock<mutex> lock(mMutexNewKFsQueue);
        return(!mlNewKFsQueue.empty());
    }
    
    bool Cache::CheckNewMPs()
    {
        unique_lock<mutex> lock(mMutexNewMPsQueue);
        return(!mlNewMPsQueue.empty());
    }
    
    bool Cache::CheckUpKFs()
    {
        unique_lock<mutex> lock(mMutexUpKFsQueue);
        return(!mlUpKFsQueue.empty());
    }
    
    bool Cache::CheckUpMPs()
    {
        unique_lock<mutex> lock(mMutexUpMPsQueue);
        return(!mlUpMPsQueue.empty());
    }
    
    void Cache::InsertDataFromServer(){
        
        while(true){
            
            std::string data;
            
            // Check if there are data in the queue
            if(CheckNewKFs()){
                //cout << "New KFs availabe at queue and spent ";
                {
                    unique_lock<mutex> lock(mMutexNewKFsQueue);
                    data = mlNewKFsQueue.front();
                    mlNewKFsQueue.pop_front();
                }
                    
                subNewInsertKeyFramesFromServer(data);
            }
            
            if(CheckNewMPs()){
                //cout << "New MPs availabe at queue and spent ";
                {
                    unique_lock<mutex> lock(mMutexNewMPsQueue);
                    data = mlNewMPsQueue.front();
                    mlNewMPsQueue.pop_front();
                }
                    
                subNewInsertMapPointFromServer(data);
            }
            
            if(CheckUpKFs()){

                {
                    unique_lock<mutex> lock(mMutexUpKFsQueue);
                    data = mlUpKFsQueue.front();
                    mlUpKFsQueue.pop_front();
                }
                    
                subUpdatedKeyFramesPose(data);
            }
            
            if(CheckUpMPs()){

                {
                    unique_lock<mutex> lock(mMutexUpMPsQueue);
                    data = mlUpMPsQueue.front();
                    mlUpMPsQueue.pop_front();
                }
                    
                subUpdatedMapPointsPose(data);
            }
            
            usleep(500000);
            
            if (CheckFinishRunSubFromServer())
                break;
        }
        
        SetFinishRunSubFromServer();
        cout << "SetFinishRunSubFromServer..." << endl;
    }

    void Cache::subNewInsertKeyFramesFromServer(std::string data) {

        //ROS_INFO("sub New Insert KeyFrames From Server");

        /*std::map<int, cv::Mat> transMs;
        std::stringstream iiis(msg->TRANSM);
        boost::archive::text_iarchive iiia(iiis);
        iiia >> transMs;
        iiis.clear();*/
        
        std::vector<string> elementVec;

        if( transMs.find( this->pClientId ) == transMs.end() )
        {
            cout << "Return from insert KFs" << endl;
            return ;
        }
        cv::Mat Ttrans = transMs[ this->pClientId ];
        
        {
            std::stringstream is(data);
            boost::archive::text_iarchive ia(is);
            
            ia >> elementVec;
            is.clear();
        }
        
        //cout << "get new keyframes from server : " << elementVec.size() << endl;
        for (int mit = 0; mit < (int) elementVec.size(); mit++) {
            try {
                ORB_SLAM2::KeyFrame *tKF = new ORB_SLAM2::KeyFrame();
                {
                    std::stringstream iis(elementVec[mit]);
                    boost::archive::text_iarchive iia(iis);
                    iia >> tKF;
                }
                
                if (tKF && (tKF->mnClientId != this->pClientId)) {

                    if( this->getKeyFrameById(tKF->mnId) )
                        continue;

                    tKF->setCache(this);

                    cv::Mat Tcw = tKF->GetPose();
                    Tcw = Tcw * Ttrans.inv();
                    tKF->SetPose( Tcw );

                    tKF->setFixed();
                    this->AddKeyFrameFromServer(tKF);
                    this->addKeyFrametoDB(tKF);
                    
                    //delete tKF;
                }
            } catch (...) {
                cout << "error in : " << mit << endl;
            }
        }
        elementVec.clear();

    }

    void Cache::subNewInsertMapPointFromServer(std::string data) {

        //ROS_INFO("sub New Insert MapPoint From Server");

        /*std::map<int, cv::Mat> transMs;
        std::stringstream iiis(msg->TRANSM);
        boost::archive::text_iarchive iiia(iiis);
        iiia >> transMs;
        iiis.clear();*/
        
        std::vector<string> elementVec;

        if( transMs.find( this->pClientId ) == transMs.end() )
        {
            cout << "Return from insert MPs" << endl;
            return ;
        }
        
        cv::Mat Ttrans = transMs[ this->pClientId ];

        {std::stringstream is(data);
        boost::archive::text_iarchive ia(is);
        
        ia >> elementVec;
        is.clear();}
        
        int count = 0;

        //cout << "Get new map points from server : " << elementVec.size() << endl;
        for (int mit = 0; mit < (int) elementVec.size(); mit++) {
            try {
                ORB_SLAM2::MapPoint *tMP = new ORB_SLAM2::MapPoint();
                {std::stringstream iis(elementVec[mit]);
                boost::archive::text_iarchive iia(iis);
                iia >> tMP;}
                if (tMP && (tMP->mnClientId != this->pClientId)) {

                    if( this->getMapPointById( tMP->mnId ) )
                        continue;

                    tMP->setCache(this);

                    cv::Mat tPose = tMP->GetWorldPos();
                    cv::Mat tcw = Ttrans.rowRange(0, 3).col(3);
                    cv::Mat Rcw = Ttrans.rowRange(0, 3).colRange(0, 3);
                    tPose = Rcw * tPose + tcw ;
                    tMP->SetWorldPos( tPose );

                    tMP->setFixed();
                    this->AddMapPointFromServer(tMP);
                    
                    count++;
                }
            } catch (...) {
                cout << "error in : " << mit << endl;
            }
        }
        //cout << "Inserted " << count << "new MPs" << endl;
        elementVec.clear();

    }

    void Cache::subUpdatedKeyFramesPose(std::string data) {

        /*ROS_INFO("sub Updated KeyFrames Pose");

        std::map<int, cv::Mat> transMs;
        std::stringstream iiis(msg->TRANSM);
        boost::archive::text_iarchive iiia(iiis);
        iiia >> transMs;
        iiis.clear();*/

        if( transMs.find( this->pClientId ) == transMs.end() )
        {
            cout << "Return from update KFs" << endl;
            return ;
        }
        cv::Mat Ttrans = transMs[ this->pClientId ];

        std::vector<string> elementVec;
        elementVec.clear();
        {std::stringstream is(data);
        boost::archive::text_iarchive ia(is);
        ia >> elementVec;
        is.clear();}

        for (int mit = 0; mit < (int) elementVec.size(); mit++) {
            try {
                ORB_SLAM2::KeyFramePose *tKFP = new ORB_SLAM2::KeyFramePose();
                {std::stringstream iis(elementVec[mit]);
                boost::archive::text_iarchive iia(iis);
                iia >> tKFP;}
                if (tKFP) {
                    ORB_SLAM2::KeyFrame *tKF = this->getKeyFrameById(tKFP->KFPId);
                    if (tKF && tKF->getFixed()) {
                        cv::Mat Tcw = tKFP->KFPose;
                        Tcw = Tcw * Ttrans.inv();
                        tKF->SetPose(Tcw);
                    }
                }
                //delete tKFP;
            } catch (...) {
                cout << "error in : " << mit << endl;
            }
        }

        elementVec.clear();

    }

    void Cache::subUpdatedMapPointsPose(std::string data) {

        /*ROS_INFO("sub Updated MapPoints Pose");

        std::map<int, cv::Mat> transMs;
        std::stringstream iiis(msg->TRANSM);
        boost::archive::text_iarchive iiia(iiis);
        iiia >> transMs;
        iiis.clear();*/

        if( transMs.find( this->pClientId ) == transMs.end() )
        {
            cout << "Return from update MPs" << endl;
            return ;
        }
        cv::Mat Ttrans = transMs[ this->pClientId ];

        std::vector<string> elementVec;
        elementVec.clear();
        {std::stringstream is(data);
        boost::archive::text_iarchive ia(is);
        ia >> elementVec;
        is.clear();}
        
        int count = 0;
        
        //cout << "get updated map points from server : " << elementVec.size() << endl;

        for (int mit = 0; mit < (int) elementVec.size(); mit++) {
            try {
                ORB_SLAM2::MapPointPose *tMPP = new ORB_SLAM2::MapPointPose();
                {std::stringstream iis(elementVec[mit]);
                boost::archive::text_iarchive iia(iis);
                iia >> tMPP;}
                if (tMPP) {
                    ORB_SLAM2::MapPoint *tMP = this->getMapPointById(tMPP->MPPId);
                    if (tMP && tMP->getFixed() ) {

                        cv::Mat tPose = tMPP->MPPose;
                        cv::Mat tcw = Ttrans.rowRange(0, 3).col(3);
                        cv::Mat Rcw = Ttrans.rowRange(0, 3).colRange(0, 3);
                        tPose = Rcw * tPose + tcw;
                        tMP->SetWorldPos( tPose );
                    }
                }
                //delete tMPP;
            } catch (...) {
                cout << "error in : " << mit << endl;
            }
        }
        //cout << "Inserted " << count << "updated MPs" << endl;

        elementVec.clear();

    }


    void Cache::insertTmpKeyFrame(KeyFrame *pKF) {

        if (pKF) {

            unique_lock<mutex> lock(mMutexTmpKFMap);
            tmpKFMap[pKF->mnId] = pKF;
            kfStatus[pKF->mnId] = KF_IN_CACHE;

        }

    }

    void Cache::eraseTmpKeyFrame(long unsigned int pID) {
        unique_lock<mutex> lock(mMutexTmpKFMap);
        if (tmpKFMap.find(pID) != tmpKFMap.end()) {

            tmpKFMap.erase(pID);

        }
    }


    bool Cache::isStopped() {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Cache::CheckFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }
    
    bool Cache::CheckFinishRunSubFromServer() {
        unique_lock<mutex> lock(mMutexFinishRunSubFromServer);
        return mbFinishRequestedRunSubFromServer;
    }

    void Cache::SetFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
        unique_lock<mutex> lock2(mMutexStop);
        mbStopped = true;
    }
    
    void Cache::SetFinishRunSubFromServer() {
        unique_lock<mutex> lock(mMutexFinishRunSubFromServer);
        mbFinishedRunSubFromServer = true;
        //unique_lock<mutex> lock2(mMutexStop);
        //mbStopped = true;
    }

    void Cache::RequestFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }
    
    void Cache::RequestFinishRunSubFromServer() {
        
        while(CheckNewKFs() || CheckNewMPs() || CheckUpKFs() || CheckUpMPs())
            usleep(5000);
        
        {
            unique_lock<mutex> lock(mMutexFinishRunSubFromServer);
            mbFinishRequestedRunSubFromServer = true;
        }
    }

    bool Cache::isFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }
    
    bool Cache::isFinishedRunSubFromServer() {
        unique_lock<mutex> lock(mMutexFinishRunSubFromServer);
        return mbFinishedRunSubFromServer;
    }

    bool Cache::Stop() {
        unique_lock<mutex> lock(mMutexStop);
        if (mbStopRequested && !mbNotStop) {
            mbStopped = true;
            cout << "Cache STOP" << endl;
            return true;
        }

        return false;
    }

}