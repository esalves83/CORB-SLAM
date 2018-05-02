//
// Created by lifu on 2017/6/4.
//

#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <strings.h>
#include <stdlib.h>
#include <string>

#include <boost/asio.hpp>

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

//#include<ros/ros.h>
//#include <cv_bridge/cv_bridge.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include <thread>

#include "MapFusion.h"
#include "ServerMap.h"

#define CHUNK_SIZE 6291456

using namespace std;

using namespace CORBSLAM_SERVER;

static int const maxConnectionBacklog = 5;

/*void call_from_thread(int connFd, int numAgent){
    // This function will be called from a thread
    
    int socket_buffer_size = 6291456;
    
    char *pch;
    
    char received[socket_buffer_size+1];
    bzero(received, socket_buffer_size+1);
    
    string temp;
    //temp.clear();
    
    std::size_t found;
    while(true)
    {    
        bzero(received, socket_buffer_size);        
        int recvd_size = read(connFd, received, socket_buffer_size);
        
        // Concaternar em um buffer de trabalho
        temp += received;
        
        // Verificar que tipo de mensagem foi recebida
        pch = strchr(received,'#');
        if(pch != NULL)
        {
            //cout << "Agente " << numAgent << " KFs" << temp.length() << " bytes" << endl;
            temp.pop_back();
            if(mapfusion->MapFusion::insertKeyFrameToMap(temp, numAgent))
                cout << "--Server Client[" << numAgent << "]: insert keyframe to map func. KFs count: " << temp.length() << " bytes"  << endl;
            
            temp.clear();
        }
        
        pch = strchr(received,'$');
        if(pch != NULL)
        {
            //cout << "Agente " << numAgent << " MPs" << temp.length() << " bytes" << endl;
            temp.pop_back();
            if(mapfusion->MapFusion::insertMapPointToMap(temp, numAgent))
                cout << "--Server Client[" << numAgent << "]: insert MapPoint to map func. MPs count: " << temp.length() << " bytes"  << endl;
            
            temp.clear();
        }
        
        found = temp.find("!!");
        if (found!=std::string::npos){
            
            temp.erase(temp.length()-2);
            if(mapfusion->MapFusion::updateKeyFrameToMap(temp, numAgent))
                cout << "--Server Client[" << numAgent << "]: update keyframe to map func. KFs count: " << temp.length() << " bytes"  << endl;
            
            temp.clear();
        }
        
        found = temp.find("&&");
        if (found!=std::string::npos){
            
            temp.erase(temp.length()-2);
            if(mapfusion->MapFusion::updateMapPointToMap(temp, numAgent))
                cout << "--Server Client[" << numAgent << "]: update map points to map func. KFs count: " << temp.length() << " bytes"  << endl;
            
            temp.clear();
            
            cout << "***************************" << endl;
        }
        
    }
    std::cout << "\nClosing thread and conn " << connFd << std::endl;
    //close(connFd);
}*/

int main(int argc, char **argv)
{
    //ros::init(argc, argv, "Corbslam_server");
    //ros::start();

    if( argc != 3) {
        cout << "lack of vocabulary path !!!\n";
        return false;
    }
    //ROS_INFO("Corbslam_server start!");
    cout << endl << "CORBSLAM Server start!" << endl << endl;

    // Create MapFusion system. It initializes all system threads and gets ready to process frames.

    std::string strSettingPath = argv[2];
    
    MapFusion* mapfusion = new MapFusion( strSettingPath );
    
    //ROS_INFO("MapFusion");

    mapfusion->loadORBVocabulary( argv[1] );
    
    //ROS_INFO("loadORBVocabulary");

    mapfusion->createKeyFrameDatabase();
    
    //ROS_INFO("createKeyFrameDatabase");

    /*ros::NodeHandle n;

    ros::ServiceServer InsertKeyFrameService = n.advertiseService("insertKeyFrameToMap", &MapFusion::insertKeyFrameToMap, mapfusion );
    
    ROS_INFO("insertKeyFrameToMap");

    ros::ServiceServer InsertMapPointService = n.advertiseService("insertMapPointToMap", &MapFusion::insertMapPointToMap, mapfusion);
    
    ROS_INFO("insertMapPointToMap");

    ros::ServiceServer updateKeyFrameToMapService = n.advertiseService("updateKeyFrameToMap", &MapFusion::updateKeyFrameToMap, mapfusion);
    
    ROS_INFO("updateKeyFrameToMap");

    ros::ServiceServer updateMapPointToMapService = n.advertiseService("updateMapPointToMap", &MapFusion::updateMapPointToMap, mapfusion);

    ROS_INFO("Publish services finished !");*/

    std::thread * mapFusionThread = new thread(&MapFusion::fuseSubMapToMap, mapfusion);

    // publish update keyframe and mappoint poses to client
    //std::thread * pubThread = new thread(&MapFusion::runPubTopic, mapfusion);
    std::thread * pubThread = new thread(&MapFusion::runUpdateToClient, mapfusion);

    // wait to get subcribe new keyframes or new mappoints
    //ros::MultiThreadedSpinner spinner(2);

    //spinner.spin();

    // Stop all threads

    // Save camera trajectory

    //ros::shutdown();
    
    socklen_t len; //store size of the address
    int connFd;
    struct sockaddr_in clntAdd;
    
    //create socket
    int socketId = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socketId < 0)
        std::cerr << "Cannot open socket" << std::endl;

    struct sockaddr_in serverAddr;
    bzero((char*)&serverAddr, sizeof(serverAddr));
    
    serverAddr.sin_family       = AF_INET;
    serverAddr.sin_port         = htons(8080);
    serverAddr.sin_addr.s_addr  = INADDR_ANY;
    
    //bind socket
    if (bind(socketId, (struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0)
        printf("deu erro!");
    
    if(listen(socketId, maxConnectionBacklog) != 0)
        printf("Erro ao escutar socket\n");
    
    int numAgent = 1;
    while(numAgent < 6){
        
        cout << "Listening..." << endl;
        
        len = sizeof(clntAdd);
        connFd = accept(socketId, (struct sockaddr *)&clntAdd, &len); // Blocking function
        
        if (connFd < 0)
            cout << "Can't receive data" << endl;
        else
            cout << "Accept connection from agent " << numAgent << endl;
        
        //std::thread teste(call_from_thread, connFd, numAgent);
        std::thread readFromClient(&MapFusion::runUpdateFromClient, mapfusion, connFd, numAgent);
        readFromClient.detach();
        
        //std::thread writeToClient(&MapFusion::runUpdateToClient, mapfusion, connFd, numAgent);
        //writeToClient.detach();
        
        numAgent++;
    }
    
    while(1);

    return 0;
}
