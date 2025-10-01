/*
 * @Author: ylh 
 * @Date: 2024-04-20 11:45:11 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-06-04 23:42:52
 */

#include "fusion_slam/modules/map/map.h"


Map::Map(const std::string &configPath){
    localMapPtr = pcl::make_shared<PCLPointCloud>();
    kdtreePtr = pcl::make_shared<KDTree>();
    globalMapPtr = pcl::make_shared<PCLPointCloud>();
    globalCloudPtr = pcl::make_shared<PCLPointCloud>();
    globalMapFilterResultPtr = pcl::make_shared<PCLPointCloud>();
    halfLocalMapLength = 50.0;
    saveMap = false;
    savePath = "";
    mapResolution = 0.5;
    YAML::Node configNode;
    configNode = YAML::LoadFile(configPath);
    mapResolution = configNode["map"]["map_resolution"].as<double>();
    halfLocalMapLength = configNode["map"]["local_map_length"].as<double>();
    halfLocalMapLength *= 0.5;
    saveMap = configNode["map"]["save_map"].as<bool>();
    savePath = configNode["map"]["save_path"].as<std::string>();
    float mapLeafSize = 0.2;
    mapLeafSize = configNode["map"]["map_leaf_size"].as<float>();
    globalMapPtrFilter.setLeafSize(mapLeafSize, mapLeafSize, mapLeafSize);
    LOG(INFO) <<"\033[1;32m"<< "map: " << "\033[0m" << std::endl;
    std::cout << "      map_resolution  : " << mapResolution << std::endl;
    std::cout << "      local_map_length: " << halfLocalMapLength*2.0 << std::endl;
    std::cout << "      save_map        : " << saveMap << std::endl;
    std::cout << "      save_path       : " << savePath << std::endl;
    std::cout << "      map_leaf_size   : " << mapLeafSize << std::endl;
}

Map::~Map(){}

int Map::addPcdAndUpdateLocalMap(PCLPointCloudPtr& cloud, const Eigen::Quaterniond& rot, const Eigen::Vector3d& pose){
    
        PCLPointCloud globalCloud;
        pcl::transformPointCloud(*cloud, globalCloud, quaternionTrans2Mat4d(rot, pose).cast<float>());
        if(saveMap){
            *globalCloudPtr = globalCloud;
            *globalMapPtr += *globalCloudPtr;
        }
        if(localMapPtr->points.empty()) {*localMapPtr = globalCloud;}
        else{
            for (auto &&point : globalCloud) {
                std::vector<int> ind;
                std::vector<float> distance;
                kdtreePtr->nearestKSearch(point, 5, ind, distance);
                if (distance[0] > mapResolution) localMapPtr->emplace_back(point);
            }         
            auto isInBox = [&](const PointType& pt) {
            return std::abs(pt.x - pose[0]) < halfLocalMapLength &&
            std::abs(pt.y - pose[1]) < halfLocalMapLength &&
            std::abs(pt.z - pose[2]) < halfLocalMapLength;
            };
            auto it = std::partition(localMapPtr->points.begin(), localMapPtr->points.end(), isInBox);
            localMapPtr->points.resize(it-localMapPtr->points.begin());
            localMapPtr->width = localMapPtr->points.size();
            localMapPtr->height = 1;
            localMapPtr->is_dense = true;
        }
        kdtreePtr->setInputCloud(localMapPtr);
        return 0;
}

int Map::reset(){
    localMapPtr->clear();
    globalMapPtr->clear();
    return 0;
}

PCLPointCloudConstPtr Map::getLocalMap(){
    return localMapPtr;
}

KDTreeConstPtr Map::readKDtree(){
    return kdtreePtr;
}

void Map::saveGlobalMap(){
    if(!saveMap) return;
    if(globalMapPtr->points.empty()){
        LOG(INFO) << "global map is empty" << std::endl;
        return;
    }
    globalMapPtrFilter.setInputCloud(globalMapPtr);
    globalMapPtrFilter.filter(*globalMapFilterResultPtr);
    if (pcl::io::savePCDFileASCII(savePath, *globalMapFilterResultPtr) == -1)
    {
        LOG(INFO) << " save fail ! " << std::endl;
    }else{
        LOG(INFO) << " save pcd success, save_path " << savePath << std::endl;
    }
}