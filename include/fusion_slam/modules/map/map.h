/*
 * @Author: ylh 
 * @Date: 2024-04-20 10:37:19 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-06-04 22:38:01
 */
#ifndef MAP_H
#define MAP_H
#include "fusion_slam/type/pointcloud.h"
#include "fusion_slam/type/base_type.h"
#include "fusion_slam/math/math.h"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>

class Map{
private:
    PCLPointCloudPtr localMapPtr;
    KDTreePtr kdtreePtr;
    double mapResolution;
    double halfLocalMapLength;
    bool saveMap;
    std::string savePath;
    PCLPointCloudPtr globalMapPtr;
    PCLPointCloudPtr globalCloudPtr;
    PCLPointCloudPtr globalMapFilterResultPtr;
    VoxelFilter globalMapPtrFilter;
public:
    using Ptr = std::shared_ptr<Map>;
    Map(const std::string &configPath);
    ~Map();
    int addPcdAndUpdateLocalMap(PCLPointCloudPtr& cloud, const Eigen::Quaterniond& rot, const Eigen::Vector3d& pose);
    int reset();
    PCLPointCloudConstPtr getLocalMap();
    KDTreeConstPtr readKDtree();
    void saveGlobalMap();
};
#endif