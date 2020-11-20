/* @file  Namespace::class -> Camvox::PointCloude
 * @brief - 
 * @funct   
 * @funct   
 * @funct   
 * @funct   
 * @funct   
 * @funct     
 * @value - 
 * @state - 
 * @flag  - 
 * thread -
 */ 

#ifndef POINTCLOUDE_H
#define POINTCLOUDE_H

#include "pointcloudmapping.h"
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/core/core.hpp>
#include <mutex>

namespace Camvox
{
    class PointCloude                                                        //! need to be changed to PointCloud
    {
        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;

    public:
        PointCloud::Ptr pcE;

    public:
        Eigen::Isometry3d T;
        int pcID;
        //protected:
    };
} // namespace Camvox

#endif 
