/*
 * Copyright (c) 2013, Michael E. Ferguson
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * The names of the authors may not be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL AUTHORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <manipulation_msgs/GraspableObjectList.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

class ObjectSupportSegmention
{
public:
    ObjectSupportSegmention(ros::NodeHandle & n): nh_ (n), debug_(true)
    {
        ros::NodeHandle nh ("~");

        // frame_id: frame to transform cloud to (should be XY horizontal)
        frame_ = "base_link";
        nh.getParam("frame_id", frame_);

        // use_debug: enable/disable output of a cloud containing object points
        nh.getParam("use_debug", debug_);

        // cluster_tolerance: minimum separation distance of two objects
        double cluster_tolerance = 0.01;
        nh.getParam("cluster_tolerance", cluster_tolerance);
        extract_clusters_.setClusterTolerance(cluster_tolerance);

        // cluster_min_size: minimum size of an object
        int cluster_min_size = 50;
        nh.getParam("cluster_min_size", cluster_min_size);
        extract_clusters_.setMinClusterSize(cluster_min_size);

        // TODO: set limit
        range_filter_.setFilterFieldName("z");
        range_filter_.setFilterLimits(0, 2.5);

        // TODO
        voxel_grid_.setLeafSize(0.005f, 0.005f, 0.005f);
        voxel_grid_.setFilterFieldName("z");
        voxel_grid_.setFilterLimits(0, 1.8);

        segment_.setOptimizeCoefficients (true);
        segment_.setModelType (pcl::SACMODEL_PLANE);
        segment_.setMaxIterations (100);
        segment_.setDistanceThreshold (0.01);

        // clouds and objects
        cloud_sub_ = nh_.subscribe< pcl::PointCloud<pcl::PointXYZRGB> >("/head_camera/depth_registered/points",
                                                                        1,
                                                                        &ObjectSupportSegmention::cloudCallback,
                                                                        this);
        if(debug_)
            cloud_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("objects_cloud", 1);

        object_pub_ = nh_.advertise< manipulation_msgs::GraspableObjectList >("objects_detected", 1);
    }

    void cloudCallback ( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud )
    {
        ROS_INFO("Cloud recieved with %d points.", (int) cloud->points.size());
        // filter out noisy long-range points
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
        range_filter_.setInputCloud(cloud);
        range_filter_.filter(*cloud_filtered);
        ROS_INFO("Filtered for range, now %d points.", (int) cloud_filtered->points.size());

        // transform to grounded
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
        if (!pcl_ros::transformPointCloud (frame_, *cloud_filtered, *cloud_transformed, listener_))
        {
            ROS_ERROR ("Error transforming to frame %s", frame_.c_str () );
            return;
        }

        // voxel grid it
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);
        voxel_grid_.setInputCloud (cloud_transformed);
        voxel_grid_.filter (*cloud_filtered2);
        ROS_INFO("Filtered for transformed Z, now %d points.", (int) cloud_filtered2->points.size());

        // remove support planes
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        int thresh = cloud_filtered2->points.size()/8;
        while (true)
        {
            // Segment the largest planar component from the remaining cloud
            segment_.setInputCloud(cloud_filtered2);
            segment_.segment (*inliers, *coefficients);
            if (inliers->indices.size () < (size_t) thresh) // TODO: make configurable?
            {
                ROS_INFO("No more planes to remove.");
                break;
            }
            ROS_INFO("Removing a plane with %d points.", (int) inliers->indices.size());

            // Extract the non-planar parts
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud (cloud_filtered2);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*cloud_filtered2);
        }
        ROS_INFO("Cloud now %d points.", (int) cloud_filtered2->points.size());

        // cluster
        std::vector<pcl::PointIndices> clusters;
        extract_clusters_.setInputCloud (cloud_filtered2);
        extract_clusters_.extract (clusters);
        ROS_INFO("Extracted %d clusters.", (int) clusters.size());

        // ... and publish!
        pcl::PointCloud<pcl::PointXYZRGB> output;
        extract_indices_.setInputCloud(cloud_filtered2);
        manipulation_msgs::GraspableObjectList object_list;
        if(debug_)
            output.header.frame_id = cloud_transformed->header.frame_id;
        for(size_t i= 0; i < clusters.size(); i++)
        {
            manipulation_msgs::GraspableObject object;
            object.reference_frame_id = frame_;

            pcl::PointCloud<pcl::PointXYZRGB> new_cloud;
            extract_indices_.setIndices( pcl::PointIndicesPtr( new pcl::PointIndices (clusters[i])) );
            extract_indices_.filter(new_cloud);

            pcl::toROSMsg(new_cloud, object.region.cloud);
            object_list.graspable_objects.push_back(object);
            if(debug_)
            {
                ROS_INFO("Adding cluster of size %d.", (int) new_cloud.points.size());
                std::vector<pcl::PCLPointField> fields;
                pcl::getFields(new_cloud, fields);
                size_t idx;
                for (idx = 0; idx < fields.size(); idx++)
                {
                    if ( fields[idx].name == "rgb" || fields[idx].name == "rgba" )
                        break;
                }

                float hue = (360.0 / clusters.size()) * i;

                float r, g, b;
                hsv2rgb(hue, 0.8 /*saturation*/, 1.0 /*value*/, r, g, b);

                // colorize cloud
                for (size_t j = 0; j < new_cloud.points.size(); j++)
                {
                    pcl::PointXYZRGB &p = new_cloud.points[j];
                    unsigned char* pt_rgb = (unsigned char*) &p;
                    pt_rgb += fields[idx].offset;
                    (*pt_rgb) = (unsigned char) (r * 255);
                    (*(pt_rgb+1)) = (unsigned char) (g * 255);
                    (*(pt_rgb+2)) = (unsigned char) (b * 255);
                }
                output += new_cloud;
            }
        }
        object_pub_.publish(object_list);
        if(debug_)
            cloud_pub_.publish(output);
        ROS_INFO("Processing done.");
    }

    // see
    // http://en.wikipedia.org/wiki/HSL_and_HSV#Converting_to_RGB
    // for points on a dark background you want somewhat lightened
    // colors generally... back off the saturation (s)
    static void hsv2rgb(float h, float s, float v, float& r, float& g, float& b)
    {
        // pulled from ecto, thanks Troy!
        float c = v * s;
        float hprime = h/60.0;
        float x = c * (1.0 - fabs(fmodf(hprime, 2.0f) - 1));

        r = g = b = 0;

        if (hprime < 1) {
            r = c; g = x;
        } else if (hprime < 2) {
            r = x; g = c;
        } else if (hprime < 3) {
            g = c; b = x;
        } else if (hprime < 4) {
            g = x; b = c;
        } else if (hprime < 5) {
            r = x; b = c;
        } else if (hprime < 6) {
            r = c; b = x;
        }

        float m = v - c;
        r += m; g+=m; b+=m;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    ros::Publisher object_pub_;

    tf::TransformListener listener_;
    std::string frame_;

    pcl::PassThrough<pcl::PointXYZRGB> range_filter_;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_;
    pcl::SACSegmentation<pcl::PointXYZRGB> segment_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> extract_clusters_;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_;

    bool debug_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_support_segmentation");
    ros::NodeHandle n;
    ObjectSupportSegmention segmention(n);
    ros::spin();
    return 0;
}
