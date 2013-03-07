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
 *     * Neither the name of Vanadium Labs LLC nor the names of its 
 *       contributors may be used to endorse or promote products derived 
 *       from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <tf/tf.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <nav_msgs/OccupancyGrid.h>

/*
 * Read an OctoMap file, publish a 2d map.
 * TODO: read OctoMaps from a topic -- octomap_server depends on arm_navigation currently :(
 */
int main( int argc, char* argv[] )
{
    double resolution_, height_;

    /* Need a name of an octomap */
    if (argc < 2)
    {
        std::cout << "ERROR: Usage is octo_to_2d <file.ot>" << std::endl;
        return -1;
    }
    const std::string octomap_filename = argv[1];

    /* This is a node, create it, load params */
    ros::init( argc, argv, "octo_to_2d" );
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    /* resolution of output 2d map */
    nh.param("map2d_resolution", resolution_, 0.05); 

    /* z-height of output 2d map, default to height of Turtlebot 3dSensor Optical Frame */
    nh.param("map2d_height", height_, 0.287);    

    /* Load the Octomap */
    octomap::ColorOcTree* tree = dynamic_cast<octomap::ColorOcTree*>(octomap::AbstractOcTree::read(octomap_filename));
    // TODO: handle errors?

    /* Create Map Publisher (latched) */
    ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

    /* Get coordinates for map2d */
    double x0, y0, z0;
    double x1, y1, z1;
    tree->getMetricMin(x0, y0, z0);
    tree->getMetricMax(x1, y1, z1);

    /* Fill in the map2d header */
    nav_msgs::OccupancyGrid map;
    map.info.resolution = resolution_;
    map.info.width = (int) (x1-x0)/resolution_;
    map.info.height = (int) (y1-y0)/resolution_;
    map.info.origin.position.x = x0;
    map.info.origin.position.y = y0;
    map.info.origin.position.z = 0.0;
    /* TODO: is there some notion of rotation we really need, 
             or can it just be 0,0,0,1 below? */   
    tf::Quaternion q;
    q.setRPY(0,0,0);
    map.info.origin.orientation.x = q.x();
    map.info.origin.orientation.y = q.y();
    map.info.origin.orientation.z = q.z();
    map.info.origin.orientation.w = q.w();

    /* Fill in the map2d data */
    map.data.resize(map.info.width * map.info.height);
    for(int y = 0; y < map.info.height; y++)
    {
        for(int x = 0; x < map.info.width; x++)
        {
            /* TODO: should we stop at some depth to ensure that we don't 
                     mark things open when actually we have something inside? */
            octomap::ColorOcTreeNode *n = tree->search(x0 + resolution_*x, y0 + resolution_*y, height_);
            if(n)
            {
                if(tree->isNodeOccupied(n))
                    map.data[map.info.width * y + x] = 100;
                else
                    map.data[map.info.width * y + x] = 0;
            }
            else
            {
                map.data[map.info.width * y + x] = -1;
            }
        }
    }

    /* Publish the Map */
    pub.publish(map);

    ros::spin();
    return 0;
}
