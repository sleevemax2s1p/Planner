#ifndef __JPS_H
#define __JPS_H

#include <jps_basis/data_utils.h>
#include <ros/ros.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_collision/map_util.h>
#include <jps_basis/data_type.h>
#include "planner.h"
#include <list>
#include <chrono>
using namespace std;
using namespace JPS;

class Jps
{
private:
    std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();

    std::vector<Point *> path;
    int xDim;
    int yDim;
    int zDim;

    /////原来是 dim(0) x  dim(1) y  dim(2) z
    //现在是dim(0) z  dim(1) y dim(2) x
public:
    const vector<signed char> *map;
    vector<signed char> InitJps(vector<vector<vector<int>>> &_map) //初始化地图
    {
        xDim = _map.size();

        yDim = _map[0].size();

        zDim = _map[0][0].size();

        vector<signed char> realmap(xDim * yDim * zDim);

        int index = 0; //转换有问题******************************
        for (int k = 0; k < _map[0][0].size(); k++)
            for (int j = 0; j < _map[0].size(); j++)
                for (int i = 0; i < _map.size(); i++)

                {
                    if (_map[i][j][k] == 1)
                    {
                        realmap[index++] = 1;

                        // printf("curindex1=%d\n", i * h* w + j * h + k);
                    }
                    else if (_map[i][j][k] == 0)
                    {
                        realmap[index++] = 0;
                        // printf("curindex0=%d\n",i * h* w + j * h + k);
                    }
                }

       
        return realmap;
        
    }

    std::list<Point *> *getPath(Point *start_point, Point *end_point, double &time, double &distance)
    {
        std::list<Point *> *path = new std::list<Point *>();
        Vecf<3> origin{-0.5, -0.5, -0.5};
        Veci<3> _dim{xDim, yDim, zDim}; //x,y,z

        map_util->setMap(origin, _dim, *map, 1);

        

        const Vec3f start(start_point->x, start_point->y, start_point->z);
        const Vec3f goal(end_point->x, end_point->y, end_point->z);

        std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(false)); // Declare a planner
        planner_ptr->setMapUtil(map_util);                                  // Set collision checking function
        planner_ptr->updateMap();

        ros::Time start_time = ros::Time::now();
        bool valid_jps = planner_ptr->plan(start, goal, 1, true); // Plan from start to goal using JPS
        time = (ros::Time::now() - start_time).toSec();
        distance = total_distance3f(planner_ptr->getRawPath());
        
        auto path_jps = planner_ptr->getRawPath();
        for (const auto &it : path_jps)
        {
            //std::cout << it.transpose()(0) << "  " << it.transpose()(1) << "  " << it.transpose()(2) << std::endl;
            int x = int(it.transpose()(0));
            int y = int(it.transpose()(1));
            int z = int(it.transpose()(2));
            Point *node = new Point(x, y, z);
            path->push_back(node); //将数据放到path里面去
        }
        return path;
    }
};
#endif