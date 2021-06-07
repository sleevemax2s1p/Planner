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
class Timer
{
    typedef std::chrono::high_resolution_clock high_resolution_clock;
    typedef std::chrono::milliseconds milliseconds;

public:
    explicit Timer(bool run = false)
    {
        if (run)
            Reset();
    }
    void Reset()
    {
        _start = high_resolution_clock::now();
    }
    milliseconds Elapsed() const
    {
        return std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - _start);
    }

private:
    high_resolution_clock::time_point _start;
};
class Jps
{
private:
    std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();
    
    std::vector<Point *> path;
    int height;
    int width;
    int length;

    /////原来是 dim(0) x  dim(1) y  dim(2) z
    //现在是dim(0) z  dim(1) y dim(2) x
public:
    const vector<signed char> *map;
    vector<signed char> InitJps(vector<vector<vector<int>>> &_map) //初始化地图
    {
        int x = _map.size();

        int y = _map[0].size();

        int z = _map[0][0].size();
        
         vector<signed char>realmap(x * y * z);
        // cout << "original map" << endl;
        // for (int i = 0; i < _map.size(); i++)
        // {
        //     for (int j = 0; j < _map[0].size(); j++)
        //     {
        //         for (int k = 0; k < _map[0][0].size(); k++)
        //         {
        //             printf("%d", _map[i][j][k]);
        //             printf(" ");
        //         }
        //         cout << endl;
        //     }
        //     cout << " " << endl;
        // }
      
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
        

        // for (int k = 0; k < _map[0][0].size(); k++)
        //     for (int j = 0; j < _map[0].size(); j++)
        //         for (int i = 0; i < _map.size(); i++)
        //         {
        //             if (_map[i][j][k] == 1)
        //             {
        //              //   realmap[k* x * y + j * x + i] = 1;
        //                  realmap[index++] = 1;
        //                 // printf("curindex1=%d\n", i * h* w + j * h + k);
        //             }
        //             else if (_map[i][j][k] == 0)
        //             {
        //                 // realmap[k* x * y + j * x + i] = 0;
        //                   realmap[index++] = 0;
        //                 // printf("curindex0=%d\n",i * h* w + j * h + k);
        //             }
        //         }
        //    printf("start point(7,4,7)=%d\n",realmap[ 7+4*10+7*5*10]);
        // printf("start point(8,,7)=%d\n",_map[7][4][7]);
       
return realmap;
    //     map = realmap;
    //    printf("realmap[0]=%d      ",realmap[0]);
         
    }

    std::list<Point *> *getPath(Point *start_point, Point *end_point, double &time, double &distance)
    {
        std::list<Point *> *path = new std::list<Point *>();
        Vecf<3> origin{-0.5, -0.5, -0.5};
        Veci<3> _dim{10, 5, 14};  //x,y,z

     
        map_util->setMap(origin, _dim, *map, 1);

        // cout << "realmap" << endl;
        // for (int n = 0; n <( *map).size(); n++)
        // {
        //     if (n % 5 == 0)
        //         cout << endl;
        //         printf("%d      ",(*map)[n]);
        // }
        // cout<<"*mapSize="<<(*map).size()<<endl;
       
        const Vec3f start(start_point->x, start_point->y, start_point->z); 
        const Vec3f goal(end_point->x, end_point->y, end_point->z);

        std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(false)); // Declare a planner
        planner_ptr->setMapUtil(map_util);                                 // Set collision checking function
        planner_ptr->updateMap();
        
        ros::Time start_time = ros::Time::now();
        bool valid_jps = planner_ptr->plan(start, goal, 1, true); // Plan from start to goal using JPS
        time=(ros::Time::now()-start_time).toSec();
        distance = total_distance3f(planner_ptr->getRawPath());
        //printf("JPS Path Distance: %f\n", total_distance3f(planner_ptr->getRawPath()));

        //printf("JPS Path: \n");
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