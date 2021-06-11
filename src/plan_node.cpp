
#include "planner/A_star.h"
#include "planner/Map_Construct.h"
#include "planner/visualization.h"
#include "planner/jps.h"
#include <fstream>
#define USING_JPS
#define RADOM_DATA
#define MULTEST
int main(int argc, char **argv)
{

    const char file_name[] = "/home/ryan/course/src/Planner/src/test2.txt";
    const string csvFilename ="/home/ryan/course/src/Planner/src/result.csv";
   
    ofstream outfile(csvFilename,   ios::trunc);
    outfile << "JPS Planner"<<endl<<"index,"
            << "plannerTime,"
            << "plannerDistance" << endl;
    double *avgTime = new double();
    double *avgDistance = new double();
    double *time = new double();
    *avgTime = 0;
    *avgDistance = 0;
#ifdef USING_JPS
    double *distance = new double();
#else
    int *distance = new int();
#endif
    ros::init(argc, argv, "plan_node");
    ros::NodeHandle nh;

#ifndef RADOM_DATA
    nh.param("sx", sx, 7);
    nh.param("ex", ex, 0);
    nh.param("sy", sy, 4);
    nh.param("ey", ey, 0);
    nh.param("sz", sz, 7);
    nh.param("ez", ez, 0);

    vector<vector<vector<int>>> map_data(10, vector<vector<int>>(5, vector<int>(14)));
    map_data[0] = {{0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1},
                   {1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                   {1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0},
                   {1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0}};
    map_data[1] = {{1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1},
                   {1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
                   {0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0},
                   {1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0},
                   {1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0}};
    map_data[2] = {{1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0},
                   {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
                   {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0}};
    map_data[3] = {{0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1},
                   {1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0},
                   {0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0},
                   {1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0},
                   {1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0}};
    map_data[4] = {{1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1},
                   {0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0},
                   {1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1},
                   {0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1},
                   {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0}};
    map_data[5] = {{0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0},
                   {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1}};
    map_data[6] = {{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                   {1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0},
                   {1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0},
                   {1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
    map_data[7] = {{1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0},
                   {1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0},
                   {0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0},
                   {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}};
    map_data[8] = {{0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1},
                   {1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                   {1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0}};
    map_data[9] = {{1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
                   {1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
                   {1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0},
                   {0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
#else
    ifstream infile(file_name);
    int T;
    infile >> T; //路径规划的次数
    cout<<"t="<<T<<endl;
    // if (T > 1)
    // {
    //     #define MULTEST;
    // }
    for (int i = 0; i < T; i++)
    {
        int X, Y, Z;
        int spx, spy, spz, epx, epy, epz;
        infile >> X >> Y >> Z;
        infile >> spx >> spy >> spz >> epx >> epy >> epz;
        cout << "X Y Z  " << X << "\t" << Y << "\t" << Z << endl;
        cout << "spx:" << spx << "\t"
             << "spy:" << spy << "\t"
             << "spz:" << spz << endl;
        cout << "epx:" << epx << "\t"
             << "epy:" << epy << "\t"
             << "epz:" << epz << endl;
        nh.param("sx", sx, spx);
        nh.param("ex", ex, epx);
        nh.param("sy", sy, spy);
        nh.param("ey", ey, epy);
        nh.param("sz", sz, spz);
        nh.param("ez", ez, epz);

        vector<vector<vector<int>>> map_data(X, vector<vector<int>>(Y, vector<int>(Z)));
        for (int i = 0; i < X; ++i)
        {
            for (int j = 0; j < Y; ++j)
            {
                for (int k = 0; k < Z; ++k)
                {
                    infile >> map_data[i][j][k];
                }
            }
        }

#endif
    ROS_INFO("start the %d planner\n", i + 1);
    // for (int i = 0; i < X; ++i)
    // {
    //     for (int j = 0; j < Y; ++j)
    //     {
    //         for (int k = 0; k < Z; ++k)
    //         {
    //             cout << map_data[i][j][k] << " ";
    //         }
    //         cout << endl;
    //     }
    //     cout << endl
    //          << endl;
    // }
    ros::Rate(100);
#ifndef MULTEST
    Map_Construct map(map_data, nh);
    Visualization v(nh);
    v.Init();
#endif

#ifdef USING_JPS
    Jps jps;
#else
        A_star a;
#endif

#ifdef USING_JPS
    int x = map_data.size();
    int y = map_data[0].size();
    int z = map_data[0][0].size();

    vector<signed char> realmap(x * y * z);
    realmap = jps.InitJps(map_data);
    jps.map = &realmap;
#else
        a.InitAstar(map_data);
#endif
    Point *start_point = new Point(sx, sy, sz); //start point
    Point *end_point = new Point(ex, ey, ez);   //end point

#ifdef USING_JPS
    std::list<Point *> *path = jps.getPath(start_point, end_point, *time, *distance);
    std::cout << " plan finished-----------------------" << endl
              << " JPS time consume:" << *time << "s" << endl
              << " JPS path total distance:" << *distance << "m" << endl;
    std::cout << " pathSize=" << path->size() << std::endl;
    *avgTime += *time / T;
    *avgDistance += *distance / T;
    outfile << i << "," << *time << "," << *distance << endl;
#ifndef MULTEST
    std::cout << " path:" << std::endl;
    list<Point *>::iterator p1;
    for (p1 = (*path).begin(); p1 != (*path).end(); p1++)
    {
        std::cout << " " << (*p1)->x << "  " << (*p1)->y << "  " << (*p1)->z << std::endl;
    }
#endif

#else
        std::list<Point *> *path = a.getPath(start_point, end_point, *time, *distance);
        std::cout << "plan finished-----------------------" << endl
                  << "time consume:" << *time << "s"
                  << "      path total distance:" << *distance << "m" << endl;
#endif
    delete (start_point);
    delete (end_point);
#ifndef MULTEST
    while (ros::ok())
    {
        map.build();
        v.draw_path(*path);

        ros::spinOnce();
    }
#endif
}

std::cout << "average time consume:" << *avgTime << "s"<<'\t'
          << "average distance" << *avgDistance << "m" << endl;
outfile.close();
delete (time);
delete (distance);
return 0;
}
