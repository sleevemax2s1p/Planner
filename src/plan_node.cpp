
#include "planner/A_star.h"
#include "planner/Map_Construct.h"
#include "planner/visualization.h"
#include "planner/jps.h"
#define USING_JPS
int main(int argc, char **argv)
{
    double *time = new double();
#ifndef USING_JPS
    int *distance = new int();
#else
    double *distance = new double();
#endif
    ros::init(argc, argv, "plan_node");
    ros::NodeHandle nh;
    nh.param("sx", sx, 7);
    nh.param("ex", ex, 0);
    nh.param("sy", sy, 4);
    nh.param("ey", ey, 0);
    nh.param("sz", sz, 7);
    nh.param("ez", ez, 0);
    //#ifndef USING_JPS
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
    // #else
    //     vector<vector<vector<int>>> map_data(5, vector<vector<int>>(5, vector<int>(5)));
    //     map_data[0] = {{0, 1, 1, 1, 1},
    //                    {0, 0, 1, 1, 1},
    //                    {0, 1, 1, 1, 1},
    //                    {0, 1, 1, 1, 1},
    //                    {0, 1, 1, 1, 1}};
    //     map_data[1] = {{1, 0, 1, 1, 1},
    //                    {1, 0, 1, 1, 1},
    //                    {1, 1, 1, 1, 1},
    //                    {1, 1, 1, 1, 1},
    //                    {0, 0, 0, 0, 0}};
    //     map_data[2] = {{1, 1, 1, 1, 0},
    //                    {1, 1, 1, 1, 0},
    //                    {1, 1, 1, 0, 0},
    //                    {1, 1, 1, 1, 0},
    //                    {1, 1, 1, 1, 0}};
    //     map_data[3] = {{0, 0, 0, 0, 0},
    //                    {1, 1, 1, 1, 1},
    //                    {1, 1, 1, 1, 1},
    //                    {1, 1, 1, 1, 1},
    //                    {1, 1, 1, 1, 1}};
    //     map_data[4] = {{0, 1, 0, 1, 1},
    //                    {0, 1, 1, 1, 1},
    //                    {0, 0, 0, 1, 1},
    //                    {0, 1, 1, 0, 1},
    //                    {0, 1, 1, 1, 0}};

    // #endif
    ROS_INFO("start");
    ros::Rate(100);
    Map_Construct map(map_data, nh);
    Visualization v(nh);
    v.Init();

#ifndef USING_JPS
    A_star a;
#else
    Jps jps;
#endif

#ifndef USING_JPS
    a.InitAstar(map_data);
#else
    int x = map_data.size();
    int y = map_data[0].size();
    int z = map_data[0][0].size();

    vector<signed char> realmap(x * y * z);
    realmap = jps.InitJps(map_data);
    jps.map = &realmap;
#endif
    Point *start_point = new Point(sx, sy, sz); //start point
    Point *end_point = new Point(ex, ey, ez);   //end point

#ifndef USING_JPS
    std::list<Point *> *path = a.getPath(start_point, end_point, *time, *distance);
    std::cout << "plan finished-----------------------" << endl
              << "time consume:" << *time<< "s"
              << "      path total distance:" << *distance << "m" << endl;

#else
    std::list<Point *> *path = jps.getPath(start_point, end_point, *time, *distance);
    std::cout << " plan finished-----------------------" << endl
              << " JPS time consume:" << *time<< "s"<<endl
              << " JPS path total distance:" << *distance << "m" << endl;
    std::cout << " pathSize="<<path->size() << std::endl;
    std::cout << " path:"<<std::endl;
    list<Point *>::iterator p1;     
    for(p1=(*path).begin();p1!=(*path).end();p1++){
		  std::cout <<" "<<(*p1)->x<< "  " <<  (*p1)->y<< "  " << (*p1)->z << std::endl;
	 }	   
#endif

    while (ros::ok())
    {
        map.build();
        v.draw_path(*path);
        // now = ros::Time().now().toSec();

        //  while(now-last_time==2){
        //      ROS_INFO("planning");
        //     map.build();
        //     vector<Point*> path = a.getPath(start_point,end_point);
        //     v.draw_path(path);
        //     last_time = now;
        //  }
        ros::spinOnce();
    }
    delete (start_point);
    delete (end_point);
    delete (time);
    delete (distance);
    return 0;
}
