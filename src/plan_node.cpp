
#include "planner/A_star.h"
#include "planner/Map_Construct.h"
#include "planner/visualization.h"
#include "planner/jps.h"
#include "planner/Dijkstra.h"
#include "planner/RRT.h"
#include <fstream>

#define USINGDATASET


int main(int argc, char **argv)
{
    double *Astar_time = new double();
    double *Astar_distance = new double();

    double *Jps_time = new double();
    double *Jps_distance = new double();

    double *RRT_time = new double();
    double *RRT_distance = new double();

    double *Dijkstra_time = new double();
    double *Dijkstra_distance = new double();
    ros::init(argc,argv,"plan_node");
    ros::NodeHandle nh;
    nh.param("sx",sx,20);
    nh.param("ex",ex,2);
    nh.param("sy",sy,13);
    nh.param("ey",ey,4);
    nh.param("sz",sz,2);
    nh.param("ez",ez,5);    
    
    vector<vector<vector<int>>> map_data(21,vector<vector<int>>(15,vector<int>(14)));
    map_data[0] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,1,1,1,0,0,0,0,0,0,0},
                   {1,0,0,0,1,1,1,0,0,0,0,0,0,0},
                   {1,0,0,0,1,1,1,0,0,0,0,0,0,0},
                   {1,0,0,0,1,1,1,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[1] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,1,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,1,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,1,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,1,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };   
    map_data[2] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,1,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,1,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,1,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,1,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[3] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,1,1,1,0,0,0,0,0,0,0},
                   {1,0,0,0,1,1,1,0,0,0,0,0,0,0},
                   {1,0,0,0,1,1,1,0,0,0,0,0,0,0},
                   {1,0,0,0,1,1,1,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[4] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[5] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[6] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[7] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };         
    map_data[8] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,1,1,1,1,1,0,0,0,0,0},
                   {1,0,0,0,1,1,1,1,1,0,0,0,0,0},
                   {1,0,0,0,1,1,1,1,1,0,0,0,0,0},
                   {1,0,0,0,1,1,1,1,1,0,0,0,0,0},
                   {1,0,0,0,1,1,1,1,1,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[9] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[10] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[11] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[12] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,1,1,1,1,1,1,1,1,1,0,0,0,0},
                   {1,1,1,1,1,1,1,1,1,1,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[20] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,1,1,0,0,0,0,0,0},
                   {1,0,0,0,0,1,0,0,1,0,0,0,0,0},
                   {1,0,0,0,1,0,0,0,0,1,0,0,0,0},
                   {1,0,0,1,0,0,0,0,0,0,1,0,0,0},
                   {1,0,0,0,1,0,0,0,0,1,0,0,0,0},
                   {1,0,0,0,0,1,0,0,1,0,0,0,0,0},
                   {1,0,0,0,0,0,1,1,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[13] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,1,1,1,1,1,1,1,1,0,0,0,0,0},
                   {1,1,1,1,1,1,1,1,1,0,0,0,0,0},
                   {1,1,1,1,1,1,1,1,1,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,1,1,1,1,1,1,1,1,0,0,0,0,0},
                   {1,1,1,1,1,1,1,1,1,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[14] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[15] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[16] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[17] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[18] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,1,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,1,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,1,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,1,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,1,1,1,1,1,0,0,0,0,0,0,0,0},
                   {1,1,0,0,0,1,0,0,0,0,0,0,0,0},
                   {1,1,0,0,0,1,0,0,0,0,0,0,0,0},
                   {1,1,1,1,1,1,0,0,0,0,0,0,0,0}
                   };
    map_data[19] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    ROS_INFO("start");      
    ros::Rate(100);   
    Map_Construct map(map_data,nh);
    Visualization v(nh);
    v.Init(); 
    Point* start_point= new Point(sx,sy,sz);
    Point* end_point = new Point(ex,ey,ez);

    node* startpoint = new node(20, 13, 2);
	node *endpoint =new node(2,4,5);

    //     A star
    A_star a;
    a.InitAstar(map_data);
    std::list<Point*>* A_star_path = a.getPath(start_point,end_point,*Astar_time,*Astar_distance);
    std::cout<<"A_star: plan finished-----------------------"<<endl<<"time consume:"<<*Astar_time<<"s"<<"       path total distance:"<<*Astar_distance<<"m"<<endl;
    //
    // JPS
    Jps jps;
    int x = map_data.size();
    int y = map_data[0].size();
    int z = map_data[0][0].size();

    vector<signed char> realmap(x * y * z);
    realmap = jps.InitJps(map_data);
    jps.map = &realmap;
    std::list<Point *> *Jps_path = jps.getPath(start_point, end_point, *Jps_time, *Jps_distance);
    std::cout <<"JPS: plan finished-----------------------"<<endl<< "time consume:"<<*Jps_time<<"s"<<"       path total distance:"<<*Jps_distance <<"m"<<endl;
    //JPS

    //RRT
    RRT rrt(startpoint,endpoint, map_data,1,5);
    list<Point*>* RRT_path=rrt.getPath(start_point, end_point,*RRT_time,*RRT_distance);
	// for(auto point:*RRT_path){
	// 	std::cout<<"x:"<<point->x<<"y:"<<point->y<<"z:"<<point->z<<endl;
	// };
    std::cout<<"RRT: plan finished-----------------------"<<endl<<"time consume:"<<*RRT_time<<"s"<<"       path total distance:"<<*RRT_distance<<"m"<<endl;
    //RRT

    //Dijkstra     在此修改****
    Dijkstra djs;
    djs.InitDijkstra_opt3(map_data);
    //std::list<Point*>* path = a.getPath(start_point,end_point,*time,*distance);
    //std::list<Point*> path;
    ros::Time now = ros::Time::now();
    djs.FindPath_opt3(*start_point, *end_point);
    *Dijkstra_time = (ros::Time::now()-now).toSec();
    list<Point *> *Dijkstra_path = djs.getPath_opt3(start_point, end_point, *Dijkstra_time, *Dijkstra_distance);
    std::cout<<"Dijkstra: plan finished-----------------------"<<endl<<"time consume:"<<*Dijkstra_time<<"s"<<"       path total distance:"<<*Dijkstra_distance<<"m"<<endl;
    //Dijkstra
    while(ros::ok()){
        map.build();
        v.draw_path(*A_star_path,0,0,1.0,0);
        v.draw_path(*Jps_path,0,1.0,1.0,1);
        v.draw_path(*RRT_path,1.0,1.0,1.0,2);
        v.draw_path(*Dijkstra_path,0.8,1.0,0.6,3);
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
    delete(start_point);
    delete(end_point);
    delete(Astar_time);
    delete(Astar_distance);
    delete(Jps_time);
    delete(Jps_distance);
    delete(RRT_time);
    delete(RRT_distance);
    delete(Dijkstra_time);
    delete(Dijkstra_distance);
    return 0;
}
