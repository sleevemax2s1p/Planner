
#include "planner/A_star.h"
#include "planner/Map_Construct.h"
#include "planner/visualization.h"
int main(int argc, char **argv)
{
     double *time = new double();
    int *distance = new int();
    ros::init(argc,argv,"plan_node");
    ros::NodeHandle nh;
    nh.param("sx",sx,21);
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
    A_star a;
    a.InitAstar(map_data);
    Point* start_point= new Point(sx,sy,sz);
    Point* end_point = new Point(ex,ey,ez);

    
    std::list<Point*>* path = a.getPath(start_point,end_point,*time,*distance);
    std::cout<<"plan finished-----------------------"<<endl<<"time consume:"<<*time<<"s"<<"       path total distance:"<<*distance<<"m"<<endl;
    //std::list<Point*> path;

    //v.draw_path(a.getPath(start_point,end_point));
    double now = ros::Time().now().toSec();
    double last_time = ros::Time().now().toSec();
    while(ros::ok()){
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
    delete(start_point);
    delete(end_point);
    delete(time);
    delete(distance);
    return 0;
}
