
#include "planner/A_star.h"
#include "planner/Map_Construct.h"
#include "planner/visualization.h"
int main(int argc, char **argv)
{
    bool mapIsInited = false;
    ros::init(argc,argv,"plan_node");
    ros::NodeHandle nh;

    nh.param("sx",sx,0);
    nh.param("ex",ex,2);
    nh.param("sy",sy,0);
    nh.param("ey",ey,2);
    nh.param("sz",sz,0);
    nh.param("ez",ez,2);
    std::cout<<"sx:"<<sx<<"sy:"<<sy<<"sz:"<<sz<<"ex:"<<ex<<"ey:"<<ey<<"ez:"<<ey<<endl;
    vector<vector<vector<int>>> map_data(10,vector<vector<int>>(5,vector<int>(14)));
    map_data[0] = {{0,0,1,0,0,1,1,0,1,1,1,1,1,1},
                   {1,0,0,0,1,0,1,0,1,0,1,0,1,1},
                   {0,0,0,0,0,0,0,0,0,0,0,0,0,1},
                   {1,0,1,0,1,1,1,1,0,0,0,0,1,0},
                   {1,0,0,0,0,0,1,1,1,1,1,0,0,0}
                   };
    map_data[1] = {{1,0,1,0,0,1,1,0,1,0,0,0,1,1},
                   {1,0,0,0,1,0,1,0,1,0,1,0,1,0},
                   {0,1,0,0,0,0,0,0,1,0,0,1,0,0},
                   {1,0,0,0,1,0,0,1,0,0,0,0,1,0},
                   {1,0,1,0,0,0,1,0,1,0,1,0,0,0}
                   };   
    map_data[2] = {{1,0,0,0,0,0,0,0,1,0,0,0,0,0},
                   {0,0,0,0,0,0,1,1,0,0,0,0,1,0},
                   {1,1,1,0,0,0,0,0,0,0,0,0,0,0},
                   {0,0,0,0,0,0,0,0,0,0,1,0,0,0},
                   {1,0,0,0,0,0,0,0,0,0,0,0,1,0}
                   }; 
    map_data[3] = {{0,1,0,1,0,0,0,0,0,0,0,1,0,1},
                   {1,0,0,0,0,1,1,1,1,0,0,0,1,0},
                   {0,0,1,1,1,0,1,0,1,0,1,0,0,0},
                   {1,0,1,0,1,0,0,0,0,1,1,0,1,0},
                   {1,0,0,0,0,1,0,1,0,1,0,0,0,0}
                   };
    map_data[4] = {{1,0,0,1,0,1,0,1,0,1,0,1,1,1},
                   {0,0,0,0,0,0,1,0,1,0,1,1,0,0},
                   {1,0,1,1,1,1,1,1,0,0,0,1,1,1},
                   {0,0,0,1,0,1,1,0,1,0,1,0,1,1},
                   {1,1,1,0,0,0,0,0,0,0,0,1,0,0}
                   };
    map_data[5] = {{0,0,0,1,0,1,0,1,1,1,0,0,0,0},
                   {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {0,0,0,0,0,1,0,0,0,0,0,1,0,0},
                   {1,0,0,1,0,0,0,0,0,0,1,0,0,1},
                   };
    map_data[6] = {{0,0,0,0,1,0,0,0,0,0,0,0,0,1},
                   {1,1,1,1,0,1,0,1,0,0,0,0,0,0},
                   {1,1,1,1,1,0,0,1,0,1,0,1,0,0},
                   {1,0,0,0,0,1,0,1,0,0,0,0,0,0},
                   {0,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    map_data[7] = {{1,1,1,0,0,0,0,0,0,0,0,0,0,0},
                   {0,0,0,0,0,0,0,0,0,1,0,0,1,0},
                   {1,0,0,1,0,1,0,0,0,0,0,0,1,0},
                   {0,1,0,1,1,0,0,0,0,0,0,1,0,0},
                   {1,0,0,0,0,0,0,0,0,0,1,0,0,0}
                   };            
    map_data[8] = {{0,0,0,0,0,1,0,1,0,1,1,0,0,1},
                   {1,1,1,1,0,0,0,0,0,0,0,0,0,0},
                   {0,0,0,0,0,0,0,0,0,0,0,0,1,0},
                   {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                   {1,0,0,1,0,1,0,1,0,0,1,0,1,0}
                   };
    map_data[9] = {{1,1,1,0,1,0,1,0,1,0,1,0,1,0},
                   {1,0,1,0,1,1,1,0,0,0,0,0,0,0},
                   {1,1,1,1,1,0,0,0,1,0,1,0,1,0},
                   {0,0,0,0,1,0,1,0,1,0,0,0,0,0},
                   {0,0,0,0,0,0,0,0,0,0,0,0,0,0}
                   };
    ROS_INFO("start");         
    Map_Construct map(map_data,nh);
    Visualization v;
    v.Init(nh);
    A_star a;
    //Point* start_point = new Point(sx,sy,sz);
    //Point* end_point = new Point(ex,ey,ez);
    while(ros::ok()){
         //a.InitAstar(map_data);
         map.build();
         //vector<Point*> path;
         //v.draw_path(path);
    }
    
    return 0;
}
