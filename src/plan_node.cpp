
#include "planner/A_star.h"
#include "planner/Map_Construct.h"
#include "planner/visualization.h"
int main(int argc, char **argv)
{
    double *time = new double();
    int *distance = new int();
    ros::init(argc,argv,"plan_node");
    ros::NodeHandle nh;
    nh.param("sx",sx,7);
    nh.param("ex",ex,0);
    nh.param("sy",sy,4);
    nh.param("ey",ey,0);
    nh.param("sz",sz,7);
    nh.param("ez",ez,0);    
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
