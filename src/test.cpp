
#include "planner/A_star.h"
#include "planner/Map_Construct.h"
#include "planner/visualization.h"
#include "planner/jps.h"
#include "planner/Dijkstra.h"
#include "planner/RRT.h"
#include <fstream>


const char file_name[] = "/home/helab/course/src/planner/map_dataset/dataset.txt";

int main(int argc, char **argv)
{
    double *Astar_time = new double();
    double *Astar_distance = new double();

    double *Jps_time = new double();
    double *Jps_distance = new double();

    double *RRT_time = new double();
    double *RRT_distance = new double();
    ifstream infile(file_name);
    int T, cases;
    double sum = 0;
    infile >> T;
    cases = T;
    int i = 0;
    
    while (T--)
    {
        std::cout<<(i++)<<"-----------------------"<<endl;
        double *Astar_time1 = new double();
         double *Astar_distance1 = new double();

        double *Jps_time1 = new double();
        double *Jps_distance1 = new double();

         double *RRT_time1= new double();
        double *RRT_distance1 = new double();
        int X, Y, Z;
        int spx, spy, spz, epx, epy, epz;
        infile >> X >> Y >> Z;
        infile >> spx >> spy >> spz >> epx >> epy >> epz;
        cout << "X Y Z  " << X << "\t" << Y << "\t" << Z << endl;
        cout << spx << "\t" << spy << "\t" << spz << endl;
        cout << epx << "\t" << epy << "\t" << epz << endl;

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
        ros::init(argc, argv, "plan_node");
        ros::NodeHandle nh;
        nh.param("sx", sx, spx);
        nh.param("ex", ex, epx);
        nh.param("sy", sy, spy);
        nh.param("ey", ey, epy);
        nh.param("sz", sz, spz);
        nh.param("ez", ez, epz);
        ROS_INFO("start");
        ros::Rate(100);
        Map_Construct map(map_data, nh);
        Visualization v(nh);
        v.Init();
         Point* start_point= new Point(sx,sy,sz);
    Point* end_point = new Point(ex,ey,ez);

    node* startpoint = new node(sx,sy,sz);
	node *endpoint =new node(ex,ey,ez);

    //     A star
    A_star a;
    a.InitAstar(map_data);

    std::list<Point*>* A_star_path = a.getPath(start_point,end_point,*Astar_time1,*Astar_distance1);
    *Astar_time+=*Astar_time1;
    std::cout<<1<<endl;
    //
    // JPS
    Jps jps;
    int x = map_data.size();
    int y = map_data[0].size();
    int z = map_data[0][0].size();

    vector<signed char> realmap(x * y * z);
    realmap = jps.InitJps(map_data);
    jps.map = &realmap;
    std::list<Point *> *Jps_path = jps.getPath(start_point, end_point, *Jps_time1, *Jps_distance1);
    *Jps_time+=*Jps_time1;
    std::cout<<2<<endl;
    //JPS

    //RRT
    RRT rrt(startpoint,endpoint, map_data,1,5);
    list<Point*>* RRT_path=rrt.getPath(start_point, end_point,*RRT_time1,*RRT_distance1);
   *RRT_time+=*RRT_time1;
   std::cout<<3<<endl;
	// for(auto point:*RRT_path){
	// 	std::cout<<"x:"<<point->x<<"y:"<<point->y<<"z:"<<point->z<<endl;
	// };
    //RRT
    delete(startpoint);
    delete(endpoint);
    delete(start_point);
    delete(end_point);
    delete(Astar_time1);
    delete(Astar_distance1);
    delete(Jps_time1);
    delete(Jps_distance1);
    delete(RRT_time1);
    delete(RRT_distance1);
    }

    std::cout<<"A_star average time:"<<*Astar_time/1000<<"s"<<endl;
    std::cout<<"Jps average time:"<<*Jps_time/1000<<"s"<<endl;
    std::cout<<"RRT average time:"<<*RRT_time/1000<<"s"<<endl;
    delete(Astar_time);
    delete(Astar_distance);
    delete(Jps_time);
    delete(Jps_distance);
    delete(RRT_time);
    delete(RRT_distance);
}