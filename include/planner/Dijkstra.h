#ifndef _DIJKSTRA_
#define _DIJKSTRA_

#include <ros/ros.h>
#include "planner.h"
#include <list>
#include <vector>
#include <queue>
#include <cmath>

using namespace std;

class Dijkstra : public Abstract_planner
{
public:
    bool InitDijkstra_raw(const vector<vector<vector<int>>> &_map);
    list<Point *> *getPath(Point *start_point, Point *end_point, double &time, int &distance);
    vector<Point *> getSurroundPoints(const Point *point) const;
    bool FindPath_raw(const Point &start_point, const Point &end_point);
    bool isCanreach(const Point *point, const Point *target) const;
    Point *isInList(const std::list<Point *> &list, const Point *point) const;
    Point *getLeastFpoint();
    int Point_to_node(const Point& p);
    inline void node_to_Point(Point &p, const int node);
    bool clear();
    bool getAllPath(vector<list<Point*>> &vlp, const Point &start_point, const Point &end_point);
    int calcG(Point *temp_start, Point *end);
    int calcH(Point *start, Point *end);
    int calcF(Point *point);

private:
    bool bulidgraph_raw(const vector<vector<vector<int>>> &_map);
    inline int point_to_node(const int x, const int y, const int z);
    inline bool postion_check(const int x, const int y, const int z);
    inline void resize_vectors(void);
    
    vector<double> lowcost;
    vector<list<Point>> path;
    vector<int>path_to_node;
    vector<bool>vis;
    vector<int>pre;
    vector<vector<double>> cost;
    list<Point *> path_one;
    list<Point *> openlist;
    list<Point *> closelist;
    bool isInit=false;
    bool isClear = false;
    int x_size=0, y_size=0, z_size=0;
    int graph_size=0;
    const double graph_max_size=1e6;
};


#endif
