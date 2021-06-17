#include "planner/Dijkstra.h"

// O(n^2)
const double INF = 1e9;

int Dijkstra::Point_to_node(const Point &p)
{
    if (!isInit)
    {
        ROS_ERROR("Point_to_node Please initialize the graph first!!!");
        return -1;
    }
    else
    {
        if (p.x < 0 || p.x >= x_size || p.y < 0 || p.y >= y_size || p.z < 0 || p.z >= z_size)
        {
            ROS_ERROR("Point_to_node point error");
            return -1;
        }
        return point_to_node(p.x, p.y, p.z);
    }
}
inline bool Dijkstra::postion_check(const int x, const int y, const int z)
{
    return x >= 0 && x < x_size && y >= 0 && y < y_size && z >= 0 && z < z_size;
}

inline int Dijkstra::point_to_node(const int x, const int y, const int z)
{
    return (x * y_size + y) * z_size + z;
}

inline void Dijkstra::node_to_Point(Point &p, const int node)
{
    int t = node;
    p.z = t % z_size;
    t /= z_size;
    p.y = t % y_size;
    p.x = t / y_size;
    // p.parent = NULL;
}
inline void Dijkstra::resize_vectors(const int siz)
{
    lowcost.resize(siz, INF);
    vis.resize(siz, false);
    pre.resize(siz, -1);
    cost.resize(siz);
    for (auto i = 0; i < cost.size(); ++i)
    {
        cost[i].resize(siz, INF);
    }
}

bool Dijkstra::bulidgraph_raw(const vector<vector<vector<int>>> &_map)
{
    if (_map.empty())
        return false;
    x_size = (int)_map.size();
    for (int i = 0; i < x_size; ++i)
    {
        if (i == 0)
        {
            if (_map[0].empty())
                return false;
            y_size = (int)_map[0].size();
        }
        else
        {
            if ((int)_map[i].size() != y_size)
                return false;
        }
        for (int j = 0; j < y_size; ++j)
        {
            if (j == 0 && i == 0)
            {
                if (_map[0][0].empty())
                    return false;
                z_size = (int)_map[0][0].size();
                if (graph_max_size / x_size / y_size / z_size < 1)
                {
                    ROS_ERROR("The map is too, too, too large!!!");
                    return false;
                }
                graph_size = x_size * y_size * z_size;
                resize_vectors(graph_size);
            }
            else
            {
                if ((int)_map[i][j].size() != z_size)
                {
                    return false;
                }
            }
            for (int k = 0; k < z_size; ++k)
            {
                if (_map[i][j][k])
                    continue;
                int p1 = point_to_node(i, j, k), p2;
                // cout<<"P1 = "<<p1<<"\t"<<i<<"\t"<<j<<"\t"<<k<<endl;
                // cout<<i<<"\t"<<j<<"\t"<<k<<endl;
                for (int ii = -1; ii <= 1; ++ii)
                {
                    for (int jj = -1; jj <= 1; ++jj)
                    {
                        for (int kk = -1; kk <= 1; ++kk)
                        {
                            if (ii == 0 && jj == 0 && kk == 0)
                                continue;
                            int xx = i + ii;
                            int yy = j + jj;
                            int zz = k + kk;
                            // cout<<xx<<"\t"<<yy<<"\t"<<zz<<endl;
                            if (postion_check(xx, yy, zz) && _map[xx][yy][zz] == 0)
                            {
                                p2 = point_to_node(xx, yy, zz);
                                cost[p1][p2] = mysqrt[ii * ii + jj * jj + kk * kk];
                            }
                        }
                    }
                }
            }
        }
    }
    return true;
}

bool Dijkstra::bulidgraph_opt1(const vector<vector<vector<int>>> &_map)
{
    if (_map.empty())
        return false;
    x_size = (int)_map.size();
    p2i.clear();
    i2p.clear();
    for (int i = 0; i < x_size; ++i)
    {
        if (i == 0)
        {
            if (_map[0].empty())
                return false;
            y_size = (int)_map[0].size();
        }
        else
        {
            if ((int)_map[i].size() != y_size)
                return false;
        }
        for (int j = 0; j < y_size; ++j)
        {
            if (j == 0 && i == 0)
            {
                if (_map[0][0].empty())
                    return false;
                z_size = (int)_map[0][0].size();
                if (graph_max_size / x_size / y_size / z_size < 1)
                {
                    ROS_ERROR("The map is too, too, too large!!!");
                    return false;
                }
                graph_size = x_size * y_size * z_size;
                // cout<<x_size<<"\t"<<y_size<<"\t"<<z_size<<"\t"<<graph_size<<endl;
                resize_vectors(graph_size);
                p2i.resize(graph_size, -1);
            }
            else
            {
                if ((int)_map[i][j].size() != z_size)
                {
                    return false;
                }
            }
            for (int k = 0; k < z_size; ++k)
            {
                if (_map[i][j][k])
                    continue;
                int p1 = point_to_node(i, j, k), p2;
                if (p2i[p1] == -1)
                {
                    p2i[p1] = i2p.size();
                    i2p.push_back(p1);
                }
                // cout<<"P1 = "<<p1<<"\t"<<i<<"\t"<<j<<"\t"<<k<<endl;
                // cout<<i<<"\t"<<j<<"\t"<<k<<endl;
                for (int ii = -1; ii <= 1; ++ii)
                {
                    for (int jj = -1; jj <= 1; ++jj)
                    {
                        for (int kk = -1; kk <= 1; ++kk)
                        {
                            if (ii == 0 && jj == 0 && kk == 0)
                                continue;
                            int xx = i + ii;
                            int yy = j + jj;
                            int zz = k + kk;
                            // cout<<xx<<"\t"<<yy<<"\t"<<zz<<endl;
                            if (postion_check(xx, yy, zz) && _map[xx][yy][zz] == 0)
                            {
                                p2 = point_to_node(xx, yy, zz);
                                if (p2i[p2] == -1)
                                {
                                    p2i[p2] = i2p.size();
                                    i2p.push_back(p2);
                                }
                                // cout<<"P2 = "<<p2<<"\t"<<xx<<"\t"<<yy<<"\t"<<zz<<endl;
                                const int &ip1 = p2i[p1], &ip2 = p2i[p2];
                                cost[ip1][ip2] = mysqrt[ii * ii + jj * jj + kk * kk];
                            }
                        }
                    }
                }
            }
        }
    }
    return true;
}

bool Dijkstra::bulidgraph_opt2(const vector<vector<vector<int>>> &_map)
{
    if (_map.empty())
        return false;
    edgegraph.clear();
    x_size = (int)_map.size();
    for (int i = 0; i < x_size; ++i)
    {
        if (i == 0)
        {
            if (_map[0].empty())
                return false;
            y_size = (int)_map[0].size();
        }
        else
        {
            if ((int)_map[i].size() != y_size)
                return false;
        }
        for (int j = 0; j < y_size; ++j)
        {
            if (j == 0 && i == 0)
            {
                if (_map[0][0].empty())
                    return false;
                z_size = (int)_map[0][0].size();
                if (graph_max_size / x_size / y_size / z_size < 1)
                {
                    ROS_ERROR("The map is too, too, too large!!!");
                    return false;
                }
                graph_size = x_size * y_size * z_size;
                // cout<<x_size<<"\t"<<y_size<<"\t"<<z_size<<"\t"<<graph_size<<endl;
                resize_vectors(graph_size);
                edgegraph.resize(graph_size);
            }
            else
            {
                if ((int)_map[i][j].size() != z_size)
                {
                    return false;
                }
            }
            for (int k = 0; k < z_size; ++k)
            {
                if (_map[i][j][k])
                    continue;
                int p1 = point_to_node(i, j, k), p2;
                // if (p2i[p1] == -1)
                // {
                //     p2i[p1] = i2p.size();
                //     i2p.push_back(p1);
                // }
                // cout<<"P1 = "<<p1<<"\t"<<i<<"\t"<<j<<"\t"<<k<<endl;
                // cout<<i<<"\t"<<j<<"\t"<<k<<endl;
                for (int ii = -1; ii <= 1; ++ii)
                {
                    for (int jj = -1; jj <= 1; ++jj)
                    {
                        for (int kk = -1; kk <= 1; ++kk)
                        {
                            if (ii == 0 && jj == 0 && kk == 0)
                                continue;
                            int xx = i + ii;
                            int yy = j + jj;
                            int zz = k + kk;
                            // cout<<xx<<"\t"<<yy<<"\t"<<zz<<endl;
                            if (postion_check(xx, yy, zz) && _map[xx][yy][zz] == 0)
                            {
                                p2 = point_to_node(xx, yy, zz);
                                // if (p2i[p2] == -1)
                                // {
                                //     p2i[p2] = i2p.size();
                                //     i2p.push_back(p2);
                                // }
                                // cout<<"P2 = "<<p2<<"\t"<<xx<<"\t"<<yy<<"\t"<<zz<<endl;
                                // const int &ip1 = p2i[p1], &ip2 = p2i[p2];
                                edgegraph[p1].push_back(Edge(p2, mysqrt[ii * ii + jj * jj + kk * kk]));
                            }
                        }
                    }
                }
            }
        }
    }
    return true;
}

bool Dijkstra::bulidgraph_opt3(const vector<vector<vector<int>>> &_map)
{
    if (_map.empty())
        return false;
    edgegraph.clear();
    x_size = (int)_map.size();
    for (int i = 0; i < x_size; ++i)
    {
        if (i == 0)
        {
            if (_map[0].empty())
                return false;
            y_size = (int)_map[0].size();
        }
        else
        {
            if ((int)_map[i].size() != y_size)
                return false;
        }
        for (int j = 0; j < y_size; ++j)
        {
            if (j == 0 && i == 0)
            {
                if (_map[0][0].empty())
                    return false;
                z_size = (int)_map[0][0].size();
                if (graph_max_size / x_size / y_size / z_size < 1)
                {
                    ROS_ERROR("The map is too, too, too large!!!");
                    return false;
                }
                graph_size = x_size * y_size * z_size;
                // cout<<x_size<<"\t"<<y_size<<"\t"<<z_size<<"\t"<<graph_size<<endl;
                edgegraph.resize(graph_size);
                i2p.clear();
                p2i.resize(graph_size, -1);
            }
            else
            {
                if ((int)_map[i][j].size() != z_size)
                {
                    return false;
                }
            }
            for (int k = 0; k < z_size; ++k)
            {
                if (_map[i][j][k])
                    continue;
                int p1 = point_to_node(i, j, k), p2;
                if (p2i[p1] == -1)
                {
                    p2i[p1] = i2p.size();
                    i2p.push_back(p1);
                }
                for (int ii = -1; ii <= 1; ++ii)
                {
                    for (int jj = -1; jj <= 1; ++jj)
                    {
                        for (int kk = -1; kk <= 1; ++kk)
                        {
                            if (ii == 0 && jj == 0 && kk == 0)
                                continue;
                            int xx = i + ii;
                            int yy = j + jj;
                            int zz = k + kk;
                            // cout<<xx<<"\t"<<yy<<"\t"<<zz<<endl;
                            if (postion_check(xx, yy, zz) && _map[xx][yy][zz] == 0)
                            {
                                p2 = point_to_node(xx, yy, zz);
                                if (p2i[p2] == -1)
                                {
                                    p2i[p2] = i2p.size();
                                    i2p.push_back(p2);
                                }
                                const int &ip1 = p2i[p1], &ip2 = p2i[p2];
                                // edgegraph[p1].push_back(Edge(p2, mysqrt[ii * ii + jj * jj + kk * kk]));
                                edgegraph[ip1].push_back(Edge(ip2, mysqrt[ii * ii + jj * jj + kk * kk]));
                            }
                        }
                    }
                }
            }
        }
    }
    lowcost.resize(i2p.size(), INF);
    vis.resize(i2p.size(), false);
    pre.resize(i2p.size(), -1);
    return true;
}

bool Dijkstra::InitDijkstra_raw(const vector<vector<vector<int>>> &_map)
{
#ifdef DEBUG_INFO
    ROS_INFO("Start raw dijkstra init");
#endif
    if (!bulidgraph_raw(_map))
    {
        ROS_INFO("MAP_ERROR!!!");
        return false;
    }
    isInit = true;
    isClear = true;
#ifdef DEBUG_INFO
    ROS_INFO("Dijkstra's graph initialize success.");
#endif
    return true;
}

bool Dijkstra::InitDijkstra_opt1(const vector<vector<vector<int>>> &_map)
{
#ifdef DEBUG_INFO
    ROS_INFO("Start dijkstra optimize 1 init");
#endif
    if (!bulidgraph_opt1(_map))
    {
        ROS_INFO("MAP_ERROR!!!");
        return false;
    }
    isInit = true;
    isClear = true;
#ifdef DEBUG_INFO
    ROS_INFO("Dijkstra's graph initialize success.");
#endif
    return true;
}

bool Dijkstra::InitDijkstra_opt2(const vector<vector<vector<int>>> &_map)
{
#ifdef DEBUG_INFO
    ROS_INFO("Start dijkstra optimize 2 init");
#endif
    if (!bulidgraph_opt2(_map))
    {
        ROS_INFO("MAP_ERROR!!!");
        return false;
    }
    isInit = true;
    isClear = true;
#ifdef DEBUG_INFO
    ROS_INFO("Dijkstra's graph initialize success.");
#endif
    return true;
}

bool Dijkstra::InitDijkstra_opt3(const vector<vector<vector<int>>> &_map)
{
#ifdef DEBUG_INFO
    ROS_INFO("Start dijkstra optimize 3 init");
#endif
    if (!bulidgraph_opt3(_map))
    {
        ROS_INFO("MAP_ERROR!!!");
        return false;
    }
    isInit = true;
    isClear = true;
#ifdef DEBUG_INFO
    ROS_INFO("Dijkstra's graph initialize success.");
#endif
    return true;
}

bool Dijkstra::clear()
{
    if (!isInit)
    {
        ROS_ERROR("Please initialize graph first!!!");
        return false;
    }
    for (auto &i : lowcost)
    {
        i = INF;
    }
    for (size_t i = 0; i < vis.size(); ++i)
    {
        vis[i] = false;
    }
    for (auto &i : pre)
    {
        i = -1;
    }
    isClear = true;
    path_one.clear();
    return true;
}

bool Dijkstra::FindPath_raw(const Point &start_point, const Point &end_point)
{
    if (!isInit)
    {
        ROS_ERROR("Please initialize graph first!!!");
        return false;
    }
    int sp = Point_to_node(start_point);
    int ep = Point_to_node(end_point);
#ifdef DEBUG_INFO
    cout << "Max    Size = " << graph_size << "\t" << x_size << "\t" << y_size << "\t" << z_size << endl;
    cout << "Start Point = " << sp << "\t" << start_point.x << "\t" << start_point.y << "\t" << start_point.z << endl;
    cout << "End   Point = " << ep << "\t" << end_point.x << "\t" << end_point.y << "\t" << end_point.z << endl;
#endif
    if (!isClear)
        clear();
    lowcost[sp] = 0;
    isClear = false;
    path_one.clear();
    for (int j = 0; j < graph_size; ++j)
    {
        int k = -1;
        int mindistance = INF;
        for (int i = 0; i < graph_size; ++i)
        {
            if (!vis[i] && lowcost[i] < mindistance)
            {
                mindistance = lowcost[i];
                k = i;
            }
        }
        if (k == -1)
            break;
        vis[k] = true;
        for (int i = 0; i < graph_size; ++i)
        {
            if (!vis[i] && cost[k][i] + lowcost[k] < lowcost[i])
            {
                lowcost[i] = lowcost[k] + cost[k][i];
                pre[i] = k;
            }
        }
    }
    if (lowcost[ep] == INF)
    {
        ROS_ERROR("Can't Reach the end point!!!");
        return false;
    }
    else
    {
        cout << "min distance=" << lowcost[ep] << endl;
        int temp = ep;
        Point pt(0, 0, 0);
        node_to_Point(pt, temp);
        path_one.clear();
        while (pre[temp] != -1)
        {
            Point *p = new Point(pt.x, pt.y, pt.z);
            temp = pre[temp];
            node_to_Point(pt, temp);
            p->parent = &pt;
            path_one.push_back(p);
        }
        Point *p = new Point(pt.x, pt.y, pt.z);
        p->parent = nullptr;
        path_one.push_back(p);
#ifdef DEBUG_INFO
        cout << "Dijkstra raw method success!\n";
#endif
        return true;
    }
}

bool Dijkstra::FindPath_opt1(const Point &start_point, const Point &end_point)
{
    if (!isInit)
    {
        ROS_ERROR("Please initialize graph first!!!");
        return false;
    }
    int sp = Point_to_node(start_point);
    int ep = Point_to_node(end_point);
    if (p2i[sp] == -1 || p2i[ep] == -1)
    {
        ROS_ERROR("The start point or end point can NOT arrive!!!");
    }
#ifdef DEBUG_INFO
    cout << "Max    Size = " << graph_size << "\t" << x_size << "\t" << y_size << "\t" << z_size << endl;
    cout << "Start Point = " << sp << "\t" << start_point.x << "\t" << start_point.y << "\t" << start_point.z << endl;
    cout << "End   Point = " << ep << "\t" << end_point.x << "\t" << end_point.y << "\t" << end_point.z << endl;
#endif
    int point_size = i2p.size();
    if (!isClear)
        clear();
    lowcost[p2i[sp]] = 0;
    isClear = false;
    path_one.clear();
    for (int j = 0; j < point_size; ++j)
    {
        int k = -1;
        int mindistance = INF;
        for (int i = 0; i < point_size; ++i)
        {
            if (!vis[i] && lowcost[i] < mindistance)
            {
                mindistance = lowcost[i];
                k = i;
            }
        }
        if (k == -1)
            break;
        vis[k] = true;
        for (int i = 0; i < point_size; ++i)
        {
            if (!vis[i] && cost[k][i] + lowcost[k] < lowcost[i])
            {
                lowcost[i] = lowcost[k] + cost[k][i];
                pre[i] = k;
                // Point *p = new Point(0, 0, 0);
                // node_to_Point(*p, i);
                // p->parent = p_last;
                // path_one.push_back(p);
                // p_last = p;
                // cout<<p->x<<"\t"<<p->y<<"\t"<<p->z<<endl;
            }
        }
    }
    if (lowcost[p2i[ep]] == INF)
    {
        ROS_ERROR("Can't Reach the end point!!!");
        return false;
    }
    else
    {
        cout << "min distance=" << lowcost[p2i[ep]] << endl;
        int temp = p2i[ep];
        Point pt(0, 0, 0);
        node_to_Point(pt, i2p[temp]);
        path_one.clear();
        while (pre[temp] != -1)
        {
            Point *p = new Point(pt.x, pt.y, pt.z);
            // cout << "x y z = " << pt.x << "\t" << pt.y << '\t' << pt.z << endl;
            temp = pre[temp];
            node_to_Point(pt, i2p[temp]);
            p->parent = &pt;
            path_one.push_back(p);
        }
        Point *p = new Point(pt.x, pt.y, pt.z);
        // cout << "x y z = " << pt.x << "\t" << pt.y << '\t' << pt.z << endl;
        p->parent = nullptr;
        path_one.push_back(p);
#ifdef DEBUG_INFO
        cout << "Dijkstra optimize 1 success!\n";
#endif
        return true;
    }
}

bool Dijkstra::FindPath_opt2(const Point &start_point, const Point &end_point)
{
    if (!isInit)
    {
        ROS_ERROR("Please initialize graph first!!!");
        return false;
    }
    int sp = Point_to_node(start_point);
    int ep = Point_to_node(end_point);
#ifdef DEBUG_INFO
    cout << "Max    Size = " << graph_size << "\t" << x_size << "\t" << y_size << "\t" << z_size << endl;
    cout << "Start Point = " << sp << "\t" << start_point.x << "\t" << start_point.y << "\t" << start_point.z << endl;
    cout << "End   Point = " << ep << "\t" << end_point.x << "\t" << end_point.y << "\t" << end_point.z << endl;
#endif
    if (!isClear)
        clear();
    lowcost[sp] = 0;
    isClear = false;
    path_one.clear();
    // Point *start_p = new Point(start_point.x, start_point.y, start_point.z);
    // Point *p_last = nullptr;
    // start_p->parent = p_last;
    // path_one.push_back(start_p);
    // p_last = start_p;
    priority_queue<qnode> que;
    while (!que.empty())
    {
        que.pop();
        /* code */
    }
    que.push(qnode(sp, 0));
    while (!que.empty())
    {
        qnode t = que.top();
        que.pop();
        int &id = t.i;
        if (vis[id])
        {
            continue;
        }
        vis[id] = true;
        for (auto i = 0; i < edgegraph[id].size(); ++i)
        {
            int &tid = edgegraph[id][i].v;
            double &c = edgegraph[id][i].cost;
            if (!vis[tid] && lowcost[tid] > lowcost[id] + c)
            {
                lowcost[tid] = lowcost[id] + c;
                que.push(qnode(tid, lowcost[tid]));
                pre[tid] = pre[id];
            }
        }
    }
    if (lowcost[ep] == INF)
    {
        ROS_ERROR("Can't Reach the end point!!!");
        return false;
    }
    else
    {
        cout << "min distance=" << lowcost[ep] << endl;
        int temp = ep;
        Point pt(0, 0, 0);
        node_to_Point(pt, temp);
        path_one.clear();
        while (pre[temp] != -1)
        {
            Point *p = new Point(pt.x, pt.y, pt.z);
            // cout << "x y z = " << pt.x << "\t" << pt.y << '\t' << pt.z << endl;
            temp = pre[temp];
            node_to_Point(pt, temp);
            p->parent = &pt;
            path_one.push_back(p);
        }
        Point *p = new Point(pt.x, pt.y, pt.z);
        // cout << "x y z = " << pt.x << "\t" << pt.y << '\t' << pt.z << endl;
        p->parent = nullptr;
        path_one.push_back(p);
#ifdef DEBUG_INFO
        cout << "Dijkstra optimize 2 success!\n";
#endif
        return true;
    }
}

bool Dijkstra::FindPath_opt3(const Point &start_point, const Point &end_point)
{
    if (!isInit)
    {
        ROS_ERROR("Please initialize graph first!!!");
        return false;
    }
    int sp = Point_to_node(start_point);
    int ep = Point_to_node(end_point);
#ifdef DEBUG_INFO
    cout << "Max    Size = " << graph_size << "\t" << x_size << "\t" << y_size << "\t" << z_size << endl;
    cout << "Start Point = " << sp << "\t" << start_point.x << "\t" << start_point.y << "\t" << start_point.z << endl;
    cout << "End   Point = " << ep << "\t" << end_point.x << "\t" << end_point.y << "\t" << end_point.z << endl;
#endif
    if (!isClear)
        clear();
    lowcost[p2i[sp]] = 0;
    isClear = false;
    path_one.clear();
    // Point *start_p = new Point(start_point.x, start_point.y, start_point.z);
    // Point *p_last = nullptr;
    // start_p->parent = p_last;
    // path_one.push_back(start_p);
    // p_last = start_p;
    priority_queue<qnode> que;
    while (!que.empty())
    {
        que.pop();
        /* code */
    }
    que.push(qnode(p2i[sp], 0));
    while (!que.empty())
    {
        qnode t = que.top();
        que.pop();
        int &id = t.i;
        if (vis[id])
        {
            continue;
        }
        vis[id] = true;
        for (auto i = 0; i < edgegraph[id].size(); ++i)
        {
            int &tid = edgegraph[id][i].v;
            double &c = edgegraph[id][i].cost;
            if (!vis[tid] && lowcost[tid] > lowcost[id] + c)
            {
                lowcost[tid] = lowcost[id] + c;
                que.push(qnode(tid, lowcost[tid]));
                pre[tid] = pre[id];
            }
        }
    }
    if (lowcost[p2i[ep]] == INF)
    {
        ROS_ERROR("Can't Reach the end point!!!");
        return false;
    }
    else
    {
#ifdef DEBUG_INFO
        cout << "min distance=" << lowcost[p2i[ep]] << endl;
#endif
        int temp = p2i[ep];
        Point pt(0, 0, 0);
        node_to_Point(pt, ep);
        path_one.clear();
        while (pre[temp] != -1)
        {
            Point *p = new Point(pt.x, pt.y, pt.z);
            temp = pre[temp];
            node_to_Point(pt, i2p[temp]);
            p->parent = &pt;
            path_one.push_back(p);
        }
        Point *p = new Point(pt.x, pt.y, pt.z);
        p->parent = nullptr;
        path_one.push_back(p);
#ifdef DEBUG_INFO
        cout << "Dijkstra optimize 3 success!\n";
#endif
        return true;
    }
}

list<Point *> *Dijkstra::getPath_raw(Point *start_point, Point *end_point, double &time, double &distance)
{
    if (!isInit)
    {
        ROS_INFO("Please Init the graph first!!! && Please find path first!!!");
        return nullptr;
    }
    int sp = Point_to_node(*start_point);
    int ep = Point_to_node(*end_point);
    if (sp < 0 || sp >= graph_max_size || ep < 0 || ep >= graph_max_size)
    {
        ROS_INFO("Point Error!!!");
        return nullptr;
    }
    clock_t startTime, endTime;
    startTime = clock();
    FindPath_raw(*start_point, *end_point);
    endTime = clock();
    time = (double)(endTime - startTime) / CLOCKS_PER_SEC;
    distance = lowcost[ep];
    return &path_one;
}
list<Point *> *Dijkstra::getPath_opt1(Point *start_point, Point *end_point, double &time, double &distance)
{
    if (!isInit)
    {
        ROS_INFO("Please Init the graph first!!! && Please find path first!!!");
        return nullptr;
    }
    int sp = Point_to_node(*start_point);
    int ep = Point_to_node(*end_point);
    if (sp < 0 || sp >= graph_max_size || ep < 0 || ep >= graph_max_size || p2i[sp] == -1 || p2i[ep] == -1)
    {
        ROS_INFO("Point Error!!!");
        return nullptr;
    }
    clock_t startTime, endTime;
    startTime = clock();
    FindPath_opt1(*start_point, *end_point);
    endTime = clock();
    time = (double)(endTime - startTime) / CLOCKS_PER_SEC;
    distance = lowcost[p2i[ep]];
    return &path_one;
}
list<Point *> *Dijkstra::getPath_opt2(Point *start_point, Point *end_point, double &time, double &distance)
{
    if (!isInit)
    {
        ROS_INFO("Please Init the graph first!!! && Please find path first!!!");
        return nullptr;
    }
    int sp = Point_to_node(*start_point);
    int ep = Point_to_node(*end_point);
    if (sp < 0 || sp >= graph_max_size || ep < 0 || ep >= graph_max_size)
    {
        ROS_INFO("Point Error!!!");
        return nullptr;
    }
    clock_t startTime, endTime;
    startTime = clock();
    FindPath_opt2(*start_point, *end_point);
    endTime = clock();
    time = (double)(endTime - startTime) / CLOCKS_PER_SEC;
    distance = lowcost[ep];
    return &path_one;
}
list<Point *> *Dijkstra::getPath_opt3(Point *start_point, Point *end_point, double &time, double &distance)
{
    if (!isInit)
    {
        ROS_INFO("Please Init the graph first!!! && Please find path first!!!");
        return nullptr;
    }
    int sp = Point_to_node(*start_point);
    int ep = Point_to_node(*end_point);
    if (sp < 0 || sp >= graph_max_size || ep < 0 || ep >= graph_max_size || p2i[sp] == -1 || p2i[ep] == -1)
    {
        ROS_INFO("Point Error!!!");
        return nullptr;
    }
    clock_t startTime, endTime;
    startTime = clock();
    FindPath_opt3(*start_point, *end_point);
    endTime = clock();
    time = (double)(endTime - startTime) / CLOCKS_PER_SEC;
    distance = lowcost[p2i[ep]];
    return &path_one;
}

/* use BFS*/
bool Dijkstra::getAllPath(vector<list<Point *>> &vlp, const Point &start_point, const Point &end_point)
{
    if (!isInit || isClear)
    {
        ROS_ERROR("Find Path Error!!!");
    }
    int sp = Point_to_node(start_point);
    int ep = Point_to_node(end_point);
    if (lowcost[sp] != 0)
    {
        ROS_ERROR("Start Point Error, Please run findpath");
    }
    if (lowcost[ep] == INF)
    {
        ROS_ERROR("End Point can't reach!!!");
    }
    list<Point *> lp;
    queue<Point *> qp;
    while (!qp.empty())
    {
        break;
    }
    return true;
}

// bool Dijkstra_raw::FindPath(const Point &start_point, const Point &end_point)
// {
//     if (!isInit)
//     {
//         ROS_INFO("Please initialize graph first!!!");
//         return false;
//     }
//     if (!isClear)
//     {
//         if(!clear())
//         {
//             ROS_INFO("Graph clear fail!!!");
//             return false;
//         }
//         ROS_INFO("Graph clear finish.");
//     }
//     isClear = false;
//     graph[start_point.x][start_point.y][start_point.z].G = 0;
//     for(int i=0; i<graph.size();++i)
//     {
//         for(int j=0;j<graph[i].size();++j)
//         {
//             for(int k=0;k<graph[i][j].size(); ++k)
//             {
//                 Point *flagk=nullptr;
//                 int Min = INF;
//                 for(int ii=0; ii<graph.size();++ii)
//                 {
//                     for(int jj=0; jj<graph[ii].size();++jj)
//                     {
//                         for(int kk=0; kk<graph[ii][jj].size();++kk)
//                         {
//                             auto &t = graph[ii][jj][kk];
//                             if(!t.vis&&t.G<Min)
//                             {
//                                 Min=t.G;
//                                 flagk = &t;
//                             }
//                         }
//                     }
//                 }
//                 if(flagk == nullptr)
//                 {
//                     return true;
//                 }
//                 flagk->vis=true;
//                 for(int ii=0; ii<graph.size(); ++ii)
//                 {
//                     for(int jj=0; jj<graph[ii].size(); ++j)
//                     {
//                         for(int kk=0; kk<graph[ii][jj].size(); ++k)
//                         {
//                             auto &t = graph[ii][jj][kk];
//                             if(!t.vis&&flagk->G);
//                         }
//                     }
//                 }
//             }
//         }
//     }
// }
