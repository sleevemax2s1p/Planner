#include <iostream>
#include <vector>
#include <queue>
#include <ctime>
#include <fstream>

const int num_data = 10;        //the number of cases
const int X = 60;               //the X size of map
const int Y = 10;   //30;       //the Y size of map
const int Z = 10;   //30;       //the Z size of map
const int x_obs_max = 3;
const int x_obs_min = 1;
const int y_obs_max = 5;//9;
const int y_obs_min = 2;//2;
const int z_obs_max = 5;//9;
const int z_obs_min = 2;//2;    
const double rate = 10;     //the rate of obstacle 0~100
// const char file_name[] = "Datasets_with_ten_data.txt";  // the file name of map
const char file_name[] = "smalldatasetfordjs.txt";  // the file name of map

//  output format:
//  first line: the number of case
//  for every case
//  the first line: three number, x, y, z
//  the second line: two points x0 y0 z0 x1 y1 z1
//  start from the third line: map

using namespace std;
struct qnode
{
    int x, y, z;
    qnode(int _x = 0, int _y = 0, int _z = 0) : x(_x), y(_y), z(_z) {}
    bool operator==(const qnode &q) const
    {
        return x == q.x && y == q.y && z == q.z;
    }
};

inline bool isinmap(int x, int y, int z)
{
    return x >= 0 && x < X && y >= 0 && y < Y && z >= 0 && z < Z;
}

bool bfs(const vector<vector<vector<int>>> &_m, const qnode &start_point, const qnode &end_point)
{
    vector<vector<vector<int>>> vis(_m.size(), vector<vector<int>>(_m[0].size(), vector<int>(_m[0][0].size(), 0)));
    queue<qnode> q;
    q.push(start_point);
    vis[start_point.x][start_point.y][start_point.z] = 1;
    while (!q.empty())
    {
        qnode t = q.front();
        // cout<<t.x<<"\t"<<t.y<<"\t"<<t.z<<endl;
        if (t == end_point)
        {
            return true;
        }
        q.pop();
        for (int ii = -1; ii <= 1; ++ii)
        {
            for (int jj = -1; jj <= 1; ++jj)
            {
                for (int kk = -1; kk <= 1; ++kk)
                {
                    int xx = t.x + ii;
                    int yy = t.y + jj;
                    int zz = t.z + kk;
                    if (isinmap(xx, yy, zz) && vis[xx][yy][zz] == 0 && _m[xx][yy][zz] == 0)
                    {
                        q.push(qnode(xx, yy, zz));
                        vis[xx][yy][zz] = 1;
                    }
                }
            }
        }
    }
    return false;
}

int get_rand()
{
    return rand() % 100000 <= rate * 1000 ? 1 : 0;
}
int get_rand(int mi, int mx)
{
    if (mi >= mx)
    {
        return mi;
    }
    return rand() < mx ? mi : mi + rand() % (mx - mi);
}

int main()
{
    // freopen(file_name, "w", stdout);
    ofstream fout(file_name);
    srand(time(0));
    fout << num_data << endl;
    for (int T = 0; T < num_data; ++T)
    {
        qnode start_point(0, Y / 2, Z / 2), end_point(X - 1, Y / 2 + 1, Z / 2 + 1);
        fout << X << " " << Y << " " << Z << endl;
        fout << start_point.x << " " << start_point.y << " " << start_point.z << " ";
        fout << end_point.x << " " << end_point.y << " " << end_point.z << endl;
        vector<vector<vector<int>>> _map(X, vector<vector<int>>(Y, vector<int>(Z, 0)));
        do
        {
            /* code */
            for (auto i = 0; i < X; i += get_rand(x_obs_min, x_obs_max))
            {
                for (auto j = 0; j < Y; j += get_rand(y_obs_min, y_obs_max))
                {
                    for (auto k = 0; k < Z; k += get_rand(z_obs_min, z_obs_max))
                    {
                        if (get_rand())
                        {
                            int x_size = get_rand(x_obs_min, x_obs_max);
                            int y_size = get_rand(y_obs_min, y_obs_max);
                            int z_size = get_rand(z_obs_min, z_obs_max);
                            for (int ii = 0; ii < x_size; ++ii)
                            {
                                for (int jj = 0; jj < y_size; ++jj)
                                {
                                    for (int kk = 0; kk < z_size; ++kk)
                                    {
                                        if (isinmap(i + ii, j + jj, k + kk))
                                        {
                                            _map[i + ii][j + jj][k + kk] = 1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            // cout<<"RAND finish"<<endl;
            _map[start_point.x][start_point.y][start_point.z] = 0;
            _map[end_point.x][end_point.y][end_point.z] = 0;
        } while (!bfs(_map, start_point, end_point));
        for (size_t i = 0; i < _map.size(); ++i)
        {
            for (auto j = 0; j < _map[i].size(); ++j)
            {
                for (auto k = 0; k < _map[i][j].size(); ++k)
                {
                    fout << _map[i][j][k] << " ";
                }
                fout << endl;
            }
            fout << endl;
        }
        cout << "Number " << T + 1 << " case finish!!! " << endl;
    }
    cout << "Success" << endl;
    return 0;
}