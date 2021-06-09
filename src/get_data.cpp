#include<iostream>
#include<vector>
#include<queue>
#include<ctime>

const int num_data = 1;
const int X = 20;
const int Y = 12;
const int Z = 12;
const char file_name[]="test1.txt";
const double rate = 70; //0~100


//  output format:
//  first line: the number of case
//  for every case
//  the first line: three number, x, y, z
//  the second line: two points x0 y0 z0 x1 y1 z1
//  start from the third line: map

using namespace std;
struct qnode
{
    int x,y,z;
    qnode(int _x=0, int _y=0, int _z=0):x(_x),y(_y),z(_z){}
    bool operator ==(const qnode &q) const
    {
        return x==q.x&&y==q.y&&z==q.z;
    }
};

inline bool isinmap(int x,int y, int z)
{
    return x>=0&&x<X&&y>=0&&y<Y&&z>=0&&z<Z;
}

bool bfs(const vector<vector<vector<int>>> &_m, const qnode &start_point, const qnode &end_point)
{
    vector<vector<vector<int>>> vis(_m.size(), vector<vector<int>>(_m[0].size(),vector<int>(_m[0][0].size(),0)));
    queue<qnode> q;
    q.push(start_point);
    vis[start_point.x][start_point.y][start_point.z]=1;
    while(!q.empty())
    {
        qnode t=q.front();
        // cout<<t.x<<"\t"<<t.y<<"\t"<<t.z<<endl;
        if(t==end_point)
        {
            return true;
        }
        q.pop();
        for(int ii=-1;ii<=1;++ii)
        {
            for(int jj=-1;jj<=1;++jj)
            {
                for(int kk=-1;kk<=1;++kk)
                {
                    int xx=t.x+ii;
                    int yy=t.y+jj;
                    int zz=t.z+kk;
                    if(isinmap(xx, yy, zz)&&vis[xx][yy][zz]==0&&_m[xx][yy][zz]==0)
                    {
                        q.push(qnode(xx, yy, zz));
                        vis[xx][yy][zz]=1;
                    }
                }
            }
        }

    }
    return false;
}

int get_rand()
{
    return rand()%100000<=rate*1000 ? 1 : 0; 
}

int main()
{
    freopen(file_name, "w", stdout);
    cout<<num_data<<endl;
    for(int T=0;T<num_data;++T)
    {
        srand(time(0));
        qnode start_point(0,0,0), end_point(X-1, Y-1, Z-1);
        cout<<X<<" "<<Y<<" "<<Z<<endl;
        cout<<start_point.x<<" "<<start_point.y<<" "<<start_point.z<<" ";
        cout<<end_point.x<<" "<<end_point.y<<" "<<end_point.z<<endl;
        vector<vector<vector<int>>> _map(X, vector<vector<int>>(Y,vector<int>(Z)));
        do
        {
            /* code */
            for(auto &_v : _map)
            {
                for(auto &vi : _v)
                {
                    for(auto &i : vi)
                    {
                        i=get_rand();
                    }
                }
            }
            // cout<<"RAND finish"<<endl;
            _map[start_point.x][start_point.y][start_point.z]=0;
            _map[end_point.x][end_point.y][end_point.z]=0;
        } while (!bfs(_map,start_point, end_point));
        for(size_t i=0;i<_map.size();++i)
        {
            for(auto j=0;j<_map[i].size();++j)
            {
                for(auto k=0;k<_map[i][j].size();++k)
                {
                    cout<<_map[i][j][k]<<" ";
                }
                cout<<endl;
            }
            cout<<endl;
        }
    }
    return 0;
}