#ifndef _A_STAR_
#define _A_STAR_

#include "planner.h"
#include <list>

class A_star:public Abstract_planner{
    public:
        void InitAstar(vector<vector<vector<int>>> &_map);
        
        
        std::vector<Point*> getPath(Point &start_point,Point &end_point);
        std::vector<Point*>getSurroundPoints(const Point *point)const;
        Point* FindPath(Point &start_point,Point &end_point);
        bool isCanreach(const Point* point,const Point* target)const;
        Point* isInList(const std::list<Point*>& list,const Point* point)const;
        Point* getLeastFpoint();

        int calcG(Point *temp_start,Point *end);
        int calcH(Point *start,Point *end);
        int calcF(Point *point);

    private:
        std::list<Point*> openlist;
        std::list<Point*> closelist;

};

Point* A_star::getLeastFpoint(){
    if(!openlist.empty()){
        auto resPoint = openlist.front();
        for(auto& point:openlist)
        if(point->F<resPoint->F)
            resPoint = point;
        return resPoint;
    }
    return NULL;
}
int A_star::calcG(Point *temp_start,Point *end){
    return temp_start-end;
}

int A_star::calcH(Point *start,Point *end){
    return start-end;
}

int A_star::calcF(Point *point){
    return point->G+point->H;
}
void A_star::InitAstar(vector<vector<vector<int>>> &_map){
    vector<vector<vector<int>>> realmap(map.size()*2-1,vector<vector<int>>(map[0].size()*2-1,vector<int>(map[0][0].size()*2-1)));
    for(int i=0;i<map.size();i++)
        for(int j=0;j<map.size();j++)
            for(int k=0;k<map.size();k++){
                if(map[i][j][k]==1){
                    realmap[2*i][2*j][2*k] = 1;
                }
            };
    map = realmap;
}
Point* A_star::FindPath(Point &start_point,Point &end_point){
    openlist.push_back(new Point(start_point.x,start_point.y,start_point.z));
    while(!openlist.empty()){
        
    }
    return NULL;
}

Point* A_star::isInList(const std::list<Point*>& list,const Point* point)const{
    for(auto p:list)
        if(p->x==point->x&&p->y==point->y&&p->z ==point->z)
            return p;
    return NULL;
}
bool A_star::isCanreach(const Point* point,const Point* target)const{
    if(target->x<0||target->y<0||target->z<0||target->x>=map.size()||target->y>=map[0].size()||target->z>=map[0][0].size()
        ||map[point->x][point->y][point->z]==1||(point->x==target->x&&point->y==target->y&&point->z==target->z)||isInList(closelist,target))
        return false;
    else return true;
}
std::vector<Point*> A_star::getPath(Point &start_point,Point &end_point){
    Point* result = FindPath(start_point,end_point);
    std::vector<Point*>path;
    while(result){
        path.push_back(result);
        result = result->parent;
    }
    openlist.clear();
    closelist.clear();
    return path;
}


vector<Point*> A_star::getSurroundPoints(const Point *point)const{
    vector<Point*>surround_points;
    for(int i=point->x-1;i<=point->x+1;i++)
        for(int j=point->y-1;j<=point->y+1;j++)
            for(int k=point->z-1;k<=point->z+1;k++){
                Point* surround_point = new Point(i,j,k);
                if(isCanreach(point,surround_point)) surround_points.push_back(surround_point);
                else delete surround_point;
            }
    return surround_points;
}
#endif