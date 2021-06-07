#pragma once
#ifndef _RRT_H
#define _RRT_H

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <typeinfo>
#include <time.h>
#include "planner.h"

using namespace std;


class node {
private:
    int x, y, z;                // �ڵ�����
    vector<float> pathX, pathY;// ·��
    node* parent;              // ���ڵ�
    float cost;
public:
    node(int _x, int _y, int _z);
    int getX();
    int getY();
    int getZ();
	void setcost(float);
	float getcost();
    void setParent(node*);
    node* getParent();
};

class RRT :public Abstract_planner
{
private:
    node* startNode, * goalNode;          // ��ʼ�ڵ��Ŀ��ڵ�
    vector< vector< vector<int> > > obstacleList; // �ϰ���
    vector<node*> nodeList;               // 
    int stepSize;                       // ����
    int goal_sample_rate;

    // ���������������һ��α���������ʵ����һ�����з��������й̶����㷨��ֻ�е����Ӳ�ͬʱ�����вŲ�ͬ��
    // ���Բ�Ӧ�ð����ӹ̶��ڳ����У�Ӧ��������������������ӣ����������ʱ��ʱ��ȡ�
    random_device goal_rd;                // random_device��������������Ϊ���ӵ�������޷�������ֵ��
    mt19937 goal_gen;                     // mt19937��һ�ָ�Ч������������㷨
    uniform_int_distribution<int> goal_dis;  //�����Դ�������Դ����������㷨�����������

    random_device area_rd1;
    mt19937 area_gen1;
    uniform_real_distribution<float> area_dis1;

	random_device area_rd2;
	mt19937 area_gen2;
	uniform_real_distribution<float> area_dis2;
	
	random_device area_rd3;
	mt19937 area_gen3;
	uniform_real_distribution<float> area_dis3;



public:
    RRT(node*, node*, vector<vector<vector<int>>>&, int, int);
    node* getNearestNode(const vector<float>&);
    bool collisionCheck(node*);
    vector<node*> planning();
	virtual list<Point*>* getPath(Point* start_point, Point* end_point, double& time, float& distance);


};

#endif
