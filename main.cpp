#include <math.h>
#include <list>
#include <iostream>
#include <vector>
#include <malloc.h>
#include <map>
#include <algorithm> 


using namespace std;

typedef pair<int, int> pos;
typedef pair<pos, int> PAIR;


typedef struct CarInfo
{
	unsigned int ID;		//车辆编号
	unsigned int T_in;		//申请进入时间点
	unsigned int T_out;		//申请离开时间点
	unsigned int tStart;	//实际进入时间点
	unsigned int tEnd;		//实际离开时间点
	unsigned int t;			//最大等待时间
	unsigned int m;			//质量
	unsigned int T1;		//等待时间
	unsigned int T2;		//总罚时
	unsigned int Z;			//单车耗能
	unsigned int state;		//状态		0：还未申请入库  1：已到申请时间，加入候车队列  2：在最长等待时间内被接入库，在进库路上
							//			3：停在泊车位	 4：已到申请出库时间，加入候车队列  5：在出库路上   6：离开		7:未进库过
	pos p;					//所泊车位置
}Car;

typedef struct parkMap{
	unsigned int stoStart;			//记录停车位到入口的距离
	unsigned int stoEnd;				//记录停车位到出口的距离
	int s;					//记录从入口到停车位再到出口的距离
	int F;					//A*算法的启发式函数，F=G+H
	int G;
	int H;
	bool isEntrance;		//用来标记过道是否为某一停车位出口
	bool isused;			//用于标记停车位是否当前正在被使用
	char c;					//地图内容
	pos parent;
	//list<pos> pathToStart;	//从入口到该停车位的路径
	//list<pos> pathToEnd;	//从该停车位到出口的路径
	//parkMap() :isEntrance(false), isused(false), toStart(0), toEnd(0), F(0), G(0), H(0), parent(pos(-1,-1))
	//{
	//}
}Map;

typedef struct RobotInfo{
	unsigned int ID;					//机器人ID
	unsigned int tStart;				//运动起始点
	unsigned int tEnd;					//运动终点
	bool isFree;						//机器人是否为空闲状态
	pos p;								//机器人所处位置

}Robot;


int k, p, a, b;		//能耗系数k 罚时系数p 泊车机器人系数a 客户停车等待系数b
int w, h;			//停车库宽w 停车库高h
Map *parkmap;
int N;
int M = 1;			//机器人个数
Car *car;
Robot *robot;

pos inlet;	//出入口坐标
pos outlet;

//A*算法用
list<pos> openList;		//开启列表
list<pos> closeList;	//关闭列表
list<pos> path;


vector<PAIR> parkList;		//按降序排列存储各个停车位的坐标

// 函数功能：输入数据
void Input(){
	cin >> k >> p >> a >> b;
	cin >> w >> h;

	//cout << k << " " << p << " " << a << " " << b << endl;

	parkmap = (Map *)malloc(sizeof(Map)* w * h);
	for (int i = 0; i < w; i++){
		for (int j = 0; j < h; j++){
			cin >> parkmap[i * h + j].c;
			parkmap[i * h + j].isEntrance = false;
			parkmap[i * h + j].isused = false;
			parkmap[i * h + j].stoStart = 0;
			parkmap[i * h + j].stoEnd = 0;
			parkmap[i * h + j].G = 0;
			parkmap[i * h + j].H = 0;
			parkmap[i * h + j].F = 0;
			parkmap[i * h + j].parent = pos(-1, -1);
		}
	}

	//for (int i = 0; i < w; i++){
	//	for (int j = 0; j < h; j++){
	//		cout << parkmap[i * h + j].c << " ";
	//	}
	//	cout << endl;
	//}

	cin >> N;
	//cout << N << endl;

	car = (Car*)malloc(sizeof(Car)* N);
	for (int i = 0; i < N; i++){
		cin >> car[i].ID;
		cin >> car[i].T_in;
		cin >> car[i].T_out;
		cin >> car[i].t;
		cin >> car[i].m;
		car[i].t += car[i].T_in;		//将最长等待时间转化为最长等待时刻
		car[i].state = 0;
		car[i].T1 = 0;
		car[i].T2 = 0;
	}

	//for (int i = 0; i < N; i++){
	//	cout << car[i].ID << " ";
	//	cout << car[i].T_in << " ";
	//	cout << car[i].T_out << " ";
	//	cout << car[i].t << " ";
	//	cout << car[i].m << " ";
	//	cout << endl;
	//}

	return;
}

void initRobot(){
	robot = (Robot*)malloc(sizeof(Robot)*M);
	for (int i = 0; i < M; i++){
		robot[i].ID = i;
		robot[i].isFree = true;
		robot[i].p = inlet;
	}
	return;
}

///**********************************************************************************/////////////////////
/// 地图判定函数区

// 说明：判断每个车位有且只有一个入口, 即每个红色区域旁边有且只有一个白色区域；
bool isOnlyEntrance(int x, int y){
	int count = 0;
	if (x - 1 >= 0 && parkmap[(x - 1)*h + y].c == 'X'){
		count++;
		parkmap[(x - 1)*h + y].isEntrance = true;
	}
	if (x + 1 < w  && parkmap[(x + 1)*h + y].c == 'X'){
		count++;
		parkmap[(x + 1)*h + y].isEntrance = true;
	}
	if (y - 1 >= 0 && parkmap[x*h + (y - 1)].c == 'X'){
		count++;
		parkmap[x*h + (y - 1)].isEntrance = true;
	}
	if (y + 1 < h  && parkmap[x*h + (y + 1)].c == 'X'){
		count++;
		parkmap[x*h + (y + 1)].isEntrance = true;
	}
	if (count != 1) 
		return false;
	else 
		return true;
}

// 递归：将从入口所能到的通道（含出口）都进行标记，寻找连通图
void goThrough(bool *isAccessible, int x, int y){

	if (parkmap[x*h + y].c == 'B' || parkmap[x*h + y].c == 'P' || isAccessible[x*h + y] == true)
		return;

	//cout << "x: " << x << " y: " << y << endl;
	//for (int i = 0; i < w; i++){
	//	for (int j = 0; j < h; j++){
	//		cout << isAccessible[i * h + j] << " ";
	//	}
	//	cout << endl;
	//}

	isAccessible[x*h + y] = true;
	if (x + 1 < w)
		goThrough(isAccessible, x + 1, y);
	if (y + 1 < h)
		goThrough(isAccessible, x, y + 1);
	if (x - 1 >= 0)
		goThrough(isAccessible, x - 1, y);
	if (y - 1 >= 0)
		goThrough(isAccessible, x, y - 1);
	return;
}

/*
判断地图是否有效：
1.泊车机器人必须能到达每个车位；
2.每个车位有且只有一个入口, 即每个红色区域旁边有且只有一个白色区域；
3.出口和入口各只有一个，且不重合，分布在地图边缘，泊车机器人从入口和出口都能到达每个车位。
*/
bool isMapValid(){

	int Icount = 0, Ecount = 0;  //出入口数量
	bool *isAccessible = (bool*)malloc(sizeof(bool)* w * h);	//从入口开始，地图是否可达

	// 判断每一个车位是否有且只有一个入口,并寻找入出口坐标
	for (int i = 0; i < w; i++)
	for (int j = 0; j < h; j++){
		isAccessible[i * h + j] = false;
		if (parkmap[i * h + j].c == 'P' && !isOnlyEntrance(i, j)){
			free(isAccessible);
			return false;
		}
		else if (parkmap[i * h + j].c == 'I'){
			Icount++;
			inlet = make_pair(i, j);
		}
		else if (parkmap[i * h + j].c == 'E'){
			Ecount++;
			outlet = make_pair(i, j);
		}
	}

	//判断出入口数量是否均为1,是否都在地图边缘
	//cout << "count: " << Icount << " " << Ecount << endl;
	if (Icount != 1 || Ecount != 1){
		free(isAccessible);
		return false;
	}
	//cout << "start: " << start.first << " " << start.second << endl;
	if (inlet.first != 0 && inlet.first != (w - 1) && inlet.second != 0 && inlet.second != (h - 1)){
		free(isAccessible);
		return false;
	}
	//cout << "end: " << end.first << " " << end.second << endl;
	if (outlet.first != 0 && outlet.first != (w - 1) && outlet.second != 0 && outlet.second != (h - 1)){
		free(isAccessible);
		return false;
	}

	// 标记从入口开始所能到达的通道位置，含出口
	goThrough(isAccessible, inlet.first, inlet.second);
	// 判断出口是否在连通图中
	if (isAccessible[outlet.first * h + outlet.second] == false){
		free(isAccessible);
		return false;
	}

	//for (int i = 0; i < w; i++){
	//	for (int j = 0; j < h; j++){
	//		cout << isAccessible[i * h + j] << " ";
	//	}
	//	cout << endl;
	//}

	// 判断每一个停车位的出口是否都能连通道出口，否则就存在无法到达的车位
	for (int i = 0; i < w; i++)
	for (int j = 0; j < h; j++)
	if (parkmap[i * h + j].isEntrance == true && isAccessible[i * h + j] == false){
		free(isAccessible);
		return false;
	}

	return true;
}

///**********************************************************************************/////////////////////

///**********************************************************************************/////////////////////
//  A*算法

//函数功能：计算G
int calcG(pos p){
	pos parent = parkmap[p.first*h + p.second].parent;
	//cout << "parent: " << parent.first << " " << parent.second << endl;
	int parentG = (parent == pos(-1, -1) ? 0 : parkmap[parent.first*h + parent.second].G);
	//cout << "G: " << parentG + 1 << endl;
	return parentG + 1;
}

//函数功能：计算H
int calcH(pos p){
	//cout << "p: " << p.first << " " << p.second << "  end:" << end.first << " " << end.second << endl;
	//cout << "H: " << abs(p.first - end.first) + abs(p.second - end.second) << endl;
	return abs(p.first - outlet.first) + abs(p.second - outlet.second);
}


//函数功能：计算F
int calcF(pos p){
	//cout << "F: " << map[p.first*h + p.second].G + map[p.first*h + p.second].H << endl;
	return parkmap[p.first*h + p.second].G + parkmap[p.first*h + p.second].H;
}

pos getLowedF(){
	if (!openList.empty()){
		pos posTemp = openList.front();
		for (list<pos>::iterator iter = openList.begin(); iter != openList.end();iter++)
		if (parkmap[(*iter).first*h + (*iter).second].F < parkmap[posTemp.first*h + posTemp.second].F)
			posTemp = (*iter);
		//for (auto &p : openList)
		//if (parkmap[p.first*h + p.second].F < parkmap[posTemp.first*h + posTemp.second].F)
		//	posTemp = p;
		return posTemp;
	}
	return pos(-1, -1);
}

bool isInList(list<pos> List, pos p){
	for (list<pos>::iterator iter = List.begin(); iter != List.end();iter++)
	if (p == (*iter))
		return true;
	//for (auto pvar : List)
	//if (p == pvar)
	//	return true;
	return false;

}

bool isReachable(pos p, pos target, pos pEnd){
	if (target == pEnd) return true;
	if (target.first<0 || target.first>h - 1
		|| target.second < 0 || target.second>w - 1
		|| parkmap[target.first*h + target.second].c == 'P'
		|| parkmap[target.first*h + target.second].c == 'B'
		|| p == target
		|| isInList(closeList, target))
		return false;
	return true;

}

vector<pos> getSurroundPoses(pos p, pos pEnd){
	vector<pos> surroundPoses;

	pos target;
	target.first = p.first + 1, target.second = p.second;
	if (isReachable(p, target, pEnd))
		surroundPoses.push_back(target);
	target.first = p.first - 1, target.second = p.second;
	if (isReachable(p, target, pEnd))
		surroundPoses.push_back(target);
	target.first = p.first, target.second = p.second + 1;
	if (isReachable(p, target, pEnd))
		surroundPoses.push_back(target);
	target.first = p.first, target.second = p.second - 1;
	if (isReachable(p, target, pEnd))
		surroundPoses.push_back(target);

	return surroundPoses;
}

// 利用A*算法寻找pstart到pend的路径
bool findPath(pos pStart, pos pEnd){
	openList.push_back(pStart);
	cout << "findpath:(" << pStart.first << "," << pStart.second << ") (" << pEnd.first << "," << pEnd.second << ")" << endl;
	while (!openList.empty()){
		pos curPos = getLowedF();
		openList.remove(curPos);

		//cout << "openList:  ";
		//for (auto var : openList){
		//	cout << "(" << var.first << "," << var.second << ") ";
		//}
		//cout << endl;

		closeList.push_back(curPos);

		//cout << "closeList:  ";
		//for (auto var : closeList){
		//	cout << "(" << var.first << "," << var.second << ") ";
		//}
		//cout << endl;

		//1,找到当前周围4个格中可以通过的格子(上下左右）
		vector<pos> surroundPoses = getSurroundPoses(curPos, pEnd);
		for (vector<pos>::iterator target = surroundPoses.begin(); target != surroundPoses.end(); target++){
		//for (auto target : surroundPoses){
			//cout << "target: " << target.first << " " << target.second << endl;
			//2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H  
			if (!isInList(openList, (*target))){
				parkmap[(*target).first*h + (*target).second].parent = curPos;

				parkmap[(*target).first*h + (*target).second].G = calcG(*target);
				parkmap[(*target).first*h + (*target).second].H = calcH(*target);
				parkmap[(*target).first*h + (*target).second].F = calcF(*target);

				openList.push_back(*target);
			}
			//3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F  
			else{
				int tempG = calcG(*target);
				if (tempG < parkmap[(*target).first*h + (*target).second].G){
					parkmap[(*target).first*h + (*target).second].parent = curPos;
					parkmap[(*target).first*h + (*target).second].G = tempG;
					parkmap[(*target).first*h + (*target).second].F = calcF(*target);
				}
			}
			bool isFound = isInList(openList, pEnd);
			if (isFound)
				return isFound;
		}
	}
	return false;
}

// 计算地图中两点pstart到pend的距离
int getLength(pos pStart, pos pEnd){
	//cout << "(" << pStart.first << "," << pStart.second << ") (" << pEnd.first << "," << pEnd.second << ")" << endl;
	for (int i = 0; i < w; i++){
		for (int j = 0; j < h; j++){
			parkmap[i * h + j].G = 0;
			parkmap[i * h + j].H = 0;
			parkmap[i * h + j].F = 0;
			parkmap[i * h + j].parent = pos(-1, -1);
		}
	}
	openList.clear();
	closeList.clear();
	bool isFound = findPath(pStart, pEnd);
	int len = 0;
	if (isFound){
		pos temp = pEnd;
		pos pNull = make_pair(-1, -1);
		while (temp != pNull){
			path.push_back(temp);
			len++;
			cout << "(" << temp.first << "," << temp.second << ")  ";
			temp = parkmap[temp.first*h + temp.second].parent;
		}
		cout << endl;
	}
	else{
		cout << "not found!" << endl;
	}
	return len - 1 >= 0 ? len - 1 : 0;
}

///**********************************************************************************/////////////////////




// 计算每个停车位到出口和入口的距离，并求和
void calLength(){
	for (int i = 0; i < w; i++){
		for (int j = 0; j < h; j++){
			if (parkmap[i * h + j].c == 'P'){
				cout << "pos: x: " << i << "   y:" << j << endl;
				//cout << "(" << inlet.first << "," << inlet.second << ") (" << outlet.first << "," << outlet.second << ")" << endl;
				parkmap[i * h + j].stoStart = getLength(pos(i, j), inlet);
				parkmap[i * h + j].stoEnd = getLength(outlet, pos(i, j));
				parkmap[i * h + j].s = parkmap[i * h + j].stoStart + parkmap[i * h + j].stoEnd;
			}
		}
	}

	cout << "toStart:" << endl;
	for (int i = 0; i < w; i++){
		for (int j = 0; j < h; j++){
			cout << parkmap[i * h + j].stoStart << " ";
		}
		cout << endl;
	}
	cout << endl << "toEnd:" << endl;
	for (int i = 0; i < w; i++){
		for (int j = 0; j < h; j++){
			cout << parkmap[i * h + j].stoEnd << " ";
		}
		cout << endl;
	}
}

int cmp(const PAIR &x, const PAIR &y){
	return x.second < y.second;
}

// 对停车位到出口和入口距离和进行降序排列
void vecSort(){


	for (int i = 0; i < w; i++)
	for (int j = 0; j < h; j++)
	if (parkmap[i * h + j].c == 'P'){
		parkList.push_back(make_pair(pos(i, j), parkmap[i * h + j].s));
	}
	cout << "before:" << endl;
	for (vector<PAIR>::iterator iter = parkList.begin(); iter != parkList.end(); iter++){
		PAIR temp = *iter;
		cout << "(" << temp.first.first << "," << temp.first.second << "):  " << temp.second << endl;
	}
	sort(parkList.begin(), parkList.end(), cmp);

	cout << "after:" << endl;
	for (vector<PAIR>::iterator iter = parkList.begin(); iter != parkList.end(); iter++){
		PAIR temp = *iter;
		cout << "(" << temp.first.first << "," << temp.first.second << "):  " << temp.second << endl;
	}

}



void pickInCar(Robot &robot, int ID, int curTime, pos target){
	car[ID].tStart = curTime;
	car[ID].state = 2;
	car[ID].p = target;
	robot.isFree = false;
	robot.tStart = curTime;
	robot.tEnd = curTime + getLength(inlet, target);
	robot.p = target;
	parkmap[target.first*h + target.second].isused = true;
}

void pickOutCar(Robot &robot, int ID, int curTime){
	int costTime = getLength(car[ID].p, outlet);
	car[ID].tEnd = curTime + costTime;
	car[ID].T1 = (car[ID].tStart - car[ID].T_in) + (car[ID].tEnd - car[ID].T_out);
	car[ID].state = 5;
	car[ID].p = outlet;
	robot.isFree = false;
	robot.tStart = curTime;
	robot.tEnd = curTime + costTime;
	robot.p = outlet;
	parkmap[car[ID].p.first*h + car[ID].p.second].isused = false;
}

// 函数功能：移动空载机器人robot到target位置处
void MoveRobot(Robot &robot, int curTime, pos target){
	int costTime = getLength(robot.p, target);
	robot.isFree = false;
	robot.tStart = curTime;
	robot.tEnd = curTime + costTime;
	robot.p = target;
}

void dispatch(){
	unsigned int curTime = 0;
	vector<int> ICarList;
	vector<int> ECarList;

	while (1){
		cout << curTime << " : " << endl;
		for (int i = 0; i < N; i++){
			if (car[i].state == 0){
				if (car[i].T_in == curTime){			//车已经申请，未过等待时间，加入候车队伍
					ICarList.push_back(i);
					car[i].state = 1;
				}
			}
			else if (car[i].state == 1){
				if (car[i].t == curTime + 1){					//过了最长等待时间，移出候车队伍，并离开
					for (vector<int>::iterator iter = ICarList.begin(); iter != ICarList.end(); iter++){
						if (car[i].ID == (*iter)){
							ICarList.erase(iter);
						}
					}
					car[i].state = 7;
				}
			}
			else if (car[i].state == 2 && curTime == car[i].T_out){
				ECarList.push_back(i);
				car[i].state = 4;
			}
			cout << "state: " << car[i].state << endl;

		}
		cout << "ICarList: ";
		for (vector<int>::iterator iter = ICarList.begin(); iter != ICarList.end(); iter++){
			cout << (*iter)<< " ";
		}
		cout << endl;
		cout << "ECarList: ";
		for (vector<int>::iterator iter = ECarList.begin(); iter != ECarList.end(); iter++){
			cout << (*iter) << " ";
		}
		cout << endl;
		if (!robot[0].isFree&&curTime == robot[0].tEnd){
			robot[0].isFree = true;
		}
		if (robot[0].isFree){
			if (!ICarList.empty()){
				if (robot[0].p != inlet){
					MoveRobot(robot[0], curTime, inlet);
					curTime++;
					continue;
				}
				//接入库操作
				cout << " In : " << ICarList.front() << " no 0 " << curTime << " ";
				pos target;
				for (vector<PAIR>::iterator iter = parkList.begin(); iter != parkList.end(); iter++){
					if (parkmap[(*iter).first.first*h + (*iter).first.second].isused == false){
						target = (*iter).first;
						break;
					}
				}
				cout << "target : " << target.first << " " << target.second << endl;
				pickInCar(robot[0], ICarList.front(), curTime, target);

				ICarList.erase(ICarList.begin());
			}
		}
		if (robot[0].isFree){
			if (!ECarList.empty()){
				if (robot[0].p != car[ECarList.front()].p){
					MoveRobot(robot[0], curTime, car[ECarList.front()].p);
					curTime++;
					continue;
				}
				//接出库操作
				cout << " Out : " << ECarList.front() << " no 0 " << curTime << " ";

				pickOutCar(robot[0], ECarList.front(), curTime);

				ECarList.erase(ECarList.begin());
			}
		}


		cout << "robot is busy: " << robot[0].isFree << "  target:  " << robot[0].p.first << "," << robot[0].p.second << endl;
		curTime++;
	}
}

int main(){

	Input();
	if (!isMapValid())
		cout << "NO";
	else
		cout << "YES" << endl;

	calLength();		
	vecSort();
	initRobot();
	dispatch();

	system("pause");
	return 0;
}