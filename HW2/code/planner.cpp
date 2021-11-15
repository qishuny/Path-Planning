/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"

#include <iostream>       // std::cout
#include <queue>          // std::priority_queue
#include <vector>         // std::vector
#include <functional>     // std::greater
#include <unordered_set>
#include <string>
#include <unordered_map>
#include <float.h>
#include <limits.h>
#include <stdlib.h>
#include <chrono>
#include <ctime>
#include <ratio>
#include <stack>
#include <utility>   
#include <cstdint>
#include <cstring>
#include <algorithm>
/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10

// PRM parameters
#define MAXGRAPH 10000
#define RADIUS 1
// RRT parameters
#define EPSILON 0.3
#define MAXITR 500000

// RRT* parameters
#define RI 2
using namespace std;
using namespace std::chrono;

class Node{
public:
  // general
  vector<double> angle_rad;
  int numofDOFs;
  Node* parent = nullptr;

  // PRM
  vector<int> successor;
  double g = (double) DBL_MAX;
  int index;
  bool open = false;

  // RRT*
  double cost = (double) DBL_MAX;

  Node(int n){
    numofDOFs = n;
  }
    
  Node(int numofDOFs, double* q_angleV_rad, Node* p){
    for (int i = 0; i <numofDOFs; i++){
      angle_rad.push_back(q_angleV_rad[i]);
    }
    parent = p;
  }

};

class Compare_gVal{
public:
    bool operator()(Node* a, Node* b){
        return (a->g > b->g);
    }
};

typedef struct {
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} bresenham_param_t;

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size)
{
    double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
    {
      params->Y1=p1x;
      params->X1=p1y;
      params->Y2=p2x;
      params->X2=p2y;
    }
  else
    {
      params->X1=p1x;
      params->Y1=p1y;
      params->X2=p2x;
      params->Y2=p2y;
    }

   if ((p2x - p1x) * (p2y - p1y) < 0)
    {
      params->Flipped = 1;
      params->Y1 = -params->Y1;
      params->Y2 = -params->Y2;
    }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX=params->X2-params->X1;
  params->DeltaY=params->Y2-params->Y1;

  params->IncrE=2*params->DeltaY*params->Increment;
  params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
  params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
    {
      *y = params->XIndex;
      *x = params->YIndex;
      if (params->Flipped)
        *x = -*x;
    }
  else
    {
      *x = params->XIndex;
      *y = params->YIndex;
      if (params->Flipped)
        *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
    {
      return 0;
    }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
    {
      params->DTerm += params->IncrNE;
      params->YIndex += params->Increment;
    }
  return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
		   int x_size,
 		   int y_size)

{
	bresenham_param_t params;
	int nX, nY; 
    short unsigned int nX0, nY0, nX1, nY1;

    //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
    
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

    //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
            return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
		   int x_size, int y_size)
{
    double x0,y0,x1,y1;
    int i;
    
 	//iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
    y1 = 0;
	for(i = 0; i < numofDOFs; i++)
	{
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
				return 0;
	}    
    return 1;
}


void Random_Config(double *map, int x_size, int y_size, int numofDOFs, double* q_rand){
  while(true){
    for(int i = 0; i < numofDOFs; i++){
			int temp = rand() % 360;
			q_rand[i]= (double)temp / 360 * 2 * PI;
		}   
		if(IsValidArmConfiguration(q_rand, numofDOFs, map, x_size, y_size))
		{
			return;
		}else{
    }
  }
  return;
}

vector<double> calc_direction(double* startConfig, double* endConfig, int numofDOFs){
  vector<double> unit_vector;
  double mag = 0.0;
  for (int i = 0; i < numofDOFs; i++){
    double temp = endConfig[i] - startConfig[i];
    unit_vector.push_back(temp);
    mag += temp * temp;
  } 
  for (int i = 0; i < numofDOFs; i++){
    unit_vector[i] = unit_vector[i] / mag;
  }
  return unit_vector;
}

double calc_distance(double* startConfig, double* endConfig, int numofDOFs){
  double mag = 0.0;
  for (int i = 0; i < numofDOFs; i++){
    mag += (endConfig[i] - startConfig[i]) * (endConfig[i] - startConfig[i]);
  }
  return mag;
}



bool isEqual(double* anglea, double* angleb, int numofDOFs)
{
  if (anglea == nullptr || angleb == nullptr) return false;
	for(int i=0; i<numofDOFs; i++){
		if(anglea[i] != angleb[i]){
			return false;
		}
	}
	return true;
}

bool Check_Collision(double* startConfig, double* endConfig, int numofDOFs, double* map, int x_size, int y_size)
{

  int N = 10;

  vector<double> dir = calc_direction(startConfig, endConfig, numofDOFs);
  double distance = calc_distance(startConfig, endConfig, numofDOFs);
  double* midConfig = new double[numofDOFs];

  for(int i = 1; i < N; i++)
  {
    vector<double> midConfig;
    double scale = distance / N;
    for(int j = 0; j < numofDOFs; j++)
    {
      midConfig.push_back(startConfig[j] + scale * i * dir[j]);
    }
    if(!IsValidArmConfiguration(midConfig.data(), numofDOFs, map, x_size, y_size))
    {
      return false;
    }
  }
  return true;
}
double* Extend(Node* startNode, double* endConfig, int numofDOFs, double* map, int x_size, int y_size){

  
  double* startConfig = new double[numofDOFs];
  for (int i = 0; i < numofDOFs; i++){
    startConfig[i] = startNode->angle_rad[i];
  }
  if (!Check_Collision(startConfig, endConfig, numofDOFs, map, x_size, y_size)){
    return nullptr;
  }
  double distance = 0;
  distance = calc_distance(startConfig, endConfig, numofDOFs);
  vector<double> unitVec = calc_direction(startConfig, endConfig, numofDOFs);

  double* tempConfig = new double[numofDOFs];

  if (distance > EPSILON){
    for(int i = 0; i < numofDOFs; i++){
      tempConfig[i] = startNode->angle_rad[i] + EPSILON * unitVec[i];
    }
  }else{
    memcpy(tempConfig, endConfig, numofDOFs*sizeof(double));
    
  }
 
  delete startConfig;
  if (!IsValidArmConfiguration(tempConfig, numofDOFs, map, x_size, y_size) ){
    return nullptr;
  }else{
    return tempConfig;
  }
}
class PRMap{
public:
  int numofDOFS;
  int startIdx;
  int endIdx;
  Node* startPointer;
  Node* endPointer;
  // key is the id: the number of insertion, value is the pointer to the Node
  unordered_map<int, Node*> vertices;

 

  PRMap(int n){
    numofDOFS = n;
  }

  void insertNode(int id, Node* newNode){
    vertices[id] = newNode;
  }

  void Build_RoadMap(double* rand_angle, double* map, int x_size, int y_size, int numofDOFs){
    for(int i = 0; i < MAXGRAPH; i++)
    {    
      Random_Config(map, x_size, y_size, numofDOFs, rand_angle);
    
      Node* newNode = new Node(numofDOFs, rand_angle, nullptr);
      newNode->index = i;
      
      vector<pair<double,int>> distances;
      
      for(const auto &element : vertices)
      {
        double distance = calc_distance(newNode->angle_rad.data(), element.second->angle_rad.data(), numofDOFs);
        distances.push_back(make_pair(distance, element.first));
      }

      sort(distances.begin(), distances.end());

      int connected = 0;
      int k = 0;
      while(connected < 3 && k < i){
        
        if(Check_Collision(newNode->angle_rad.data(), vertices.at(distances[k].second)->angle_rad.data(), numofDOFs, map, x_size, y_size) && distances[k].first < RADIUS)
        { 
          // printf("%d ", k);
          newNode->successor.push_back(distances[k].second);
          vertices.at(distances[k].second)->successor.push_back(i);
          connected++;
        }
        k++;
      }
      vertices[i] = newNode;
    }
  }

  bool addStartGoal(double* armstart_anglesV_rad, double* armgoal_anglesV_rad, double* map, int x_size, int y_size, int numofDOFs){
    Node* startNode = new Node(numofDOFs, armstart_anglesV_rad, nullptr);
    Node* endNode = new Node(numofDOFs, armgoal_anglesV_rad, nullptr);
    int k;
    startNode->index = -1;
    startNode->g = 0.0;
    vector<pair<double,int>> distances = {};
    for(int i = 0; i < MAXGRAPH; i++)
    {
      double distance = calc_distance(startNode->angle_rad.data(), vertices.at(i)->angle_rad.data(), numofDOFs);
      distances.push_back(make_pair(distance,i));
    }
    sort(distances.begin(), distances.end());
    
    bool isSTARTConnected = false;
    k = 0;
    while(!isSTARTConnected && k < MAXGRAPH){
      if(Check_Collision(startNode->angle_rad.data(), vertices.at(distances[k].second)->angle_rad.data(), numofDOFs, map, x_size, y_size)){
        startNode->successor.push_back(distances[k].second);
        vertices.at(distances[k].second)->successor.push_back(-1);
        isSTARTConnected = true;
      }
      k++;
    }
    vertices[-1] = startNode;
    startIdx = -1;
    startPointer = startNode;
    endNode->index = -2;
    distances.clear();
    for(int i = 0; i < MAXGRAPH; i++){
      double distance = calc_distance(endNode->angle_rad.data(), vertices.at(i)->angle_rad.data(), numofDOFs);
      distances.push_back(make_pair(distance,i));
    }
    sort(distances.begin(), distances.end());

    bool isGOALConnected = false;
    k = 0;
    while(!isGOALConnected && k < MAXGRAPH){
      if(Check_Collision(endNode->angle_rad.data(), vertices.at(distances[k].second)->angle_rad.data(), numofDOFs, map, x_size, y_size)){
        endNode->successor.push_back(distances[k].second);
        vertices.at(distances[k].second)->successor.push_back(-2);
        isGOALConnected = true;
      }
      k++;
    }
    vertices[-2] = endNode;
    endIdx = -2;
    endPointer = endNode;
    return isSTARTConnected && isGOALConnected;
  }

  bool generatePath(){
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> openList;
    unordered_set <int> closeList;
  
    openList.emplace(make_pair(0.0, -1));

    bool found = false;
    while(!openList.empty() ){
      pair<double, int> currentNode = openList.top();
      openList.pop();
      int currentIdx = currentNode.second;
      double currentgVal = currentNode.first;
      // cout << currentgVal << "current gval" << endl;
      closeList.insert(currentIdx);
      for(int i = 0; i < vertices.at(currentIdx)->successor.size(); i++){
        int newIdx = vertices.at(currentIdx)->successor[i];
        if (newIdx == endIdx){
          vertices[newIdx]->parent = vertices.at(currentIdx);
          found = true;
          break;
        }
        if (closeList.find(newIdx) == closeList.end() && !vertices.at(newIdx)->open){
          double newg = currentgVal + calc_distance(vertices.at(newIdx)->angle_rad.data(), vertices.at(currentIdx)->angle_rad.data(), numofDOFS);
          vertices.at(newIdx)->parent = vertices.at(currentIdx);
          vertices.at(newIdx)->open = true;
          openList.emplace(make_pair(newg, newIdx));
        }
      }
    } 
    return found;
  }
};

static void plannerPRM(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{   
  *plan = NULL;
	*planlength = 0;

  srand (1);

  int k;

  PRMap graph(numofDOFs);

  double* rand_angle = new double[numofDOFs]; 
  graph.Build_RoadMap(rand_angle, map, x_size, y_size, numofDOFs);
  cout << "build map done" << endl;

  bool isConnected = graph.addStartGoal(armstart_anglesV_rad, armgoal_anglesV_rad, map, x_size, y_size, numofDOFs);
  if (isConnected == true){
    cout << "start and goal are connected to the graph" << endl;
  }
  bool pathFound = graph.generatePath();  
  cout << "map size" << graph.vertices.size() << endl;
  if (pathFound){
    double cost = 0.0;
    stack<int,vector<int>> path;
    int currentIdx = graph.endIdx;
    int j = 0;
    while(currentIdx != graph.startIdx)
    {
      double* prev = graph.vertices.at(currentIdx)->angle_rad.data();
      path.push(currentIdx);
      currentIdx = graph.vertices.at(currentIdx)->parent->index;
      cost += calc_distance(prev, graph.vertices.at(currentIdx)->angle_rad.data(), numofDOFs);
      j++;
    }
    path.push(graph.startIdx);
    cout << "cost " << cost << endl;
    *planlength = j+1;

    *plan = (double**) malloc((j+1)*sizeof(double*));
    for(int k = 0; k < j+1; k++)
    {
      (*plan)[k] = (double*) malloc(numofDOFs*sizeof(double));
      for(int l = 0; l < numofDOFs; l++)
      {
        (*plan)[k][l] = graph.vertices.at(path.top())->angle_rad[l];
      }
      path.pop();
    }
  }else{
    cout << "PATH NOT FOUND" << endl;
  }
  
  
  return;
} 


class RRTtree{
public:
  int numofDOFs;
  Node* rootNode;
  Node* goalNode;
  vector<Node*> vertices;


  RRTtree(int n){
    numofDOFs = n;
	}

  void insertNode(Node* nodeTemp){
    vertices.push_back(nodeTemp);
  }

  int treeSize(){
    return vertices.size();
  }
  
  Node* Nearest_Neighbor(double* q)
	{
    int idx = -1;
		double minDist = (double) DBL_MAX;
		for(int i = 0; i < vertices.size(); i++){
		  double tempDist = 0;
			for(int j = 0; j < numofDOFs; j++){
				tempDist += (vertices[i]->angle_rad[j] - q[j]) * (vertices[i]->angle_rad[j] - q[j]);
			}
		
			if(tempDist <= minDist)
			{
				minDist = tempDist;
				idx = i;
			}
		}
		return vertices[idx];
	}
};

static void plannerRRT(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{   
  *plan = NULL;
	*planlength = 0;

  srand (1);

  Node* q_start = new Node(numofDOFs, armstart_anglesV_rad, nullptr);

  RRTtree tree(numofDOFs);
  tree.insertNode(q_start);
  tree.rootNode = q_start;

  double* rand_angle = new double[numofDOFs]; 
  bool pathFound = false;
  int count = 0;
  while (!pathFound && count <= MAXITR){
    if (count % 100 == 0){
      for(int i = 0; i < numofDOFs; i++){
        double temp = armgoal_anglesV_rad[i];
        rand_angle[i]= (double)temp;
      }     
    }else{
      Random_Config(map, x_size, y_size, numofDOFs, rand_angle);
    }

    count++;

    Node* q_near = tree.Nearest_Neighbor(rand_angle);

    double* newConfig = Extend(q_near, rand_angle, numofDOFs, map, x_size, y_size);
    if (newConfig == nullptr){
      // cout<< "invalid" <<endl;
      continue;
    }else if(isEqual(newConfig, armgoal_anglesV_rad, numofDOFs)){
      cout << "goal found" << endl;
      Node* q_goal =  new Node(numofDOFs, newConfig, q_near);
      tree.insertNode(q_goal);
      tree.goalNode = q_goal;

      pathFound = true;
      break;
    }
    else{
      Node* q_new = new Node(numofDOFs, newConfig, q_near);
      tree.insertNode(q_new);
    }
    delete newConfig;
  }

  if (pathFound){
    double cost = 0.0;
    vector<vector<double>> path;
    Node* q_path = tree.goalNode;
    while (q_path->parent != nullptr){
      path.push_back(q_path->angle_rad);
      double* prev = q_path->angle_rad.data();
      
      q_path = q_path->parent;
      cost += calc_distance(prev, q_path->angle_rad.data(), numofDOFs);

    }
    path.push_back(q_path->angle_rad);
    cout << "number of vertices " << tree.treeSize() << endl;
    cout << "cost of the tree " << cost << endl;
    int numofsamples = path.size();
    *plan = (double**) malloc(numofsamples * sizeof(double*));
    for (int i = 0 ; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs * sizeof(double)); 
        for(int j = 0; j < numofDOFs; j++){   
          (*plan)[i][j] = path[numofsamples - 1 - i][j];
        }
    }    
    *planlength = numofsamples;
  }else{
    cout << "path not found" << endl;
    cout << "tree size" << tree.treeSize() << endl;
  }
  return;
}

class RRTCONNECTtree{
public:
  int numofDOFs;
  int treeid;
  Node* rootNode;
  Node* endNodeA;
  Node* endNodeB;
  Node* goalNode;
  vector<Node*> verticesA;
  vector<Node*> verticesB;

  RRTCONNECTtree(int n, int id){
    numofDOFs = n;
    treeid = id;
	}

  void insertNode(Node* nodeTemp, int id){
    if (id == 0){
      verticesA.push_back(nodeTemp);
    }else if (id == 1){
      verticesB.push_back(nodeTemp);
    }
    
  }

  int treeSize(int id){
    if (id == 0){
      return verticesA.size();
    }else if (id == 1){
      return verticesB.size();
    }
    
  }
  
  Node* Nearest_Neighbor(double* q, int id)
	{
    int idx = -1;
		double minDist = (double) DBL_MAX;
    if (id == 0){
      for(int i = 0; i < verticesA.size(); i++){
        double tempDist = 0;
        for(int j = 0; j < numofDOFs; j++){
          tempDist += (verticesA[i]->angle_rad[j] - q[j]) * (verticesA[i]->angle_rad[j] - q[j]);
        }
      
        if(tempDist <= minDist)
        {
          minDist = tempDist;
          idx = i;
        }
      }
      return verticesA[idx];
    }else if (id == 1){
      for(int i = 0; i < verticesB.size(); i++){
        double tempDist = 0;
        for(int j = 0; j < numofDOFs; j++){
          tempDist += (verticesB[i]->angle_rad[j] - q[j]) * (verticesB[i]->angle_rad[j] - q[j]);
        }
      
        if(tempDist <= minDist)
        {
          minDist = tempDist;
          idx = i;
        }
      }
      return verticesB[idx];
    }		
    return nullptr;
	}

  void Swap(){
    treeid = (treeid == 0) ? 1 : 0;
  }

};

static void plannerRRTCONNECT(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{   
  *plan = NULL;
	*planlength = 0;

  srand (1);
  RRTCONNECTtree tree(numofDOFs, 0);

  Node* q_start = new Node(numofDOFs, armstart_anglesV_rad, nullptr);
  tree.insertNode(q_start, 0);
  tree.rootNode = q_start;

  Node* q_goal = new Node(numofDOFs, armgoal_anglesV_rad, nullptr);
  tree.insertNode(q_goal, 1);
  tree.goalNode = q_goal;

  double* rand_angle = new double[numofDOFs]; 
  bool pathFound = false;
  int count = 0;
  while (!pathFound && count <= MAXITR){
    Random_Config(map, x_size, y_size, numofDOFs, rand_angle);
    count++;
    Node* q_near = tree.Nearest_Neighbor(rand_angle, tree.treeid);
    if (q_near == nullptr){
      cout << "shouldnt happen" << endl;
    }
    double* newConfig = Extend(q_near, rand_angle, numofDOFs, map, x_size, y_size);
    if (newConfig == nullptr){
      // cout<< "invalid" <<endl;
      continue;
    }
    Node* q_newA = new Node(numofDOFs, newConfig, q_near);
    tree.insertNode(q_newA, tree.treeid);

    tree.Swap();
    Node* q_near2 = tree.Nearest_Neighbor(newConfig, tree.treeid);
    double* connectConfig = Extend(q_near2, newConfig, numofDOFs, map, x_size, y_size);

    if (connectConfig == nullptr){
      // cout<< "invalid" <<endl;
      continue;
    }
    Node* q_newB = new Node(numofDOFs, connectConfig, q_near2);
    tree.insertNode(q_newB, tree.treeid);

    if (isEqual(connectConfig, newConfig, numofDOFs)){
      tree.endNodeA = (tree.treeid == 0) ? q_newB : q_newA;
      tree.endNodeB = (tree.treeid == 0) ? q_newA : q_newB;
      pathFound = true;
    }
  }


  // for (int i = 0; i < tree.treeSize(0); i++){
  //   cout << "Node in A "; 
  //   for ( int j = 0; j < numofDOFs; j++){
  //     cout << tree.verticesA[i]->angle_rad[j] << " ";
  //   }
  //   cout << endl;
  // }


  // for (int i = 0; i < tree.treeSize(1); i++){
  //   cout << "Node in B "; 
  //   for ( int j = 0; j < numofDOFs; j++){
  //     cout << tree.verticesB[i]->angle_rad[j] << " ";
  //   }
  //   cout << endl;
  // }

  // cout << "treeA size " << tree.treeSize(0) << endl;
  // cout << "treeB size " << tree.treeSize(1) << endl;
  cout << "tree size" << tree.treeSize(0) + tree.treeSize(1) << endl;
  if (pathFound){
    double cost = 0.0;
    cout << "FOUND PATH" << endl;
    vector<vector<double>> pathA;
    Node* q_pathA = tree.endNodeA;
    while (q_pathA->parent != nullptr){
      double* prev = q_pathA->angle_rad.data();
      pathA.push_back(q_pathA->angle_rad);
      q_pathA = q_pathA->parent;
      cost += calc_distance(prev, q_pathA->angle_rad.data(), numofDOFs);
    }
    pathA.push_back(q_pathA->angle_rad);


    vector<vector<double>> pathB;
    Node* q_pathB = tree.endNodeB;
    while (q_pathB->parent != nullptr){
      double* prev = q_pathB->angle_rad.data();
      pathB.push_back(q_pathB->angle_rad);
      q_pathB = q_pathB->parent;
      cost += calc_distance(prev, q_pathB->angle_rad.data(), numofDOFs);
    }
    pathB.push_back(q_pathB->angle_rad);

    cout << "total cost" << cost << endl;
    int numofsamplesA = pathA.size();
    int numofsamplesB = pathB.size();
    int numofsamples = numofsamplesA + numofsamplesB;
    *plan = (double**) malloc(numofsamples * sizeof(double*));
    for (int i = 0 ; i < numofsamplesA; i++){
        (*plan)[i] = (double*) malloc(numofDOFs * sizeof(double)); 
        for(int j = 0; j < numofDOFs; j++){   
          (*plan)[i][j] = pathA[numofsamplesA - 1 - i][j];
        }
    }    
    for (int i = 0 ; i < numofsamplesB; i++){
        (*plan)[numofsamplesA + i] = (double*) malloc(numofDOFs * sizeof(double)); 
        for(int j = 0; j < numofDOFs; j++){   
          (*plan)[numofsamplesA + i][j] = pathB[i][j];
        }
    }   

    *planlength = numofsamples;
  }else{
    cout << "NOT FOUND" << endl;
  }
}



class RRTSTARtree{
public:
  int numofDOFs;
  Node* rootNode;
  Node* goalNode;
  vector<Node*> vertices;


  RRTSTARtree(int n){
    numofDOFs = n;
	}

  void insertNode(Node* nodeTemp){
    vertices.push_back(nodeTemp);
    vertices[vertices.size() - 1]->index = vertices.size() - 1;
  }

  int treeSize(){
    return vertices.size();
  }
  
  Node* Nearest_Neighbor(double* q){
    int idx = -1;
		double minDist = (double) DBL_MAX;
		for(int i = 0; i < vertices.size(); i++){
		  double tempDist = 0;
			for(int j = 0; j < numofDOFs; j++){
				tempDist += (vertices[i]->angle_rad[j] - q[j]) * (vertices[i]->angle_rad[j] - q[j]);
			}
		
			if(tempDist <= minDist)
			{
				minDist = tempDist;
				idx = i;
			}
		}
		return vertices[idx];
	}

  vector<int> Near(Node* q_rand){
    vector<int> near_idList;
    for (int i = 0; i < vertices.size(); i++){
      double tempDist = calc_distance(vertices[i]->angle_rad.data(), q_rand->angle_rad.data(), numofDOFs);
      if (tempDist < RI && tempDist != 0){
        near_idList.push_back(vertices[i]->index);
      }
    }
    return near_idList;
  }

  void Rewire(Node* q_rand, int numofDOFs, double* map, int x_size, int y_size){
    vector<int> potentials = Near(q_rand);
    
    double minCost = q_rand->cost;
    int minIdx = q_rand->parent->index;
    
    for(int i = 0; i < potentials.size(); i++){
      int tempIdx = potentials[i];
      // cout<< "tempIdx" << tempIdx << endl;
      if (Check_Collision(vertices[tempIdx]->angle_rad.data(), q_rand->angle_rad.data(), numofDOFs, map, x_size, y_size) ){
        double tempCost = vertices[tempIdx]->cost + calc_distance(vertices[tempIdx]->angle_rad.data(), q_rand->angle_rad.data(), numofDOFs);
        if (tempCost < minCost){
          minCost = tempCost;
          minIdx = tempIdx;
          // cout<< "rewire q_rand" << endl;
          
        }
      }
    }
    q_rand->parent = vertices[minIdx];
    q_rand->cost = minCost;

    for (int i = 0; i < potentials.size(); i++){
      
      int newIdx = potentials[i];
      if (Check_Collision(vertices[newIdx]->angle_rad.data(), q_rand->angle_rad.data(), numofDOFs, map, x_size, y_size) ){
        if(vertices[newIdx]->cost > minCost + calc_distance(vertices[newIdx]->angle_rad.data(), q_rand->angle_rad.data(), numofDOFs)){
          // cout<< "rewire around" << endl;
          vertices[newIdx]->cost = minCost + calc_distance(vertices[newIdx]->angle_rad.data(), q_rand->angle_rad.data(), numofDOFs);
          vertices[newIdx]->parent = q_rand;
        }
      }
      
    }
  }
};


static void plannerRRTSTAR(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{   
  auto timer_start = high_resolution_clock::now();
  *plan = NULL;
	*planlength = 0;

  srand (1);
  RRTSTARtree tree(numofDOFs);

  Node* q_start = new Node(numofDOFs, armstart_anglesV_rad, nullptr);
  tree.insertNode(q_start);
  q_start->cost = 0.0;
  tree.rootNode = q_start;
  
  double* rand_angle = new double[numofDOFs]; 
  bool pathFound = false;
  int count = 0;
  
  while (!pathFound && count <= MAXITR){
    // cout << "iter" << count << endl;
    if (count % 100 == 0){
      for(int i = 0; i < numofDOFs; i++){
        double temp = armgoal_anglesV_rad[i];
        rand_angle[i]= (double)temp;
      }     
    }else{
      Random_Config(map, x_size, y_size, numofDOFs, rand_angle);
    }

    count++;

    Node* q_near = tree.Nearest_Neighbor(rand_angle);
    // cout<< "q_near cost" << q_near->cost << endl;
    double* newConfig = Extend(q_near, rand_angle, numofDOFs, map, x_size, y_size);
    if (newConfig == nullptr){
      // cout<< "invalid" <<endl;
      continue;
    }
    else if(isEqual(newConfig, armgoal_anglesV_rad, numofDOFs)){
      if (pathFound){
        continue;
      }
      cout << "goal found" << endl;
      Node* q_goal = new Node(numofDOFs, newConfig, q_near);
      q_goal->cost = q_near->cost + calc_distance(q_near->angle_rad.data(), q_goal->angle_rad.data(), numofDOFs);
      tree.insertNode(q_goal);
      q_goal->parent = q_near;
      tree.goalNode = q_goal;

      pathFound = true;
      break;
    }
    else{
      Node* q_rand = new Node(numofDOFs, newConfig, q_near);
      tree.insertNode(q_rand);
      q_rand->cost = q_near->cost + calc_distance(q_near->angle_rad.data(), q_rand->angle_rad.data(), numofDOFs);
    
      tree.Rewire(q_rand, numofDOFs, map, x_size, y_size);
    }
    delete newConfig;
  }

  pathFound = true;
  if (pathFound){
    double cost = 0.0;
    vector<vector<double>> path;
    Node* q_path = tree.goalNode;
    while (q_path->parent != nullptr){
      double* prev = q_path->angle_rad.data();
      path.push_back(q_path->angle_rad);
      q_path = q_path->parent;
      cost += calc_distance(prev, q_path->angle_rad.data(), numofDOFs);
    }
    path.push_back(q_path->angle_rad);

    cout << "number of vertices " << tree.treeSize() << endl;
    cout << "cost of the tree " << cost << endl;
    int numofsamples = path.size();
    *plan = (double**) malloc(numofsamples * sizeof(double*));
    for (int i = 0 ; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs * sizeof(double)); 
        for(int j = 0; j < numofDOFs; j++){   
          (*plan)[i][j] = path[numofsamples - 1 - i][j];
        }
    }    
    *planlength = numofsamples;
  }else{
    cout << "path not found" << endl;
    int numofsamples = tree.treeSize();

    *plan = (double**) malloc(numofsamples*sizeof(double*));

    for (int i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(int j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = tree.vertices[i]->angle_rad[j];
        }

    }    
    *planlength = numofsamples;
  }
  
  return;
}


//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector of start angles for the arm 
//3nd is a row vector of goal angles for the arm 
//plhs should contain output parameters (2): 
//1st is a 2D matrix plan when each plan[i][j] is the value of jth angle at the ith step of the plan
//(there are D DoF of the arm (that is, D angles). So, j can take values from 0 to D-1
//2nd is planlength (int)
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 4) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Four input arguments required."); 
    } else if (nlhs != 2) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the start and goal angles*/     
    int numofDOFs = (int) (MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
    if(numofDOFs <= 1){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "it should be at least 2");         
    }
    double* armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
    if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN))){
        	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "numofDOFs in startangles is different from goalangles");         
    }
    double* armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);
 
    //get the planner id
    int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
    if(planner_id < 0 || planner_id > 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidplanner_id",
                "planner id should be between 0 and 3 inclusive");         
    }
    
    //call the planner
    double** plan = NULL;
    int planlength = 0;
    
    //you can may be call the corresponding planner function here
    // auto timer_start = high_resolution_clock::now();
    if (planner_id == RRT){      
      plannerRRT(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }else if (planner_id == RRTCONNECT){
      plannerRRTCONNECT(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }else if (planner_id == RRTSTAR){
      plannerRRTSTAR(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }else if (planner_id == PRM){
      plannerPRM(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }
    // auto timer_end = high_resolution_clock::now();
    // auto duration = duration_cast<milliseconds>(timer_end - timer_start);
    // cout<<"Planning done in: ";
    // cout<< duration.count() << " ms"<<endl;
    
    //dummy planner which only computes interpolated path
    // planner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength); 
    
    printf("planner returned plan of length=%d\n", planlength); 
    
    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
                plan_out[j] = armstart_anglesV_rad[j];
        }     
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;

    
    return;
    
}





