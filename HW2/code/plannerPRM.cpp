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
#include <stack>
#include <utility>   
#include <cstdint>
#include <cstring>
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
#define MAXGRAPH 1000

// RRT parameters
#define EPSILON 0.1
#define MAXITR 100000


using namespace std;


class Node{
public:
  vector<double> angle_rad;
  int numofDOFs;
  Node* parent = nullptr;

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

class PRMap{
public:
  int numofDOFS;
  
  unordered_map<int, Node*> vertices;
  
  PRMap(int n){
    numofDOFS = n;
  }

  int AddVertex(double* sample_angelsV_rad)
	{
		int vid = vertices.size();
		vertices.push_back(sample_angelsV_rad);
		vector<int> tmp;
		edges.push_back(tmp);

		return vid;
	}


  void insertNode(int id, Node* newNode){
    vertices[id] = newNode;
  }
};

class PRMGraph{
public:
	PRMGraph(int num, double* armstart_anglesV_rad, double* armgoal_anglesV_rad)
	{
		numofDOFs = num;
		vertices.clear();
		edges.clear();
		g.clear();
		//bool(PRMGraph::* fn_pt)(int,int) = compareVertice;
		//set<int,bool(PRMGraph::*)(int,int)> CloseSet (fn_pt);

		int start_id = AddVertex(armstart_anglesV_rad);
		start_angles_rad = armstart_anglesV_rad;
		goal_angles_rad = armgoal_anglesV_rad;
	}

	int AddVertex(double* sample_angelsV_rad)
	{
		int vid = vertices.size();
		vertices.push_back(sample_angelsV_rad);
		vector<int> tmp;
		edges.push_back(tmp);

		return vid;
	}

	void AddEdge(int sid, int eid)
	{
		edges[sid].push_back(eid);
		edges[eid].push_back(sid);
	}

	int numofVertices()
	{
		return vertices.size();
	}

	double* GetVertice(int id)
	{
		return vertices[id];
	}

	vector<int> GetSuccessors(int id)
	{
		return edges[id];
	}


	void PreprocessR(double* map, double* armgoal_anglesV_rad,int N, double R, int k, int x_size, int y_size) //N is the number of sampling, k is the maximum number of connected neighbours.
	{
		
		int i = 0;
		int goal_id = AddVertex(armgoal_anglesV_rad);

		int num_edges = 0;

		while(i<N)
		{

			i++;
			int num_connected = 0;

			double* random_angles_rad =(double*)malloc(numofDOFs*sizeof(double));


			GenerateRandomConfig(random_angles_rad, map, numofDOFs, x_size, y_size);

			double* tmp_angles_rad = (double*)malloc(numofDOFs*sizeof(double));
			memcpy(tmp_angles_rad, random_angles_rad, numofDOFs*sizeof(double));

			int random_id = AddVertex(tmp_angles_rad);

			for(int j=0; j<vertices.size()-1; j++)
			{
				if(dist_angles(vertices[j],	tmp_angles_rad, numofDOFs)<R)
				{
					double* extend_angles_rad = Extend(vertices[j], tmp_angles_rad, map, numofDOFs, x_size, y_size, R);
					if(angles_equal(extend_angles_rad, tmp_angles_rad, numofDOFs))
					{
						AddEdge(j, random_id);
						num_connected++;
						num_edges++;
						if(num_connected>=k) break;
					}
				}
			}
		}	

		mexPrintf("Edges connected to goal: %d\n", edges[1].size());
		mexPrintf("Edges connected to start: %d\n", edges[0].size());
		mexPrintf("Number of edges: %d\n", num_edges);
	}

	unordered_map<int, double> g;
	//set<int> OpenSet;

private:
	vector<double* > vertices;
	vector<vector<int>> edges;
	int numofDOFs;
	double* start_angles_rad;
	double* goal_angles_rad;	
};

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

double* Extend(Node* startNode, double* endConfig, int numofDOFs, double* map, int x_size, int y_size){

  double* startConfig = new double[numofDOFs];
  for (int i = 0; i < numofDOFs; i++){
    startConfig[i] = startNode->angle_rad[i];
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
  if (!IsValidArmConfiguration(tempConfig, numofDOFs, map, x_size, y_size)){
    return nullptr;
  }else{
    return tempConfig;
  }
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
  srand (time(NULL)); //seed the time

  PRMGraph graph(numofDOFs, armstart_anglesV_rad, armgoal_anglesV_rad);
  //graph.Preprocess(map, armgoal_anglesV_rad, 10000, x_size, y_size);

  
  graph.PreprocessR(map, armgoal_anglesV_rad, 30000, Epsilon, 15, x_size, y_size);
  
  cout<< "done graph" << endl;
  return;
}

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
  cout << "tree size" << tree.treeSize() << endl;

  if (pathFound){
    vector<vector<double>> path;
    Node* q_path = tree.goalNode;
    while (q_path->parent != nullptr){
      path.push_back(q_path->angle_rad);
      q_path = q_path->parent;
    }
    path.push_back(q_path->angle_rad);

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
  }
  return;
}


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

}

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
    // bool advanced = false
    // if (newConfig == nullptr){
    //   // cout<< "invalid" <<endl;
    //   advanced = false;
    // }else{
    //   Node* q_new = new Node(numofDOFs, newConfig, q_near);
    //   tree.insertNode(q_new, tree.treeid);
    //   advanced = true;
    // }

    // tree.Swap();

    // bool connected = false;

    // if (advanced){
    //   Node* q_connect = tree.Nearest_Neighbor(rand_angle, tree.treeid);
    //   if (q_connect == nullptr){
    //     cout << "shouldnt happen" << endl;
    //   }
    //   connected = Connect(q_ne)
    // }
    // if (connected){
    //   pathFound = true;
    // }

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

  cout << "treeA size " << tree.treeSize(0) << endl;
  cout << "treeB size " << tree.treeSize(1) << endl;
  if (pathFound){
    cout << "FOUND PATH" << endl;
    vector<vector<double>> pathA;
    Node* q_pathA = tree.endNodeA;
    while (q_pathA->parent != nullptr){
      pathA.push_back(q_pathA->angle_rad);
      q_pathA = q_pathA->parent;
    }
    pathA.push_back(q_pathA->angle_rad);


    vector<vector<double>> pathB;
    Node* q_pathB = tree.endNodeB;
    while (q_pathB->parent != nullptr){
      pathB.push_back(q_pathB->angle_rad);
      q_pathB = q_pathB->parent;
    }
    pathB.push_back(q_pathB->angle_rad);

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

// static void planner(
// 		   double*	map,
// 		   int x_size,
//  		   int y_size,
//            double* armstart_anglesV_rad,
//            double* armgoal_anglesV_rad,
// 	   int numofDOFs,
// 	   double*** plan,
// 	   int* planlength)
// {
// 	//no plan by default
//     //for now just do straight interpolation between start and goal checking for the validity of samples

//   double distance = 0;
//   int i,j;
//   for (j = 0; j < numofDOFs; j++){
//       if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
//           distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
//   }
//   int numofsamples = (int)(distance/(PI/20));
//   if(numofsamples < 2){
//       printf("the arm is already at the goal\n");
//       return;
//   }
//   *plan = (double**) malloc(numofsamples*sizeof(double*));
//   int firstinvalidconf = 1;
//   for (i = 0; i < numofsamples; i++){
//       (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
//       for(j = 0; j < numofDOFs; j++){
//           (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
//       }
//       if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf)
//       {
//           firstinvalidconf = 1;
//           printf("ERROR: Invalid arm configuration!!!\n");
//       }
//   }    
//   *planlength = numofsamples;
  
//   return;
// }

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
    if (planner_id == RRT){
      plannerRRT(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }else if (planner_id == RRTCONNECT){
      plannerRRTCONNECT(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }else if (planner_id == RRTSTAR){
      plannerRRTSTAR(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }else if (planner_id == PRM){
      plannerPRM(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }
    
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





