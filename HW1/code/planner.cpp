/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <mex.h>

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
#include <utility>      // std::pair, std::make_pair

/* Input Arguments */
#define    MAP_IN                  prhs[0]
#define    ROBOT_IN                prhs[1]
#define    TARGET_TRAJ             prhs[2]
#define    TARGET_POS              prhs[3]
#define    CURR_TIME               prhs[4]
#define    COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define    ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define    MAX(A, B)    ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define    MIN(A, B)    ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8



using namespace std;
using namespace std::chrono;


typedef pair<int, int> Pair;
stack<Pair, vector<Pair>> path;
bool pathFound = false;

// Node class 
class Node {
public:
    double fVal = (double) DBL_MAX;
    double gVal = (double) DBL_MAX;
    double hVal = (double) DBL_MAX;
    int tVal = -1;
    Node *parent = nullptr;
    vector<int> pose = {-1, -1};

    Node(){}
    
    Node(double f, double g, double h, int t, vector<int> p, Node* pa){
        fVal = f;
        gVal = g;
        hVal = h;
        tVal = t;
        pose = p;
        parent = pa;
    }
};


class Compare_Node {
public:
    bool operator()(Node *a, Node *b) {
        return(a->fVal > b->fVal);
    }
};

class Compare_hVal{
public:
    bool operator()(Node* a, Node* b){
        return (a->hVal > b->hVal);
    }
};

int calc_index(int x, int y, int x_size, int y_size){
    return x * y_size + y;
}

long long int calc_3dindex(int x, int y, int t, int x_size, int y_size, int t_size){
    return (x * t_size * y_size + y * t_size + t);
}

// double calc_hVal(int currentX, int currentY, int targetX, int targetY){
// // Euclidean distance
//     // return (double)sqrt(((currentX - targetX) * (currentX - targetX) + (currentY - targetY) * (currentY - targetY)));

// // Manhattan distance
//     return (double) (abs(currentX - targetX) + abs(currentY - targetY));
// }

// check if 2d matrix is within the map and not in obstacle
bool is_Valid(double* map, int x_size, int y_size, int pointX, int pointY, int collision_thresh){
    if (pointX >= 1 && pointX <= x_size && pointY >= 1 && pointY <= y_size)
    {
        if (((int)map[GETMAPINDEX(pointX, pointY, x_size, y_size)] >= 0) 
        && ((int)map[GETMAPINDEX(pointX, pointY, x_size, y_size)] < collision_thresh))  //if free
        {
            return true;
        }
    }
    return false;
}

// check if 3d matrix is within the map and not in obstacle
bool is_Valid3d(double* map, int x_size, int y_size, int t_size, int pointX, int pointY, int pointt, int collision_thresh){
    if (pointX >= 1 && pointX <= x_size && pointY >= 1 && pointY <= y_size && pointt >= 0 && pointt <= t_size)
    {
        if (((int)map[GETMAPINDEX(pointX, pointY, x_size, y_size)] >= 0) 
        && ((int)map[GETMAPINDEX(pointX, pointY, x_size, y_size)] < collision_thresh))  //if free
        {
            return true;
        }
    }
    return false;
}

// run dijkstra to generate 2d heuristic
unordered_map<int, Node*> dijkstra(
    double *map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    double *target_traj,
    int curr_time
){

    int dX[NUMOFDIRS] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dY[NUMOFDIRS] = {-1, 0, 1, -1, 1, -1, 0, 1};

    unordered_map<int, Node*> hVal_list;

    priority_queue < Node* , vector< Node*>, Compare_hVal > openList;
    unordered_set <int> closeList;
    

    int starti = target_steps/2;

    for (int i = starti; i < target_steps; i++){
        Node* goalNode = new Node();
        goalNode->pose = {(int)target_traj[i], (int)target_traj[i + target_steps]};
        goalNode->hVal = 0.0;
        int index_goal = calc_index(goalNode->pose[0], goalNode->pose[1], x_size, y_size);
        hVal_list.insert({index_goal, goalNode});
        openList.push(goalNode);
    }

    // Node* goalNode = new Node();
    // goalNode->pose = {(int) target_traj[(curr_time + 1) - 1], (int) target_traj[(curr_time + 1) + target_steps - 1]};
    // goalNode->hVal = 0;
    // int index_goal = calc_index(goalNode->pose[0], goalNode->pose[1], x_size, y_size);
    // hVal_list.insert({index_goal, goalNode});
    // openList.push(goalNode);

    while (!openList.empty()) {
        Node* currentNode = openList.top();
        openList.pop();


        int index_current = calc_index(currentNode->pose[0], currentNode->pose[1], x_size, y_size);
        closeList.insert(index_current);
        
        for (int i = 0; i < NUMOFDIRS; i++) {
            int newX = currentNode->pose[0] + dX[i];
            int newY = currentNode->pose[1] + dY[i];
            int index_new = calc_index(newX, newY, x_size, y_size);

            if (!is_Valid(map, x_size, y_size, newX, newY, collision_thresh) ||
                closeList.find(index_new) != closeList.end()) {
                continue;
            }

            double hVal_new = currentNode->hVal + (int)map[GETMAPINDEX(newX, newY, x_size, y_size)];
            
            if (hVal_list.find(index_new) == hVal_list.end()){
                Node* newNode = new Node(); 
                newNode->pose = {newX, newY};
                newNode->hVal = hVal_new;
                openList.push(newNode);
                hVal_list[index_new] = newNode;
            }
        }
    }
    
    // int idx = calc_index(robotposeX, robotposeY, x_size, y_size);
    

    // if (hVal_list[idx] == nullptr){
    //     cout<< "wrong";
    // }else{
    //     cout<< "robot pose hval";
    //     cout<< hVal_list[idx]->hVal <<endl;
    // }
    
    return hVal_list;
}


bool findPath(int x, int y, int t, int x_size, int y_size, int t_size, double* target_traj, int target_steps){
    long long int currentidx = calc_3dindex(x, y, t, x_size, y_size, t_size);
    long long int goalidx = calc_3dindex((int)target_traj[t], (int)target_traj[t + target_steps], t, x_size, y_size, t_size);
    if (goalidx == currentidx){
        cout<<"found";
        cout<< t;
        cout<<" ";
        cout<<target_traj[t];
        cout<<" ";
        cout<<target_traj[t + target_steps]<<endl;
    }
    return (goalidx == currentidx);
}


Pair generatePath(unordered_map<long long int, Node*> NodeMap, int x, int y, int t, int x_size, int y_size, int t_size, Node* startNode){
    int index_next = calc_3dindex(x, y, t, x_size, y_size, t_size);
    
    Node* nextNode = NodeMap[index_next];
    
    path.push(make_pair(x, y));

    while (nextNode->parent != startNode) {
        nextNode = nextNode->parent;
        path.push(make_pair(nextNode->pose[0], nextNode->pose[1]));
    }
    // path.push(make_pair(nextNode->pose[0], nextNode->pose[1]));
    pathFound = true;
    Pair action = path.top();
    return action;

}

static void planner(
        double *map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double *target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double *action_ptr
) {
    
    if (pathFound){
        cout<< "robot at";
        cout<< robotposeX << " "<< robotposeY <<endl;
        cout<< "target at";
        cout<<targetposeX << " " << targetposeY<<endl;
        cout<< "         "<<endl;
        
        if(!path.empty())
        {
            Pair next = path.top();
            action_ptr[0] = next.first;
            action_ptr[1] = next.second;
            path.pop();
            return; 
        }
        else
        {
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            return;
        }

    }

    auto timer_start = high_resolution_clock::now();
    unordered_map<int, Node*> h_cost = dijkstra(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, curr_time);
    
    // 9-connected grid
    int dX[NUMOFDIRS + 1] = {0, -1, -1, -1, 0, 0, 1, 1, 1};
    int dY[NUMOFDIRS + 1] = {0, -1, 0, 1, -1, 1, -1, 0, 1};
    
    cout<< target_steps << endl;

    int t_size = target_steps;
    int time = 20;
    if (x_size > 1700){
        time = 40;
    }
    if (target_steps <  20){
        time = 0;
    }
    int w = collision_thresh * 30;
   
    unordered_map<long long int, Node*> NodeMap;

    priority_queue < Node*, vector< Node*>, Compare_Node > openList;
    unordered_set <long long int> closeList;

    Node *startNode = new Node();
    startNode->tVal = time;
    startNode->hVal = 0.0;
    startNode->gVal = 0.0;
    startNode->fVal = 0.0;
    startNode->pose = {robotposeX, robotposeY};
    long long int index_start = calc_3dindex(robotposeX, robotposeY, time, x_size, y_size, t_size);
    NodeMap.insert({index_start, startNode});

    openList.push(startNode);  

    while (!openList.empty()) {
        Node *currentNode = openList.top();

        openList.pop();
        // cout<< "currentNode is";
        // cout<< currentNode->pose[0];
        // cout<< " ";
        // cout<< currentNode->pose[1]<<endl;
        

        long long int index_current = calc_3dindex(currentNode->pose[0], currentNode->pose[1], currentNode->tVal, x_size, y_size, t_size);
        closeList.insert(index_current);
        
        for (int i = 0; i < NUMOFDIRS + 1; i++) {
            int newX = currentNode->pose[0] + dX[i];
            int newY = currentNode->pose[1] + dY[i];
            int newt = currentNode->tVal + 1;
            long long int index_new3d = calc_3dindex(newX, newY, newt, x_size, y_size, t_size);
            int index_new2d = calc_index(newX, newY, x_size, y_size);

            if (!is_Valid3d(map, x_size, y_size, t_size, newX, newY, newt, collision_thresh) ||
                closeList.find(index_new3d) != closeList.end()) {
                continue;
            }
            
            // If path found, generate a stack of path
            if (findPath(newX, newY, newt, x_size, y_size, t_size, target_traj, target_steps) 
            && newX >= 1 && newX <= x_size && newY >= 1 && newY <= y_size && newt < t_size) {
                auto timer_end = high_resolution_clock::now();
                auto duration = duration_cast<milliseconds>(timer_end - timer_start);
                cout<<"Planning done in: ";
                cout<< duration.count()<<endl;
                cout<< "goal at";
                cout<< newX;
                cout<< " ";
                cout<< newY;
                cout<< " ";
                cout<< newt<<endl;

                Node* nextNode = new Node();
                nextNode->pose = {newX, newY};
                nextNode->tVal = newt;
                nextNode->parent = currentNode;
                NodeMap.insert({index_new3d, nextNode});
                // path.push(make_pair(newX, newY));
                Pair nextStep = generatePath(NodeMap, newX, newY, newt, x_size, y_size, t_size, startNode);                
                action_ptr[0] = nextStep.first;
                action_ptr[1] = nextStep.second;

                for (auto it = NodeMap.begin(); it != NodeMap.end(); ++it){
                    if (it->second != nullptr){
                        free(it->second);
                    }
                }
                for (auto it = h_cost.begin(); it != h_cost.end(); ++it){
                    if (it->second != nullptr){
                        free(it->second);
                    }
                }
                NodeMap.clear();
                h_cost.clear();
                return;
            }

            

            double gVal_new = currentNode->gVal + (int)map[GETMAPINDEX(newX, newY, x_size, y_size)];
            double hVal_new = h_cost[index_new2d]->hVal;
            double fVal_new = gVal_new + w * hVal_new;

            
            if (NodeMap.find(index_new3d) == NodeMap.end()){
                Node* newNode = new Node();
                newNode->pose = {newX, newY};
                newNode->parent = currentNode;
                newNode->gVal = gVal_new;
                newNode->hVal = hVal_new;
                newNode->fVal = fVal_new;
                newNode->tVal = newt;
                openList.push(newNode);
                NodeMap.insert({index_new3d, newNode});
            }else if(NodeMap.find(index_new3d) != NodeMap.end() && NodeMap[index_new3d]->fVal > fVal_new){
                NodeMap[index_new3d]->parent = currentNode;
                NodeMap[index_new3d]->gVal = gVal_new;
                NodeMap[index_new3d]->hVal = hVal_new;
                NodeMap[index_new3d]->fVal = fVal_new;
                NodeMap[index_new3d]->tVal = newt;
                NodeMap[index_new3d]->pose = {newX, newY};
            }
            
        }
    }

    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row std::vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row std::vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]) {

    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidNumInputs",
                          "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt("MATLAB:planner:maxlhs",
                          "One output argument required.");
    }

    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double *map = mxGetPr(MAP_IN);

    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if (robotpose_M != 1 || robotpose_N != 2) {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidrobotpose",
                          "robotpose std::vector should be 1 by 2.");
    }
    double *robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int) robotposeV[0];
    int robotposeY = (int) robotposeV[1];

    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);

    if (targettraj_M < 1 || targettraj_N != 2) {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidtargettraj",
                          "targettraj std::vector should be M by 2.");
    }
    double *targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;

    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if (targetpose_M != 1 || targetpose_N != 2) {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidtargetpose",
                          "targetpose std::vector should be 1 by 2.");
    }
    double *targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int) targetposeV[0];
    int targetposeY = (int) targetposeV[1];

    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);

    /* Create a matrix for the return action */
    ACTION_OUT = mxCreateNumericMatrix((mwSize) 1, (mwSize) 2, mxDOUBLE_CLASS, mxREAL);
    double *action_ptr = (double *) mxGetData(ACTION_OUT);

    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);

    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX,
            targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;
}