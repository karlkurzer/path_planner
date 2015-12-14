#ifndef NODE3D_H
#define NODE3D_H

//###################################################
//                      (NOT YET HYBRID) A* ALGORITHM
//	AUTHOR:		Karl Kurzer
//	WRITTEN:	2015-11-11
//###################################################

#include <iostream>
#include <time.h>
#include <string>
//#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <queue>
#include <vector>
#include "dubins.h"
//#include "dubins.cpp"
//#include <boost/timer/timer.hpp>

#include "node2d.h"

using namespace std;

//###################################################
//                                  CONST DECLARATION
//###################################################
/*
HEADING => 0 - 359 degrees, 0 being north pointing towards negative X
X-COORDINATE => designating the length of the grid/world
Y-COORDINATE => designating the width of the grid/world
*/

// GRID SIZE
const int width = 60;
const int length = 30;
// MOTION ARRAYS
const int dir = 8;
const int dx[8] = { -1, -1, 0, 1, 1, 1, 0, -1 };
const int dy[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };
const int dt[8] = { 0, 45, 90, 135, 180, 225, 270, 315 };
int dX[8][3];
int dY[8][3];
int dT[360];
// GRID
bool grid[length][width];
// PATH TRACE
bool path[length][width];
int pathClosed[length][width];
// ALGORITHM LISTS
// 2D
bool listOpen2D[length][width];
bool listClosed2D[length][width];
float listCost2D[length][width];
float listCostToGo2D[length][width];
float listCostGoal2D[length][width];
int total2D = 0;
// 3D
bool listOpen3D[length][width][dir];
bool listClosed3D[length][width][dir];
float listCost3D[length][width][dir];
// USER CHOICE
bool penalty = true;
bool dubins = true;
bool twoD = true;

//###################################################
//                              COUT STANDARD message
//###################################################

void message(const string& msg)
{
    cout << "\n### " << msg << endl;
}

////###################################################
////                                 2D NODE COMPARISON
////###################################################

//struct Compare2DNodes : public binary_function<Node2D*, Node2D*, bool>
//{
//    bool operator()(const Node2D* lhs, const Node2D* rhs) const { return lhs->getC() > rhs->getC(); }
//};

//bool operator ==(const Node2D& lhs, const Node2D& rhs) { return lhs.getX() == rhs.getX() && lhs.getY() == rhs.getY(); }

//###################################################
//                                  OBSTACLE BLOATING
//###################################################

inline bool obstacleBloating(const int x, const int y)
{
    for (int i = 0; i < 8; i++)
    {
        if (x + dx[i] >= 0 && x + dx[i] < length && y + dy[i] >= 0 && y + dy[i] < width)
        {
            if (!grid[x + dx[i]][y + dy[i]]) return false;
        }
    }
    return true;
}

////###################################################
////                                 				2D A*
////###################################################

//float aStar2D(int xStart, int yStart, int xGoal, int yGoal)
//{
//    // create empty grid
//    for (int i = 0; i < length; i++)
//    {
//        for (int j = 0; j < width; j++)
//        {
//            listOpen2D[i][j] = false;
//            listClosed2D[i][j] = false;
//            listCost2D[i][j] = 0;
//        }
//    }
//    // PREDECESSOR AND SUCCESSOR POSITION
//    int x, y, xSucc, ySucc;
//    Node2D start(xStart, yStart, 0, 0, nullptr);
//    Node2D goal(xGoal, yGoal, 0, 0, nullptr);

//    // OPEN LIST
//    priority_queue<Node2D*, vector<Node2D*>, Compare2DNodes> O;
//    // update g value
//    start.updateG(start);
//    // update h value
//    start.updateH(goal);
//    // push on priority queue
//    O.push(&start);
//    // add node to open list with total estimated cost
//    listCost2D[start.getX()][start.getY()] = start.getC();

//    // continue until O empty
//    while (!O.empty())
//    {
//        // create new node pointer
//        Node2D* nPred;
//        // pop node with lowest cost from priority queue
//        nPred = O.top();
//        x = nPred->getX();
//        y = nPred->getY();
//        // lazy deletion of rewired node
//        if (listClosed2D[x][y] == true)
//        {
//            // remove node from open list
//            O.pop();
//            continue;
//        }
//        else if (listClosed2D[x][y] == false)
//        {
//            total2D++;
//            // remove node from open list
//            O.pop();
//            listOpen2D[x][y] = false;
//            // add node to closed list
//            listClosed2D[x][y] = true;

//            // goal test
//            if (*nPred == goal)
//            {
//                return nPred->getG();
//            }
//            // continue with search
//            else
//            {
//                // create positions of successor nodes
//                for (int i = 0; i < 8; i++)
//                {
//                    xSucc = x + dx[i];
//                    ySucc = y + dy[i];
//                    // ensure successor is on grid and not blocked by obstacle
//                    if (xSucc >= 0 && xSucc < length && ySucc >= 0 && ySucc < width && grid[xSucc][ySucc] && obstacleBloating(xSucc, ySucc))
//                    {
//                        // ensure successor is not on closed list
//                        if (listClosed2D[xSucc][ySucc] == false)
//                        {
//                            Node2D* nSucc;
//                            nSucc = new Node2D(xSucc, ySucc, nPred->getG(), 0, nullptr);
//                            // calculate new g value
//                            float newG = nPred->getG() + nSucc->movementCost(*nPred);
//                            // if successor not on open list or g value lower than before put it on open list
//                            if (listOpen2D[xSucc][ySucc] == false || newG < listCost2D[xSucc][ySucc])
//                            {
//                                // set predecessor of
//                                nSucc->setPred(nPred);
//                                nSucc->updateG(*nPred);
//                                listCost2D[xSucc][ySucc] = nSucc->getG();
//                                nSucc->updateH(goal);
//                                listCostToGo2D[xSucc][ySucc] = nSucc->getH();

//                                // put successor on open list
//                                listOpen2D[xSucc][ySucc] = true;
//                                O.push(nSucc);
//                            }
//                            else delete nSucc;
//                        }
//                    }
//                }
//            }
//        }
//    }
//    // return large number to guide search away
//    if (O.empty()) return 1000;
//}

//###################################################
//                                3D  NODE DEFINITION
//###################################################

class Node3D {
public:
    // CONSTRUCTOR
    Node3D(int x, int y, float t, float g, float h, Node3D* pred)
    {
        this->x = x;
        this->y = y;
        this->t = t;
        this->g = g;
        this->h = h;
        this->pred = pred;
    }
    // GETTER METHODS
    int getX() const { return x; }
    int getY() const { return y; }
    float getT() const { return t; }
    float getG() const { return g; }
    float getH() const { return h; }
    Node3D* getPred() const { return pred; }
    // returns total estimated cost for node
    float getC() const { return g + h; }

    // SETTER METHODS
    void setX(const int& x) { this->x = x; }
    void setY(const int& y) { this->y = y; }
    void setT(const float& t) { this->t = t; }
    void setG(const float& g) { this->g = g; }
    void setH(const float& h) { this->h = h; }
    void setPred(Node3D* pred) { this->pred = pred; }

    // UPDATE METHODS
    // from start
    void updateG(const Node3D& pred) { g += movementCost(pred); }
    // to goal
    void updateH(const Node3D& goal) { h = costToGo(goal); }
    // COST CALCULATION
    // cost for movement, g
    float movementCost(const Node3D& pred) const
    {
        float distance, tPenalty = 0;
        if (penalty)
        {
            //heading penalty
            if (abs(t - pred.getT()) > 180) tPenalty = (360 - abs(t - pred.getT())) / 45;
            else tPenalty = abs(t - pred.getT()) / 45;
        }
        // euclidean distance
        distance = sqrt((x - pred.x)*(x - pred.x) + (y - pred.y)*(y - pred.y));
        return distance + tPenalty;
    }
    // cost to go, dubins path / 2D A*
    float costToGo(const Node3D& goal) const
    {
        float cost = 0, dubinsLength = 0, euclidean = 0;
        int newT = 0, newGoalT = 0;
        if (dubins)
        {
            // theta conversion
            newT = (int)((360 - t) + 180) % 360;
            newGoalT = (int)((360 - goal.t) + 180) % 360;
            //start
            double q0[] = { x, y, newT / 180 * M_PI };
            // goal
            double q1[] = { goal.x, goal.y, newGoalT / 180 * M_PI };
            // turning radius
            float r = 1;
            DubinsPath path;
            dubins_init(q0, q1, r, &path);
            dubinsLength = dubins_path_length(&path);
        }
        if (twoD && listCostGoal2D[x][y] == 0)
        {
            Node2D start2d(x,y,0,0,nullptr);
            Node2D goal2d(goal.x,goal.y,0,0,nullptr);
            listCostGoal2D[x][y] = Node2D::aStar2D(start2d, goal2d);
        }
        euclidean = sqrt((x - goal.x)*(x - goal.x) + (y - goal.y)*(y - goal.y));
        return max(euclidean, max(dubinsLength, listCostGoal2D[x][y]));
    }

private:
    // x = position (length), y = position (width), t = heading, g = cost, h = cost to go, pred = pointer to predecessor node
    int x;
    int y;
    float t;
    float g;
    float h;
    Node3D* pred;
};

//###################################################
//                                         TRACE PATH
//###################################################

void tracePath(Node3D* node, int count = 0)
{
    if (node == nullptr) return;
    path[node->getX()][node->getY()] = true;
    tracePath(node->getPred(), count);
}

//###################################################
//                                 3D NODE COMPARISON
//###################################################

struct CompareNodes : public binary_function<Node3D*, Node3D*, bool>
{
    bool operator()(const Node3D* lhs, const Node3D* rhs) const
    {

        return lhs->getC() > rhs->getC();
    }
};

bool operator ==(const Node3D& lhs, const Node3D& rhs) { return lhs.getX() == rhs.getX() && lhs.getY() == rhs.getY() && lhs.getT() == rhs.getT(); }

//###################################################
//                                              3D A*
//###################################################

float aStar3D(Node3D& start, const Node3D& goal)
{
    //boost::timer::auto_cpu_timer ctimer;
    int x, y, t, xSucc, ySucc, tSucc;

    // open list implemented as priority queue
    priority_queue<Node3D*, vector<Node3D*>, CompareNodes> O;
    // update the start node g value
    start.updateG(start);
    // update the start node h value
    start.updateH(goal);

    // push it on the priority queue
    O.push(&start);

    // add node to the open list with the total estimated costs
    listCost3D[start.getX()][start.getY()][dT[(int)start.getT()]] = start.getG();

    while (!O.empty())
    {
        // get the node with the lowest cost from the priority queue
        // x, y, t, g, h, parent
        Node3D* nPred;
        nPred = O.top();
        x = nPred->getX();
        y = nPred->getY();
        t = nPred->getT();
        // lazy deletion
        if (listClosed3D[x][y][dT[(int)t]] == true)
        {
            O.pop();
            //cout << "lazy deletion took place" << endl;
            continue;
        }
        else if (listClosed3D[x][y][dT[(int)t]] == false)
        {
            // remove node from the open list
            O.pop();
            listOpen3D[x][y][dT[(int)t]] = false;
            // add node to the closed list
            listClosed3D[x][y][dT[(int)t]] = true;

            // goal test
            if (*nPred == goal)
            {
                message("path has been found");
                tracePath(nPred);
                return nPred->getG();
            }
            // search further
            else
            {
                // determine index of motion primitive
                int j = dT[t];
                // create new positions for possible child nodes
                for (int i = 0; i < 3; i++)
                {

                    // create new position for child node
                    xSucc = x + dX[j][i];
                    ySucc = y + dY[j][i];
                    if (i == 0) tSucc = t - 45;
                    else if (i == 1) tSucc = t ;
                    else if (i == 2) tSucc = t + 45;
                    //tSucc = t + i * 45 - 45;
                    if (tSucc > 359) tSucc = tSucc % 360;
                    else if (tSucc < 0) tSucc = 360 + tSucc;

                    // ensure that the successor is in the bounds of the grid and not on the closed list and not blocked by an obstacle
                    if (xSucc >= 0 && xSucc < length && ySucc >= 0 && ySucc < width && grid[xSucc][ySucc] && obstacleBloating(xSucc, ySucc))
                    {
                        if (listClosed3D[xSucc][ySucc][dT[(int)tSucc]] == false)
                        {
                            Node3D* nSucc;
                            nSucc = new Node3D(xSucc, ySucc, tSucc, nPred->getG(), 0, nullptr);
                            // calculate new g value
                            float newG = nPred->getG() + nSucc->movementCost(*nPred);
                            // put the successor node on the open list
                            if (listOpen3D[xSucc][ySucc][dT[(int)tSucc]] == false || newG < listCost3D[xSucc][ySucc][dT[(int)tSucc]])
                            {
                                nSucc->setPred(nPred);
                                nSucc->updateG(*nPred);
                                listCost3D[xSucc][ySucc][dT[(int)tSucc]] = nSucc->getG();
                                nSucc->updateH(goal);

                                if (listOpen3D[xSucc][ySucc][dT[(int)tSucc]] == false)
                                {
                                    //lazy deletion;
                                    listOpen3D[xSucc][ySucc][dT[(int)tSucc]] = true;
                                    O.push(nSucc);
                                }
                                else {
                                    //lazy deletion;
                                    O.push(nSucc);
                                }
                            }
                            else
                            {
                                delete nSucc;
                            }
                        }
                    }
                }
            }
        }
    }
    if (O.empty())
    {
        message("path has not been found");
        return 0;
    }
}

//###################################################
//                            TRAVERSABILITY CHECKING
//###################################################

inline bool traversable(const int x, const int y, const Node3D& node)
{
    int xSucc, ySucc;

    for (int i = 0; i < dir; i++)
    {
        xSucc = x + dx[i];
        ySucc = y + dy[i];
        if (xSucc == node.getX() && ySucc == node.getY())
        {
            return false;
        }
    }
    return true;
}

//###################################################
//                                         PRINT GRID
//###################################################

void printGrid(const Node3D& start, const Node3D& goal)
{
    int total3D = 0;
    for (int i = 0; i < length; i++)
    {
        for (int j = 0; j < width; j++)
        {
            int count3D = 0;
            for (int k = 0; k < dir; k++)
            {
                if (listClosed3D[i][j][k]) count3D++;
            }
            total3D += count3D;
            pathClosed[i][j] = count3D;
        }
    }
    message("The grid");
    cout << total3D << " 3D nodes have been expanded" << endl;
    cout << total2D << " 2D nodes have been expanded" << endl;
    for (int i = 0; i < length; i++)
    {
        cout << endl;
        for (int j = 0; j < width; j++)
        {
            if (goal.getX() == i && goal.getY() == j)
                cout << "G";
            else if (start.getX() == i && start.getY() == j)
                cout << "S";
            else if (path[i][j])
                cout << "+";
            else if (pathClosed[i][j] > 0)
                cout << "|";
            else if (!grid[i][j])
                cout << "#";
            else if (grid[i][j])
                cout << ".";
        }
    }
}

//###################################################
//                                          SCENARIOS
//###################################################

void createScenario(Node3D& start3D, Node3D& goal3D, Node2D& start2D, Node2D& goal2D, int scenario) {

    int xStart, yStart, xGoal, yGoal;

    // create empty grid
    for (int i = 0; i < length; i++)
    {
        for (int j = 0; j < width; j++)
        {
            grid[i][j] = true;
            path[i][j] = false;
            listOpen2D[i][j] = false;
            listClosed2D[i][j] = false;
            listCost2D[i][j] = 0;
            listCostToGo2D[i][j] = 0;
            listCostGoal2D[i][j] = 0;
            total2D = 0;
            for (int k = 0; k < dir; k++)
            {
                listOpen3D[i][j][k] = false;
                listClosed3D[i][j][k] = false;
                listCost3D[i][j][k] = 0;
            }
        }
    }

    switch (scenario)
    {
    //
    case 0:
        start3D.setX(5);
        start3D.setY(width - 1);
        goal3D.setX(5);
        goal3D.setY(width / 2);
        goal3D.setT(0);

        goal2D.setX(5);
        goal2D.setY(width / 2);

        break;
    case 1:
        start3D.setX(length - 4);
        start3D.setY(width / 2);
        goal3D.setX(5);
        goal3D.setY(width / 2);
        goal3D.setT(0);

        goal2D.setX(5);
        goal2D.setY(width / 2);

        // horizontal
        for (int j = width / 4; j < width * 3 / 4; j++)
            grid[length / 2][j] = false;
        // vertical
        for (int j = length / 2; j < length * 3 / 4; j++)
            grid[j][width / 4] = false;
        for (int k = length / 2; k < length * 3 / 4; k++)
            grid[k][width * 3 / 4] = false;
        break;
        // straigth
    case 4:
        start3D.setX(length - 4);
        start3D.setY(width / 2);
        goal3D.setX(5);
        goal3D.setY(width / 2);
        goal3D.setT(0);

        goal2D.setX(5);
        goal2D.setY(width / 2);

        break;
        // straigth
    case 5:
        start3D.setX(length / 2);
        start3D.setY(width*3/4);
        start3D.setT(270);
        goal3D.setX(length / 2);
        goal3D.setY(width / 4);
        goal3D.setT(270);

        goal2D.setX(length/2);
        goal2D.setY(width / 4);

        break;
    case 2:
        start3D.setX(2);
        start3D.setY(2);
        goal3D.setX(5);
        goal3D.setY(56);

        goal2D.setX(5);
        goal2D.setY(56);
        if (width == 60 && length == 30)
        {
            for (int i = 0; i < 7; ++i)
                grid[i][7] = false;
            for (int i = 12; i < 18; ++i)
                grid[4][i] = false;
            for (int i = 4; i < 13; ++i)
                grid[i][18] = false;
            for (int i = 0; i < 22; ++i)
                grid[i][34] = false;
            for (int i = 4; i < 11; ++i)
                grid[i][51] = false;
            for (int i = 51; i < 59; ++i)
                grid[11][i] = false;
        }
        else message("Expected the width to be 60 and the length to be 30");
        break;

        //random szenario
    default:
        // start node
        xStart = rand() % (length - 1);
        yStart = rand() % (width - 1);
        start3D.setX(xStart);
        start3D.setY(yStart);

        // goal node
        xGoal = rand() % (length - 1);
        yGoal = rand() % (width - 1);
        while (abs(xStart - xGoal) < length / 3)
        {
            xGoal = rand() % (length - 1);
        }
        goal3D.setX(xGoal);
        goal3D.setY(yGoal);

        goal2D.setX(xGoal);
        goal2D.setY(yGoal);

        // create obstacles on the grid
        for (int i = 0; i < length; i++)
        {
            for (int j = 0; j < width; j++)
            {
                int random = rand() % 100;
                // create random obstacles, obstacles are more likely to appear together
                if (random < 1 && traversable(i, j, start3D) && traversable(i, j, goal3D))
                    grid[i][j] = false;
                else if (random < 80 && j < width && j != 0 && grid[i][j - 1] == false && traversable(i, j, start3D) && traversable(i, j, goal3D))
                    grid[i][j] = false;
            }
        }
        break;
    }
    message("created grid"); cout << length << " by " << width <<endl;
    cout << "The heading for the start state is " << start2D.getH() << " degrees north" <<endl;
    cout << "The heading for the goal state is " << goal2D.getH() << " degrees north" <<endl;
    cout << "The turning radius of the vehicle is " << 1.5 << " cells" << endl;

}

#endif // NODE3D_H
