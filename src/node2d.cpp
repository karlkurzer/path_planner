#include <functional>
#include <queue>
#include <vector>
#include "node2d.h"

//###################################################
//                                 2D NODE COMPARISON
//###################################################
struct Compare2DNodes : public std::binary_function<Node2D*, Node2D*, bool>
{
    bool operator()(const Node2D* lhs, const Node2D* rhs) const { return lhs->getC() > rhs->getC(); }
};

bool operator ==(const Node2D& lhs, const Node2D& rhs) { return lhs.getX() == rhs.getX() && lhs.getY() == rhs.getY(); }

//###################################################
//                                 				2D A*
//###################################################
float Node2D::aStar2D(Node2D& start, Node2D& goal)
{
    std::vector<int> first;
    // create empty grid
    for (int i = 0; i < length; i++)
    {
        for (int j = 0; j < width; j++)
        {
            listOpen2D[i][j] = false;
            listClosed2D[i][j] = false;
            listCost2D[i][j] = 0;
        }
    }
    // PREDECESSOR AND SUCCESSOR POSITION
    int x, y, xSucc, ySucc;

    // OPEN LIST
    std::priority_queue<Node2D*, vector<Node2D*>, Compare2DNodes> O;
    // update g value
    start.updateG(start);
    // update h value
    start.updateH(goal);
    // push on priority queue
    O.push(&start);
    // add node to open list with total estimated cost
    listCost2D[start.getX()][start.getY()] = start.getC();

    // continue until O empty
    while (!O.empty())
    {
        // create new node pointer
        Node2D* nPred;
        // pop node with lowest cost from priority queue
        nPred = O.top();
        x = nPred->getX();
        y = nPred->getY();
        // lazy deletion of rewired node
        if (listClosed2D[x][y] == true)
        {
            // remove node from open list
            O.pop();
            continue;
        }
        else if (listClosed2D[x][y] == false)
        {
            total2D++;
            // remove node from open list
            O.pop();
            listOpen2D[x][y] = false;
            // add node to closed list
            listClosed2D[x][y] = true;

            // goal test
            if (*nPred == goal)
            {
                return nPred->getG();
            }
            // continue with search
            else
            {
                // create positions of successor nodes
                for (int i = 0; i < 8; i++)
                {
                    xSucc = x + dx[i];
                    ySucc = y + dy[i];
                    // ensure successor is on grid and not blocked by obstacle
                    if (xSucc >= 0 && xSucc < length && ySucc >= 0 && ySucc < width && grid[xSucc][ySucc] && obstacleBloating(xSucc, ySucc))
                    {
                        // ensure successor is not on closed list
                        if (listClosed2D[xSucc][ySucc] == false)
                        {
                            Node2D* nSucc;
                            nSucc = new Node2D(xSucc, ySucc, nPred->getG(), 0, nullptr);
                            // calculate new g value
                            float newG = nPred->getG() + nSucc->movementCost(*nPred);
                            // if successor not on open list or g value lower than before put it on open list
                            if (listOpen2D[xSucc][ySucc] == false || newG < listCost2D[xSucc][ySucc])
                            {
                                // set predecessor of
                                nSucc->setPred(nPred);
                                nSucc->updateG(*nPred);
                                listCost2D[xSucc][ySucc] = nSucc->getG();
                                nSucc->updateH(goal);
                                listCostToGo2D[xSucc][ySucc] = nSucc->getH();

                                // put successor on open list
                                listOpen2D[xSucc][ySucc] = true;
                                O.push(nSucc);
                            }
                            else delete nSucc;
                        }
                    }
                }
            }
        }
    }
    // return large number to guide search away
    if (O.empty()) return 1000;
}
