#include <functional>
#include <queue>
#include <vector>

#include "node2d.h"

// CONSTANT VALUES
// possible directions
const int Node2D::dir = 8;
// possible movements
const int Node2D::dx[] = { -1, -1, 0, 1, 1, 1, 0, -1 };
const int Node2D::dy[] = { 0, 1, 1, 1, 0, -1, -1, -1 };

//###################################################
//                                 2D NODE COMPARISON
//###################################################
struct Compare2DNodes : public std::binary_function<Node2D*, Node2D*, bool> {
    bool operator()(const Node2D* lhs, const Node2D* rhs) const {
        return lhs->getC() > rhs->getC();
    }
};

bool operator ==(const Node2D& lhs, const Node2D& rhs) {
    return lhs.getX() == rhs.getX() && lhs.getY() == rhs.getY();
}

//###################################################
//                                 				2D A*
//###################################################
float Node2D::aStar(Node2D& start, Node2D& goal, const nav_msgs::OccupancyGrid::ConstPtr& grid) {
    // LISTS dynamically allocated ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;
    int length = width * height;
    int idx = 0;
    int idxSucc = 0;
    //    grid = new int [length];
    open = new bool [length];
    closed = new bool [length];
    cost = new float [length];
    costToGo = new float [length];

    // initialize all lists
    for (int i = 0; i < length; ++i) {
        open[i] = false;
        closed[i] = false;
        cost[i] = 0;
        costToGo[i] = 0;
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
    cost[start.getY() * width + start.getX()] = start.getC();

    // continue until O empty
    while (!O.empty()) {
        // create new node pointer
        Node2D* nPred;
        // pop node with lowest cost from priority queue
        nPred = O.top();
        x = nPred->getX();
        y = nPred->getY();
        idx = y * width + x;

        // lazy deletion of rewired node
        if (closed[idx] == true) {
            // remove node from open list
            O.pop();
            continue;
        } else if (closed[idx] == false) {
            // remove node from open list
            O.pop();
            open[idx] = false;
            // add node to closed list
            closed[idx] = true;

            // goal test
            if (*nPred == goal) {
                delete[] open;
                delete[] closed;
                delete[] cost;
                delete[] costToGo;
                return nPred->getG();
            }
            // continue with search
            else {
                // create positions of successor nodes
                for (int i = 0; i < Node2D::dir; i++) {
                    xSucc = x + Node2D::dx[i];
                    ySucc = y + Node2D::dy[i];
                    idxSucc = ySucc * width + xSucc;

                    // ensure successor is on grid ROW MAJOR
                    if (xSucc >= 0 && xSucc < width && ySucc >= 0 && ySucc < length) {
                        // ensure successor is not blocked by obstacle
                        if (grid[idxSucc] == 0) {
                            // ensure successor is not on closed list
                            if (closed[idxSucc] == false) {
                                Node2D* nSucc;
                                nSucc = new Node2D(xSucc, ySucc, nPred->getG(), 0, nullptr);
                                // calculate new g value
                                float newG = nPred->getG() + nSucc->movementCost(*nPred);

                                // if successor not on open list or g value lower than before put it on open list
                                if (open[idxSucc] == false || newG < cost[idxSucc]) {
                                    // set predecessor
                                    nSucc->setPred(nPred);
                                    nSucc->updateG(*nPred);
                                    cost[idxSucc] = nSucc->getG();
                                    nSucc->updateH(goal);
                                    costToGo[idxSucc] = nSucc->getH();
                                    // put successor on open list
                                    open[idxSucc] = true;
                                    O.push(nSucc);
                                } else { delete nSucc; }
                            }
                        }
                    }
                }
            }
        }
    }

    // return large number to guide search away
    if (O.empty()) {
        delete[] open;
        delete[] closed;
        delete[] cost;
        delete[] costToGo;
        return 1000;
    }
}
