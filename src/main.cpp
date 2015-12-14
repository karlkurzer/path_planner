#include <ros/ros.h>

#include "dubins.h"
#include "node2d.h"
#include "node3d.h"


int main( int argc, char** argv )
{

    srand(time(NULL));
    Node3D start3D(0, 0, 0, 0, 0, nullptr);
    Node3D goal3D(0, 0, 0, 0, 0, nullptr);
    Node2D start2D(0, 0, 0, 0, nullptr);
    Node2D goal2D(0, 0, 0, 0, nullptr);

    // creation of motion primitives
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (i + j < 8)
            {
                dX[i][j] = dx[i + j];
                dY[i][j] = dy[i + j];
            }
            else {
                dX[i][j] = dx[i + j - 8];
                dY[i][j] = dy[i + j - 8];
            }
        }
    }

    for (int i = 0; i < 360; i++)
    {
        int factor = (int)i / 45 - 1;
        if (factor < 0)
            dT[i] = 7;
        else
            dT[i] = factor;
    }

    int selection;
    message("Approaching Hybrid A* Search\nA pathfinding algorithm on grids, by Karl Kurzer");
    while (true)
    {
        message("Select scenario type 33 to exit\nScenario 0: empty\nScenario 1: barrier\nScenario 2: maze\nScenario 9: random");
        cin >> selection;
        if (selection == 33) break;
        message("penalty (default = 1)");
        cin >> penalty;
        message("dubins (default = 1)");
        cin >> dubins;
        message("twoD (default = 1)");
        cin >> twoD;
        createScenario(start3D, goal3D, start2D, goal2D, selection);
        message("start searching the path");
        cout << "The length is " << aStar3D(start3D, goal3D);
        printGrid(start3D, goal3D);
    }
    return 0;
}
