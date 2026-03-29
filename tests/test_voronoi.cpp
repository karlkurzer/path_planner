#include "../include/dynamicvoronoi.h"
#include <iostream>

using namespace HybridAStar;

int main() {
    DynamicVoronoi voronoi;

    // Create a 10x10 map
    int width = 10;
    int height = 10;
    bool** map1 = new bool*[width];
    for (int i = 0; i < width; ++i) {
        map1[i] = new bool[height]();
    }

    // Initialize map
    std::cout << "Initializing 10x10 map..." << std::endl;
    voronoi.initializeMap(width, height, map1);

    // Create a 20x20 map
    int new_width = 20;
    int new_height = 20;
    bool** map2 = new bool*[new_width];
    for (int i = 0; i < new_width; ++i) {
        map2[i] = new bool[new_height]();
    }

    // Re-initialize map (which should free previous map resources or cause bug with current code)
    std::cout << "Re-initializing with 20x20 map..." << std::endl;
    voronoi.initializeMap(new_width, new_height, map2);

    std::cout << "Done!" << std::endl;
    return 0;
}
