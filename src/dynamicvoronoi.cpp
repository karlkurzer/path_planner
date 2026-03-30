#include "dynamicvoronoi.h"

#include <math.h>
#include <iostream>

using namespace HybridAStar;

DynamicVoronoi::DynamicVoronoi() {
  sqrt2 = sqrt(2.0);
  data = NULL;
  gridMap = NULL;
  sizeX = 0;
  sizeY = 0;
}

DynamicVoronoi::~DynamicVoronoi() {
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
    data = NULL;
  }
}

void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY, bool initGridMap) {
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }

  sizeX = _sizeX;
  sizeY = _sizeY;

  data = new dataCell*[sizeX];
  for (int x=0; x<sizeX; x++) data[x] = new dataCell[sizeY];

  dataCell c;
  c.dist = INFINITY;
  c.sqdist = INT_MAX;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.voronoi = free;
  c.queueing = fwNotQueued;
  c.needsRaise = false;

  for (int x=0; x<sizeX; x++)
    for (int y=0; y<sizeY; y++) data[x][y] = c;

}

void DynamicVoronoi::initializeMap(int _sizeX, int _sizeY, bool** _gridMap) {
  gridMap = _gridMap;
  initializeEmpty(_sizeX, _sizeY, false);

  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {
      if (gridMap[x][y]) {
        dataCell c = data[x][y];
        if (!isOccupied(x,y,c)) {
          
          bool isSurrounded = true;
          for (int dx=-1; dx<=1; dx++) {
            int nx = x+dx;
            if (nx<=0 || nx>=sizeX-1) continue;
            for (int dy=-1; dy<=1; dy++) {
              if (dx==0 && dy==0) continue;
              int ny = y+dy;
              if (ny<=0 || ny>=sizeY-1) continue;

              if (!gridMap[nx][ny]) {
                isSurrounded = false;
                break;
              }
            }
          }
          if (isSurrounded) {
            c.obstX = x;
            c.obstY = y;
            c.sqdist = 0;
            c.dist=0;
            c.voronoi=occupied;
            c.queueing = fwProcessed;
            data[x][y] = c;
          } else setObstacle(x,y);
        }
      }
    }
  }
}

void DynamicVoronoi::occupyCell(int x, int y) {
  gridMap[x][y] = true;
  setObstacle(x,y);
}
void DynamicVoronoi::clearCell(int x, int y) {
  gridMap[x][y] = false;
  removeObstacle(x,y);
}

void DynamicVoronoi::setObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c)) return;
  
  addList.push_back(INTPOINT(x,y));
  c.obstX = x;
  c.obstY = y;
  data[x][y] = c;
}

void DynamicVoronoi::removeObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c) == false) return;

  removeList.push_back(INTPOINT(x,y));
  c.obstX = invalidObstData;
  c.obstY  = invalidObstData;    
  c.queueing = bwQueued;
  data[x][y] = c;
}

void DynamicVoronoi::exchangeObstacles(const std::vector<IntPoint>& newObstacles) {
  bool** newGrid = new bool*[sizeX];
  for (int x=0; x<sizeX; x++) {
    newGrid[x] = new bool[sizeY];
    for (int y=0; y<sizeY; y++) newGrid[x][y] = false;
  }
  for (unsigned int i=0; i<newObstacles.size(); i++) {
    newGrid[newObstacles[i].x][newObstacles[i].y] = true;
  }
  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {
      if (gridMap[x][y] != newGrid[x][y]) {
        if (gridMap[x][y]) removeObstacle(x,y);
        else setObstacle(x,y);
        gridMap[x][y] = newGrid[x][y];
      }
    }
  }
  for (int x=0; x<sizeX; x++) delete[] newGrid[x];
  delete[] newGrid;
}

void DynamicVoronoi::update(bool updateRealDist) {
  commitAndColorize(updateRealDist);

  while (!open.empty()) {
    INTPOINT p = open.pop();
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing==fwProcessed) continue;

    if (c.needsRaise) {
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<0 || nx>=sizeX) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<0 || ny>=sizeY) continue;
          dataCell nc = data[nx][ny];
          if (nc.obstX!=invalidObstData && !nc.needsRaise) {
            if(!isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])) {
              open.push(nc.sqdist, INTPOINT(nx,ny));
              nc.queueing = fwQueued;
              data[nx][ny] = nc;
              continue;
            }
            if(nc.queueing != fwQueued) {
              open.push(nc.sqdist, INTPOINT(nx,ny));
              nc.queueing = fwQueued;
              data[nx][ny] = nc;
            }
          }
        }
      }
      c.needsRaise = false;
      c.queueing = bwProcessed;
      data[x][y] = c;
    }
    else if (c.obstX != invalidObstData && isOccupied(c.obstX,c.obstY,data[c.obstX][c.obstY])) {

      c.queueing = fwProcessed;
      c.voronoi = free;

      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<0 || nx>=sizeX) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<0 || ny>=sizeY) continue;
          dataCell nc = data[nx][ny];
          bool isSurrounded = true;
          for (int dx=-1; dx<=1; dx++) {
            int nnx = nx+dx;
            if (nnx<=0 || nnx>=sizeX-1) continue;
            for (int dy=-1; dy<=1; dy++) {
              if (dx==0 && dy==0) continue;
              int nny = ny+dy;
              if (nny<=0 || nny>=sizeY-1) continue;

              if (!gridMap[nnx][nny]) {
                isSurrounded = false;
                break;
              }
            }
          }
          if (!nc.needsRaise) {
            int distx = nx-c.obstX;
            int disty = ny-c.obstY;
            int newSqDistance = distx*distx + disty*disty;
            bool overwrite =  (newSqDistance < nc.sqdist);
            if(!overwrite && newSqDistance==nc.sqdist) { 
              if (nc.obstX == invalidObstData || isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])==false) overwrite = true;
            }
            if (overwrite) {
              open.push(newSqDistance, INTPOINT(nx,ny));
              nc.queueing = fwQueued;
              if (updateRealDist) {
                nc.dist = sqrt((double)newSqDistance);
              }
              nc.sqdist = newSqDistance;
              nc.obstX = c.obstX;
              nc.obstY = c.obstY;
            } else { 
              checkVoro(x,y,nx,ny,c,nc);
            }
            data[nx][ny] = nc;
          }
        }
      }
      data[x][y] = c;
    }
  }
}

float DynamicVoronoi::getDistance(int x, int y) const {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) return data[x][y].dist;
  else return -INFINITY;
}

bool DynamicVoronoi::isVoronoi(int x, int y) const {
  dataCell c = data[x][y];
  return (c.voronoi==free || c.voronoi==voronoiKeep);
}


bool DynamicVoronoi::isOccupied(int x, int y) const {
  dataCell c = data[x][y];
  return (c.obstX==x && c.obstY==y);
}

void DynamicVoronoi::visualize(const char *filename) {
  FILE* F = fopen(filename, "w");
  if (!F) {
    std::cerr << "could not open 'result.ppm' for writing!\n";
    return;
  }
  fprintf(F, "P6\n");
  fprintf(F, "%d %d 255\n", sizeX, sizeY);

  for(int y = sizeY-1; y >=0; y--){      
    for(int x = 0; x<sizeX; x++){
      unsigned char c = 0;
      if (isVoronoi(x,y)) {
        fputc( 255, F );
        fputc( 0, F );
        fputc( 0, F );
      } else if (data[x][y].sqdist==0) {
        fputc( 0, F );
        fputc( 0, F );
        fputc( 0, F );
      } else {
        float f = 80+(data[x][y].dist*5);
        if (f>255) f=255;
        if (f<0) f=0;
        c = (unsigned char)f;
        fputc( c, F );
        fputc( c, F );
        fputc( c, F );
      }
    }
  }
  fclose(F);
}

void DynamicVoronoi::prune() {
  for(int y=0; y<sizeY; y++) {
    for(int x=0; x<sizeX; x++) {
      dataCell c = data[x][y];
      if (c.voronoi==free) {
        int count = 0;
        for (int dx=-1; dx<=1; dx++) {
          int nx = x+dx;
          if (nx<0 || nx>=sizeX) continue;
          for (int dy=-1; dy<=1; dy++) {
            if (dx==0 && dy==0) continue;
            int ny = y+dy;
            if (ny<0 || ny>=sizeY) continue;
            if (data[nx][ny].voronoi==free) count++;
          }
        }
        if (count<=1) {
          c.voronoi = voronoiPrune;
          pruneQueue.push(INTPOINT(x,y));
        }
      }
      data[x][y] = c;
    }
  }
  while (!pruneQueue.empty()) {
    INTPOINT p = pruneQueue.front();
    pruneQueue.pop();
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];
    int count = 0;
    for (int dx=-1; dx<=1; dx++) {
      int nx = x+dx;
      if (nx<0 || nx>=sizeX) continue;
      for (int dy=-1; dy<=1; dy++) {
        if (dx==0 && dy==0) continue;
        int ny = y+dy;
        if (ny<0 || ny>=sizeY) continue;
        if (data[nx][ny].voronoi==free) count++;
      }
    }
    if (count<=1) {
      c.voronoi = voronoiPrune;
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<0 || nx>=sizeX) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<0 || ny>=sizeY) continue;
          if (data[nx][ny].voronoi==free) pruneQueue.push(INTPOINT(nx,ny));
        }
      }
    }
    data[x][y] = c;
  }
  for(int y=0; y<sizeY; y++) {
    for(int x=0; x<sizeX; x++) {
      dataCell c = data[x][y];
      if (c.voronoi==voronoiPrune) {
        c.voronoi = occupied;
      }
      data[x][y] = c;
    }
  }
}

inline void DynamicVoronoi::checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc) {
  if ((c.sqdist>1 || nc.sqdist>1) && nc.obstX!=invalidObstData) {
    if (abs(c.obstX-nc.obstX) > 1 || abs(c.obstY-nc.obstY) > 1) {
      int obX = (c.obstX+nc.obstX)/2;
      int obY = (c.obstY+nc.obstY)/2;
      if (gridMap[obX][obY] == false) {
        int d1x = x-obX;
        int d1y = y-obY;
        int d2x = nx-obX;
        int d2y = ny-obY;
        if (d1x*d1x+d1y*d1y > 1 || d2x*d2x+d2y*d2y > 1) {
          c.voronoi = free;
          nc.voronoi = free;
        }
      }
    }
  }
}

void DynamicVoronoi::commitAndColorize(bool updateRealDist) {
  for (unsigned int i=0; i<removeList.size(); i++) {
    INTPOINT p = removeList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if (c.queueing != fwProcessed) continue;

    if (updateRealDist) c.dist = INFINITY;
    c.sqdist = INT_MAX;
    c.needsRaise = true;
    c.obstX = invalidObstData;
    c.obstY = invalidObstData;
    c.queueing = bwQueued;
    c.voronoi = free;
    open.push(0, INTPOINT(x,y));
    data[x][y] = c;
  }

  for (unsigned int i=0; i<addList.size(); i++) {
    INTPOINT p = addList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if (c.queueing != fwProcessed) {
      if (updateRealDist) c.dist = 0;
      c.sqdist = 0;
      c.needsRaise = false;
      c.obstX = x;
      c.obstY = y;
      c.queueing = fwQueued;
      c.voronoi = occupied;
      open.push(0, INTPOINT(x,y));
      data[x][y] = c;
    }
  }

  removeList.clear();
  addList.clear();
}

inline bool DynamicVoronoi::isOccupied(int& x, int& y, dataCell& c) {
  return (c.obstX==x && c.obstY==y);
}
