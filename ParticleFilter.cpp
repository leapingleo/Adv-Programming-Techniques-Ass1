#include "ParticleFilter.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>

// Initialise a new particle filter with a given maze of size (x,y)
ParticleFilter::ParticleFilter(char** maze, int rows, int cols) {
  particleList = new ParticleList();
  particleList_Left = new ParticleList();
  particleList_Up = new ParticleList();
  particleList_Right = new ParticleList();
  particleList_Down = new ParticleList();

  this->maze = maze;
  this->rows = rows;
  this->cols = cols;
}

// Clean-up the Particle Filter
ParticleFilter::~ParticleFilter() {
  if (maze != NULL)
    delete maze;
  if (particleList_Left != NULL)
    delete particleList_Left;
  if (particleList_Up != NULL)
    delete particleList_Up;
  if (particleList_Right != NULL)
    delete particleList_Right;
  if (particleList_Down != NULL)
    delete particleList_Down;
}

// A new observation of the robot, of size 3x3
void ParticleFilter::newObservation(Grid observation) {
    Orientation currentOrientation = getObservationOrien(observation);
    ParticleList* particleList_P = new ParticleList();

    ParticleList* particleList_Left_P = new ParticleList();
    ParticleList* particleList_Up_P = new ParticleList();
    ParticleList* particleList_Right_P = new ParticleList();
    ParticleList* particleList_Down_P = new ParticleList();

    if (observation[1][1] != '*') {
      //before the very first observation, generate all possible locations
      if (particleList->getNumberParticles() == 0) {
        for (int row = 0; row < rows; row++)
          for (int col = 0; col < cols; col++)
            if (maze[row][col] == '.') {
              ParticlePtr ptr = new Particle(col, row, getObservationOrien(observation));
              particleList_P->add_back(ptr);
            }
    } else {
      //generate p' based on P
      for (int i = 0; i < particleList->getNumberParticles(); i++) {
        //when last obs and current obs facing the same direction, means forward movement is made
        if (currentOrientation == prevOrientation){
          //generate a forward movement particle
          int x = forwardMoveOnX(particleList->get(i)->getX(), currentOrientation);
          int y = forwardMoveOnY(particleList->get(i)->getY(), currentOrientation);

          ParticlePtr ptr = new Particle(x, y, currentOrientation);
          particleList_P->add_back(ptr);
        } else {
          ParticlePtr ptr = new Particle(particleList->get(i)->getX(),
                                particleList->get(i)->getY(), currentOrientation);
          particleList_P->add_back(ptr);
        }
      }
    }
      removeByMap(observation, particleList_P);
      particleList->clear();
      copyParticles(particleList, particleList_P);
    }
    //When direction of the robot is not indicated
    else {
      Grid upObs = observation;
      Grid leftObs = obsRotateLeft(observation);
      Grid rightObs = obsRotateRight(observation);
      Grid downObs = obsUpsideDown(observation);

      //getnerate FOUR partilcelist p' for FOUR different rotation at first observation
      if (particleList->getNumberParticles() == 0) {
        for (int row = 0; row < rows; row++){
          for (int col = 0; col < cols; col++){
            if (maze[row][col] == '.') {
                ParticlePtr ptr1 = new Particle(col, row, ORIEN_LEFT);
                particleList_Left_P->add_back(ptr1);
                ParticlePtr ptr2 = new Particle(col, row, ORIEN_UP);
                particleList_Up_P->add_back(ptr2);
                ParticlePtr ptr3 = new Particle(col, row, ORIEN_RIGHT);
                particleList_Right_P->add_back(ptr3);
                ParticlePtr ptr4 = new Particle(col, row, ORIEN_DOWN);
                particleList_Down_P->add_back(ptr4);
            }
          }
        }
        removeByMap(leftObs, particleList_Left_P);
        removeByMap(upObs, particleList_Up_P);
        removeByMap(rightObs, particleList_Right_P);
        removeByMap(downObs, particleList_Down_P);
      } else {
        //generate p' using p for each rotated observation
        generatePApostro(particleList_Left_P, particleList_Left, ORIEN_LEFT, observation, prevObservation);
        generatePApostro(particleList_Up_P, particleList_Up, ORIEN_UP, observation, prevObservation);
        generatePApostro(particleList_Right_P, particleList_Right, ORIEN_RIGHT, observation, prevObservation);
        generatePApostro(particleList_Down_P, particleList_Down, ORIEN_DOWN, observation, prevObservation);

        if (prevObservation != NULL && (isObsRotatedRight(observation, prevObservation)
                    || isObsRotatedLeft(observation, prevObservation))) {
          upObs = prevObservation;
          leftObs = obsRotateLeft(prevObservation);
          rightObs = obsRotateRight(prevObservation);
          downObs = obsUpsideDown(prevObservation);
        }
        //put four P' list into one big P' list for removing particles using MAP
        addToParticleList(particleList_P, particleList_Left_P);
        addToParticleList(particleList_P, particleList_Up_P);
        addToParticleList(particleList_P, particleList_Right_P);
        addToParticleList(particleList_P, particleList_Down_P);
      }

      removeByMap(leftObs, particleList_P);
      removeByMap(upObs, particleList_P);
      removeByMap(rightObs, particleList_P);
      removeByMap(downObs, particleList_P);

      //make p = p' with each rotation
      particleList_Left = particleList_Left_P;
      particleList_Up = particleList_Up_P;
      particleList_Right = particleList_Right_P;
      particleList_Down = particleList_Down_P;

      //put 4 p' list into one big p' list
      addToParticleList(particleList_P, particleList_Left_P);
      addToParticleList(particleList_P, particleList_Up_P);
      addToParticleList(particleList_P, particleList_Right_P);
      addToParticleList(particleList_P, particleList_Down_P);

      //clear particlelist P before making P = P'
      particleList->clear();
      copyParticles(particleList, particleList_P);
    }

    prevObservation = observation;
    prevOrientation = currentOrientation;

    for (int i = 0; i < particleList->getNumberParticles(); i++)
      std::cout << "(" << particleList->get(i)->getX() <<
                   ", " << particleList->get(i)->getY() <<
                   ", " << particleList->get(i)->getOrientation() <<
                   ")"<< std::endl;
}


//the p' generator method takes three cases since the robot can
//only rotate right or left and make a forward movement
void ParticleFilter::generatePApostro(ParticleList* p, ParticleList* p1, Orientation orientation,
                                      Grid currObs, Grid prevObs){

  for (int i = 0; i < p1->getNumberParticles(); i++){

    if (isObsRotatedRight(currObs, prevObs)){
      int x = p1->get(i)->getX();
      int y = p1->get(i)->getY();

      ParticlePtr ptr = new Particle(x, y, nextRightOrientation(orientation));
      p->add_back(ptr);
    }  else if (isObsRotatedLeft(currObs, prevObs)){
      int x = p1->get(i)->getX();
      int y = p1->get(i)->getY();

      ParticlePtr ptr = new Particle(x, y, nextLeftOrientation(orientation));
      p->add_back(ptr);


    } else {
      int x = forwardMoveOnX(p1->get(i)->getX(), p1->get(i)->getOrientation());
      int y = forwardMoveOnY(p1->get(i)->getY(), p1->get(i)->getOrientation());
      ParticlePtr ptr = new Particle(x, y, p1->get(i)->getOrientation());
      p->add_back(ptr);

    }
  }
}

//returns a forward movement value on the Y-axis
int ParticleFilter::forwardMoveOnY(int n, Orientation orientation){
  int a = n;
  if (orientation == ORIEN_UP)
    a = n - 1;
  else if (orientation == ORIEN_DOWN)
    a = n + 1;
  return a;
}

//return a forward movment value on the X-axis
int ParticleFilter::forwardMoveOnX(int n, Orientation orientation){
  int a = n;
  if (orientation == ORIEN_LEFT)
    a = n - 1;
  else if (orientation == ORIEN_RIGHT)
    a = n + 1;
  return a;
}

void ParticleFilter::addToParticleList(ParticleList* p, ParticleList* subP){
   for (int i = 0; i < subP->getNumberParticles(); i++){
      p->add_back(subP->get(i));
    }
}

//remove a particle using the map
void ParticleFilter::removeByMap(Grid observation, ParticleList* p){
  for (int row = 0; row < rows; row++)
    for (int col = 0; col < cols; col++)
      if (maze[row][col] == '.')
        if (!isObsMazeMatched(maze, row, col, observation))
          for (int i = 0; i < p->getNumberParticles(); i++)
            p->deleteAt(row, col);
}



bool ParticleFilter::isFacingSameOrien(Grid currObs, Grid prevObs){
    return currObs[1][1] == prevObs[1][1];
}

//returns the int value from an observation
Orientation ParticleFilter::getObservationOrien(Grid observation){
    Orientation orien = ORIEN_DOWN;
    if (observation[1][1] == '^')
      orien = ORIEN_UP;
    else if (observation[1][1] == '>')
      orien = ORIEN_RIGHT;
    else if (observation[1][1] == '<')
      orien = ORIEN_LEFT;
    else if (observation[1][1] == 'v')
      orien = ORIEN_DOWN;
    return orien;
}

//check if an observation matches a part of the maze (3 x 3)
bool ParticleFilter::isObsMazeMatched(Grid maze, int row, int col, Grid obs){
    return maze[row - 1][col - 1] == obs[0][0] &&
           maze[row][col - 1] == obs[1][0] &&
           maze[row + 1][col - 1] == obs[2][0] &&
           maze[row - 1][col] == obs[0][1] &&
           maze[row + 1][col] == obs[2][1] &&
           maze[row - 1][col + 1] == obs[0][2] &&
           maze[row][col + 1] == obs[1][2] &&
           maze[row + 1][col + 1] == obs[2][2];
}

//returns the new Obs with left rotation
Grid ParticleFilter::obsRotateLeft(Grid observation){
  Grid newObs = NULL;

  for (int i = 0; i < 3; i++) {
     newObs = new char*[3];
     for (int j = 0; j < 3; j++) {
        newObs[j] = new char[3];
     }
  }
  newObs[0][0] = observation[0][2];
  newObs[0][1] = observation[1][2];
  newObs[0][2] = observation[2][2];
  newObs[1][0] = observation[0][1];
  newObs[1][2] = observation[2][1];
  newObs[2][0] = observation[0][0];
  newObs[2][1] = observation[1][0];
  newObs[2][2] = observation[2][0];
  newObs[1][1] = '*';

  return newObs;
}

//returns an upside down observation
Grid ParticleFilter::obsUpsideDown(Grid observation){
  Grid newObs = NULL;

  for (int i = 0; i < 3; i++) {
     newObs = new char*[3];
     for (int j = 0; j < 3; j++) {
        newObs[j] = new char[3];
     }
  }

  newObs[0][0] = observation[2][2];
  newObs[0][1] = observation[2][1];
  newObs[0][2] = observation[2][0];
  newObs[1][0] = observation[1][2];
  newObs[1][2] = observation[1][0];
  newObs[2][0] = observation[0][2];
  newObs[2][1] = observation[0][1];
  newObs[2][2] = observation[0][0];
  newObs[1][1] = '*';

  return newObs;
}

//returns a right rotated observation
Grid ParticleFilter::obsRotateRight(Grid observation){
  Grid newObs = NULL;

  for (int i = 0; i < 3; i++) {
     newObs = new char*[3];
     for (int j = 0; j < 3; j++) {
        newObs[j] = new char[3];
     }
  }
  newObs[0][0] = observation[2][0];
  newObs[0][1] = observation[1][0];
  newObs[0][2] = observation[0][0];
  newObs[1][0] = observation[2][1];
  newObs[1][2] = observation[0][1];
  newObs[2][0] = observation[2][2];
  newObs[2][1] = observation[1][2];
  newObs[2][2] = observation[0][2];
  newObs[1][1] = '*';

  return newObs;
}

//check if the current observation is rotated left from the last observation
bool ParticleFilter::isObsRotatedLeft(Grid currObs, Grid prevObs){
  return currObs[0][0] == prevObs[2][0] &&
         currObs[0][1] == prevObs[1][0] &&
         currObs[0][2] == prevObs[0][0] &&
         currObs[1][0] == prevObs[2][1] &&
         currObs[1][2] == prevObs[0][1] &&
         currObs[2][0] == prevObs[2][2] &&
         currObs[2][1] == prevObs[1][2] &&
         currObs[2][2] == prevObs[0][2];
}

//check if the current observation is rotated right from the last observation
bool ParticleFilter::isObsRotatedRight(Grid currObs, Grid prevObs){
  return currObs[0][0] == prevObs[0][2] &&
         currObs[0][1] == prevObs[1][2] &&
         currObs[0][2] == prevObs[2][2] &&
         currObs[1][0] == prevObs[0][1] &&
         currObs[1][2] == prevObs[2][1] &&
         currObs[2][0] == prevObs[0][0] &&
         currObs[2][1] == prevObs[1][0] &&
         currObs[2][2] == prevObs[2][0];
}

//check if the current observation and the last observation is the same
bool ParticleFilter::isObsRemainedSame(Grid currObs, Grid prevObs){
  return currObs[0][0] == prevObs[0][0] &&
         currObs[0][1] == prevObs[0][1] &&
         currObs[0][2] == prevObs[0][2] &&
         currObs[1][0] == prevObs[1][0] &&
         currObs[1][2] == prevObs[1][2] &&
         currObs[2][0] == prevObs[2][0] &&
         currObs[2][1] == prevObs[2][1] &&
         currObs[2][2] == prevObs[2][2];
}

//return next left-turn orientation
Orientation ParticleFilter::nextLeftOrientation(Orientation orientation){
  Orientation nextOrien = orientation;

  if (orientation == ORIEN_DOWN)
    nextOrien = ORIEN_RIGHT;
  else if (orientation % 3 == 0)
    nextOrien = ORIEN_DOWN;
  else
    nextOrien = orientation % 3 - 1;
  return nextOrien;
}

//return next right-turn orientation
Orientation ParticleFilter::nextRightOrientation(Orientation orientation){
  Orientation nextOrien = orientation;

  if (orientation == ORIEN_DOWN)
    nextOrien = ORIEN_LEFT;
  else if (orientation % 3 == 0)
    nextOrien = ORIEN_UP;
  else
    nextOrien = orientation % 3 + 1;
  return nextOrien;
}

//produce a deep copy from p1 to p
void ParticleFilter::copyParticles(ParticleList* p, ParticleList* p1){
  for (int i = 0; i < p1->getNumberParticles(); i++){
    ParticlePtr ptr = new Particle(p1->get(i)->getX(),
                          p1->get(i)->getY(), p1->get(i)->getOrientation());
    p->add_back(ptr);
  }
}

// Return a DEEP COPY of the ParticleList of all particles representing
//    the current possible locations of the robot
ParticleList* ParticleFilter::getParticles() {
   return particleList;
}
