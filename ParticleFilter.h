
#ifndef COSC_ASS_ONE_PARTICLE_FILTER
#define COSC_ASS_ONE_PARTICLE_FILTER

#include "Particle.h"
#include "ParticleList.h"
#include "Types.h"

#define MAZE_SIZE          100

class ParticleFilter {
public:

   /*                                           */
   /* DO NOT MOFIFY ANY CODE IN THIS SECTION    */
   /*                                           */


   // Initialise a new particle filter with a given maze of size (x,y)
   ParticleFilter(Grid maze, int rows, int cols);

   // Clean-up the Particle Filter
   ~ParticleFilter();

   // A new observation of the robot, of size 3x3
   void newObservation(Grid observation);


   // Return a DEEP COPY of the ParticleList of all particles representing
   //    the current possible locations of the robot
   ParticleList* getParticles();

   /*                                           */
   /* YOU MAY ADD YOUR MODIFICATIONS HERE       */
   /*                                           */

   //returns the int value from an observation
   Orientation getObservationOrien(Grid observation);

   //return a forward movment value on the Y-axis
   int forwardMoveOnY(int n, Orientation orientation);

   //return a forward movment value on the X-axis
   int forwardMoveOnX(int n, Orientation orientation);

   //check if an observation matches a part of the maze (3 x 3)
   bool isObsMazeMatched(Grid maze, int row, int col, Grid obs);

   bool isFacingSameOrien(Grid currObs, Grid prevObs);

   //returns the new Obs with left rotation
   Grid obsRotateLeft(Grid observation);

   //returns an upside down observation
   Grid obsUpsideDown(Grid observation);

   //returns a right rotated observation
   Grid obsRotateRight(Grid observation);

   //[MILESTONE-3] the p' generator method takes three cases since the robot can
   //only rotate right or left and make a forward movement
   void generatePApostro(ParticleList* p, ParticleList* p1, Orientation Orientation,
                                         Grid currObs, Grid prevObs);

   //check if the current observation is rotated left from the last observation
   bool isObsRotatedLeft(Grid currObs, Grid prevObs);

   //check if the current observation is rotated right from the last observation
   bool isObsRotatedRight(Grid currObs, Grid prevObs);

   //check if the current observation and the last observation is the same
   bool isObsRemainedSame(Grid currObs, Grid prevObs);

   void removeByMap(Grid observation, ParticleList* p);

   //return next left-turn orientation
   Orientation nextLeftOrientation(Orientation orientation);

   //return next right-turn orientation
   Orientation nextRightOrientation(Orientation orientation);

   void addToParticleList(ParticleList* p, ParticleList* subP);

   //produce a deep copy from p1 to p
   void copyParticles(ParticleList* p, ParticleList* p1);


 private:
    ParticleList* particleList;
    ParticleList* particleList_Left;
    ParticleList* particleList_Up;
    ParticleList* particleList_Right;
    ParticleList* particleList_Down;

    char** maze;
    int rows;
    int cols;
    Grid prevObservation;
    Orientation prevOrientation;
    int count = 0;
};

#endif // COSC_ASS_ONE_PARTICLE_FILTER
