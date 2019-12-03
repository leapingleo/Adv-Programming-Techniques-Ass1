
#include "Particle.h"

Particle::Particle(int x, int y, Orientation orientation){
  this->x = x;
  this->y = y;
  this->orientation = orientation;
}

// x-co-ordinate of the particle
int Particle::getX() {
   return x;
}

// y-co-ordinate of the particle
int Particle::getY() {
   return y;
}

// Orientation of the particle
Orientation Particle::getOrientation() {
   return orientation;
}
