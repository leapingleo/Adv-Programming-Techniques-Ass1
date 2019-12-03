
#include "ParticleList.h"

#include <cstdio>
#include <cstdlib>
#include <iostream>

#define PARTICLE_SIZE      10000

#ifndef DEBUG
#define DEBUG        0
#endif

// Initialise a new particle filter with a given maze of size (x,y)
ParticleList::ParticleList() {
}

// Clean-up the particle list
ParticleList::~ParticleList() {
  for (int i = 0; i < numParticles; i++)
    if (particles[i] != NULL)
      delete particles[i];
}

// Number of particles in the ParticleList
int ParticleList::getNumberParticles() {
   return numParticles;
}

// Get a pointer to the i-th particle in the list
ParticlePtr ParticleList::get(int i) {
   return particles[i];
}

ParticlePtr ParticleList::getAt(int row, int col){
  for (int i = 0; i < numParticles; i++)
    if (particles[i]->getX() == col && particles[i]->getY() == row)
        return particles[i];
  return NULL;
}

void ParticleList::deleteAt(int row, int col){
  for (int i = 0; i < numParticles; i++)
    if (particles[i]->getX() == col && particles[i]->getY() == row) {
        numParticles--;
        for (int j = i; j < numParticles; ++j) {
             ParticlePtr ptr = new Particle(particles[j + 1]->getX(),
                                            particles[j + 1]->getY(),
                                            particles[j + 1]->getOrientation());
            //   delete particles[j+1];
            particles[j] = ptr;
            particles[j+1] = NULL;
            delete particles[j+1];
          }
    }
}

// Add a particle (as a pointer) to the list
//    This class now has control over the pointer
//    And should delete the pointer if the particle is removed from the list
void ParticleList::add_back(ParticlePtr particle) {
  particles[numParticles++] = particle;
}

// Remove all particles from the list
void ParticleList::clear() {
  for (int i = 0; i < numParticles; i++){
    delete particles[i];
  }
  numParticles = 0;
}
