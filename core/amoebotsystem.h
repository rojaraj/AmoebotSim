/* Copyright (C) 2021 Joshua J. Daymude, Robert Gmyr, and Kristian Hinnenthal.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

// Defines a particle system that complies with the properties and constraints
// of the amoebot model, but is independent of any particular algorithm.

#ifndef AMOEBOTSIM_CORE_AMOEBOTSYSTEM_H_
#define AMOEBOTSIM_CORE_AMOEBOTSYSTEM_H_

#include <deque>
#include <map>
#include <set>
#include <vector>

#include <QString>

#include "core/metric.h"
#include "core/immoparticle.h"
#include "core/system.h"
#include "helper/randomnumbergenerator.h"

// AmoebotParticle must be forward declared to avoid a cyclic dependency.
class AmoebotParticle;

class AmoebotSystem : public System, public RandomNumberGenerator {
  friend class AmoebotParticle;

 public:
  // Constructs a new particle system with fresh round, activation, and movement
  // counts.
  AmoebotSystem();

  // Deletes the particles, objects, and metrics in this system before
  // destructing the system.
  virtual ~AmoebotSystem();

  // Functions for activating a particle in the system. activate activates a
  // random particle in the system, while activateParticleAt activates the
  // particle occupying the specified node if such a particle exists.
  void activate() final;
  void activateParticleAt(Node node) final;

  // Returns the number of particles in the system.
  unsigned int size() const final;

  // Returns the number of immobilized particles in the system.
  unsigned int numImmoParticles() const final;

  // Returns a reference to the particle at the specified index of particles.
  const Particle& at(int i) const final;

  // Returns a reference to the immobilized particle list.
  virtual const std::deque<ImmoParticle*>& getImmoParticles() const final;

  // Inserts a particle or an object, respectively, into the system. A particle
  // can be contracted or expanded. Fails if the respective node(s) are already
  // occupied.
  void insert(AmoebotParticle* particle);
  void insert(ImmoParticle* ImmoParticle);

  // Removes the specified particle from the system.
  void remove(AmoebotParticle* particle);

  // Functions for logging system progress. registerMovement logs the given
  // number of movements the system has made. registerActivation logs that the
  // given particle has been activated. When all particles have been activated
  // at least once, this resets its logging and triggers registerRound(), which
  // commits all counts and measures to their histories and increments the
  // number of completed asynchronous rounds by one.
  void registerMovement(unsigned int numMoves = 1);
  void registerActivation(AmoebotParticle* particle);
  void registerRound();

  // Various access functions for metrics (counts and measures). getCounts
  // (resp., getMeasures) returns a reference to the count (resp., measure)
  // list. getCount (resp., getMeasure) returns a reference to the named count
  // (resp., measure). These functions crash if the requested count/measure is
  // not found!
  const std::vector<Count*>& getCounts() const final;
  const std::vector<Measure*>& getMeasures() const final;
  Count& getCount(QString name) const final;
  Measure& getMeasure(QString name) const final;

  // Formats the count and measure histories as a JSON string. The structure of
  // this JSON string can be found in the Usage documentation.
  const QString metricsAsJSON() const final;


  // Currently used in the function updateBorderColors
  // in the class ShapeFormationFaultTolerantParticle
  // to draw the boundaries of the hexagon layers.
  // Purely visualization related, i.e. the value is not used
  // in the fault-tolerant hexagon shape formation algorithm.
  int seedOrientation() const;


 protected:
  std::vector<AmoebotParticle*> particles;
  std::map<Node, AmoebotParticle*> particleMap;
  std::set<AmoebotParticle*> activatedParticles;
  std::deque<ImmoParticle*> immoparticles;
  std::map<Node, ImmoParticle*> immoparticleMap;
  std::vector<Count*> _counts;
  std::vector<Measure*> _measures;
  int _seedOrientation;
};

#endif  // AMOEBOTSIM_CORE_AMOEBOTSYSTEM_H_
