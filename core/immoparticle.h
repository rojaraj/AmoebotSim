/* Copyright (C) 2021 Joshua J. Daymude, Robert Gmyr, and Kristian Hinnenthal.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

// Defines an immobilized particle, which is a single node of a solid object.

#ifndef AMOEBOTSIM_CORE_IMMOPARTICLE_H_
#define AMOEBOTSIM_CORE_IMMOPARTICLE_H_

#include "core/node.h"

class ImmoParticle {

 public:


  // Constructs an immobilized particle entity positioned at the given Node
    ImmoParticle(const Node& _node = Node());

  Node _node;
   bool immobilized;

};

#endif  // AMOEBOTSIM_CORE_IMMOPARTICLE_H_
