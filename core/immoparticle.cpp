/* Copyright (C) 2021 Joshua J. Daymude, Robert Gmyr, and Kristian Hinnenthal.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

#include "core/immoparticle.h"

ImmoParticle::ImmoParticle(const Node& node) :
    immobilized(true),
    _node(node) {}
