//immo_shape working .cpp file

#include <set>
#include <queue> // Include this to use std::queue
#include "immobilizedparticles.h"
#include <QtGlobal>

// Function declaration before it's used in the constructor
bool doesEnclosureOccur(const std::set<Node>& occupied, Node testNode);
Immobilizedparticles::Immobilizedparticles(const Node head,
                                           const int globalTailDir,
                                           const int orientation,
                                           AmoebotSystem& system,
                                           State state)
    : AmoebotParticle(head, globalTailDir, orientation, system),
    state(state),
    moveDir(-1),
    followDir(-1),
    leaderToken(false),
    tokenForwardDir(-1),
    tokenCurrentDir(-1),
    passForward(false),
    freeState(false),
    lineState(false),
    _borderColorsSet(false)
{
    _borderColors.fill(-1);
    _borderPointColors.fill(-1);
}


void Immobilizedparticles::activate() {
    // This the main procedure that runs when a particle is activated.

    if (leaderToken) {
        // p has received the leader token.
        // If p has an empty neighbour node, it can become the Leader.
        // Otherwise, it needs to pass on the token.
        tryToBecomeLeader();
        return;
        // The return is only here because it looks nicer when the particle does not start moving as soon as it becomes Leader.
    } else if (isInState({State::Idle})) {
        // p tries to follow any neighbour that is either the Leader or a Follower.
        for (int label : randLabels()) {
            if (hasNbrAtLabel(label)) {
                auto& nbr = nbrAtLabel(label);
                if ((nbr.isInState({State::Follower}) && !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir)))
                    || (nbr.isInState({State::Leader})   && (nbr.moveDir < 0 || !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.moveDir))))) {
                    followDir = labelToDir(label);
                    state = State::Follower;
                    break;
                }
            }
        }
    } else if (isInState({State::Leader})) {
        if (moveDir == -1) {
            moveDir = randDir();
        }
        if (isContracted() && !canExpand(dirToHeadLabel(moveDir))) {
            // p cannot move in its moveDir. Try to change moveDir to point towards an empty node.
            bool changed = false;
            for (int label : randLabels()) {
                moveDir = labelToDir(label);
                if (!hasNbrAtLabel(label) && !hasObjectAtLabel(label)) {
                    if (isExpanded()) {
                        makeHeadLabel(label);
                    }
                    changed = true;
                    break;
                }
            }

            if (!changed) {
                // p's node has no empty neighbouring node.
                // p needs to pass on the leader token to a suitable random neighbour.
                int label = labelOfFirstNbrInState({State::Idle, State::Follower});
                if (label >= 0) {
                    passLeaderToken(label);
                    moveDir = -1;
                } else {
                    // This can only happen if the Leader is the only particle and it is
                    // surrounded by ImmoParticles.
                    // So basically never, unless you are testing weird edge cases.
                }
            }
        }
    }

    if (!isInState({State::Idle})) {
        performMovement();
    }

    return;
}


void Immobilizedparticles::tryToBecomeLeader() {
    for (int label : randLabels()) {
        // Try to find a movement direction that is not occupied.
        moveDir = labelToDir(label);
        if ((isExpanded() && !hasNbrAtLabel(label) && !hasObjectAtLabel(label)) || canExpand(labelToDir(label))) {
            if (isExpanded()) {
                makeHeadLabel(label);
            }
            state = State::Leader;
            leaderToken = false;
            tokenForwardDir = -1;
            tokenCurrentDir = -1;
            passForward = false;
            updateBorderPointColors(); // Only necessary for the visualisation.
            return;
        }
    }

    // Reset moveDir since the particle has not become the Leader.
    moveDir = -1;

    // Could not become Leader: Pass on leader token to a suitable neighbour.

    // Find the label that corresponds to the current token direction.
    int label = 0;
    for (int l : uniqueLabels()) {
        if (labelToDir(l) == tokenCurrentDir) {
            label = l;
        }
    }

    int numLabels = isContracted() ? 6 : 10;

    if (passForward) {
        // Try to pass the token towards the designated token direction.
        while (hasObjectAtLabel(label)) {
            // Cannot move towards the designated token direction, take the next possible
            // label with a node that is not occupied by an object.
            label = (label + 1) % numLabels;
            passForward = false;
        }
    } else {
        // Pass the token around the neighbouring object.
        do {
            // Set label to point towards the object that p is moving around.
            label = (label - 1 + numLabels) % numLabels;
        } while (!hasObjectAtLabel(label));
        while (hasObjectAtLabel(label)) {
            // Set label to the first node that is no longer occupied by an object.
            label = (label + 1) % numLabels;
        }
    }

    passLeaderToken(label);
}



void Immobilizedparticles::passLeaderToken(const int label) {
    if (isExpanded()) {
        makeHeadLabel(label);
    }

    // Follow the particle that will receive the particle.
    state = State::Follower;
    followDir = labelToDir(label);
    if (tokenForwardDir < 0) {
        // If p was a Leader, the designated token direction needs to be set.
        // Initialise to the direction that the token is passed to.
        passForward = true;
        tokenForwardDir = followDir;
    }

    // Pass the token with all its variables.
    auto& nbr = nbrAtLabel(label);
    nbr.leaderToken = true;
    nbr.tokenForwardDir = dirToNbrDir(nbr, tokenForwardDir);
    nbr.tokenCurrentDir = dirToNbrDir(nbr, followDir);
    nbr.passForward = passForward;
    if (nbr.tokenForwardDir == nbr.tokenCurrentDir && tokenForwardDir != tokenCurrentDir) {
        if (randBool()) {
            nbr.passForward = true;
        }
    }
    nbr.updateBorderPointColors(); // Only necessary for the visualisation.

    // Reset internal variables that are only required when p has the leader token.
    leaderToken = false;
    tokenForwardDir = -1;
    tokenCurrentDir = -1;
    passForward = false;
    updateBorderPointColors(); // Only necessary for the visualisation.
}

void Immobilizedparticles::performMovement() {
    // Fairly obvious: p tries to expand if contracted and contract if expanded.
    // The usual constraints apply.
    if (isExpanded() && isInState({State::Follower, State::Leader}) && !hasBlockingTailNbr()) {
        contractTail();
    } else if (isContracted() && isInState({State::Follower}) && hasTailAtLabel(dirToHeadLabel(followDir))) {
        int followLabel = dirToHeadLabel(followDir);
        auto& nbr = nbrAtLabel(followLabel);
        int nbrContractionDir = nbrDirToDir(nbr, (nbr.tailDir() + 3) % 6);
        push(followLabel);
        followDir = nbrContractionDir;
    } else if (isInState({State::Leader}) && isContracted() && !(freeState && lineState) && canExpand(dirToHeadLabel(moveDir))) {
        expand(dirToHeadLabel(moveDir));
    }
    updateBoolStates();
    updateBorderColors(); // Only necessary for the visualisation.
}
bool Immobilizedparticles::hasBlockingTailNbr() const {
    for (int label : uniqueLabels()) {
        if (hasNbrAtLabel(label)) {
            auto& nbr = nbrAtLabel(label);
            if (nbr.isInState({State::Idle})
                || (nbr.isInState({State::Follower}) && pointsAtMyTail(nbr, nbr.dirToHeadLabel(nbr.followDir)))) {
                return true;
            }
        }
    }

    return false;
}

void Immobilizedparticles::updateBoolStates() {
    // Step 1: Update freeState

    // A particle is 'free' if it is not adjacent to any ImmoParticles and all its children are 'free'.
    // freeState and lineState are only relevant once all Idle particles have changed their state.
    if (hasObjectNbr() || hasNbrInState({State::Idle})) {
        freeState = false;
        lineState = false;
        return;
    }

    freeState = true;
    for (int label : uniqueLabels()) {
        if (hasNbrAtLabel(label)) {
            auto& nbr = nbrAtLabel(label);
            if (nbr.isInState({State::Follower}) && pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir))) {
                freeState = freeState && nbr.freeState;
            }
        }
    }


    // Step 2: Update lineState

    // The lineState only becomes relevant when the particle is free from all objects.
    if (!freeState) {
        lineState = false;
        return;
    }

    // A particle's lineState is true if its movement direction matches that of its parent
    // (unless it is the Leader) and all its children's lineState is true.
    if (isInState({State::Follower})) {
        int parentDir = -1;
        auto& nbr = nbrAtLabel(dirToHeadLabel(followDir));
        if (nbr.isInState({State::Leader}) && nbr.moveDir >= 0) {
            parentDir = nbrDirToDir(nbr, nbr.moveDir);
        } else if (nbr.isInState({State::Follower})) {
            parentDir = nbrDirToDir(nbr, nbr.followDir);
        }

        if (parentDir < 0 || parentDir != followDir) {
            lineState = false;
            return;
        }
    }

    lineState = true;
    for (int label : uniqueLabels()) {
        if (hasNbrAtLabel(label)) {
            auto& nbr = nbrAtLabel(label);
            if (nbr.isInState({State::Follower}) && pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir))) {
                lineState = lineState && nbr.lineState;
            }
        }
    }
}

std::vector<int> Immobilizedparticles::randLabels() {
    std::vector<int> result = uniqueLabels();
    RandomNumberGenerator::shuffle(std::begin(result), std::end(result));
    return result;
}


int Immobilizedparticles::particleColor() const {
    // The inner color of a particle.
    if (leaderToken) {
        return 0xff8800; // orange
    } else if (isInState({State::Idle}) || lineState) {
        return 0x000000; // black
    } else if (freeState) {
        return 0x696969; // grey
    }
    return 0xffffff; // white
}

int Immobilizedparticles::headMarkColor() const {
    // The color of the circle around the particle.
    switch (state) {
    case State::Leader:       return 0xff8800; // orange
    case State::Follower:     return 0x0000ff; // blue
    default:                  return -1;
    }
    return -1;
}

int Immobilizedparticles::headMarkDir() const {
    // This function is used when a particle needs a direction marker at its head.
    // Here, it is used to visualise the particle's follow / movement direction.
    if (isInState({State::Follower})) {
        return followDir;
    }
    return moveDir;
}

int Immobilizedparticles::tailMarkColor() const {
    return headMarkColor();
}

std::array<int, 18> Immobilizedparticles::borderColors() const {
    return _borderColors;
}

std::array<int, 6> Immobilizedparticles::borderPointColors() const {
    return _borderPointColors;
}





void Immobilizedparticles::updateBorderColors() {
    // This function can be used to draw lines around the particle.
    // Here, the function is used to outline the final line.
    if (lineState && isInState({State::Follower, State::Leader})) {
        int globalLineDir;
        if (isInState({State::Follower})) {
            globalLineDir = localToGlobalDir(followDir);
        } else {
            globalLineDir = localToGlobalDir(moveDir);
        }
        int offset = 3 * globalLineDir;
        std::vector<int> linePositions = {3, 5, 7, 12, 14, 16};
        for (int linePos : linePositions) {
            _borderColors.at((linePos + offset) % 18) = 0xa0a0a0; // light grey
        }
    } else {
        _borderColors.fill(-1);
    }
}

void Immobilizedparticles::updateBorderPointColors() {
    // This function can be used to draw small dots around the particle.
    // Here, the dots are used to show the visualise the token directions.
    if (leaderToken) {
        Q_ASSERT(tokenCurrentDir >= 0);
        Q_ASSERT(tokenForwardDir >= 0);
        _borderPointColors.at(localToGlobalDir(tokenCurrentDir)) = 0xa0a0a0; // light grey
        if (passForward) {
            _borderPointColors.at(localToGlobalDir(tokenForwardDir)) = 0x00cc00; // green
        } else {
            _borderPointColors.at(localToGlobalDir(tokenForwardDir)) = 0xff0000; // red
        }
    } else {
        _borderPointColors.fill(-1);
    }
}

QString Immobilizedparticles::inspectionText() const {
    QString text;
    text += "head: (" + QString::number(head.x) + ", " + QString::number(head.y) +
            ")\n";
    text += "orientation: " + QString::number(orientation) + "\n";
    text += "globalTailDir: " + QString::number(globalTailDir) + "\n";
    text += "state: ";
    text += [this]() {
        switch (state) {
        case State::Leader:   return "Leader";
        case State::Idle:     return "Idle";
        case State::Follower: return "Follower";
        default:              return "no state";
        }
    }();
    text += "\n";
    text += "leaderToken: " + QString(leaderToken ? "true" : "false") + "\n";
    if (leaderToken) {
        text += "tokenForwardDir: " + QString::number(tokenForwardDir) + "\n";
        text += "tokenCurrentDir: " + QString::number(tokenCurrentDir) + "\n";
        text += "passForward: " + QString(passForward ? "true" : "false") + "\n";
    } else if (state == State::Leader || state == State::Follower) {
        text += "freeState: " + QString(freeState ? "true" : "false") + "\n";
        text += "lineState: " + QString(lineState ? "true" : "false") + "\n";
    }

    return text;
}

Immobilizedparticles& Immobilizedparticles::nbrAtLabel(int label) const {
    return AmoebotParticle::nbrAtLabel<Immobilizedparticles>(label);
}

int Immobilizedparticles::labelOfFirstNbrInState(
    std::initializer_list<State> states, int startLabel, bool ignoreErrorParticles) const {
    auto prop = [&](const Immobilizedparticles& p) {
        for (auto state : states) {
            if (p.state == state) {
                return true;
            }
        }
        return false;
    };

    return labelOfFirstNbrWithProperty<Immobilizedparticles>(prop, startLabel, ignoreErrorParticles);
}

bool Immobilizedparticles::hasNbrInState(std::initializer_list<State> states)
    const {
    return labelOfFirstNbrInState(states) != -1;
}

bool Immobilizedparticles::isInState(std::initializer_list<State> states) const {
    for (auto _state : states) {
        if (_state == state) {
            return true;
        }
    }

    return false;
}



// ImmobilizedParticleSystem::ImmobilizedParticleSystem(int numParticles, int numImmoParticles, int genExpExample, int numCoinFlips) {
//     Q_ASSERT(numParticles > 0);
//     Q_ASSERT(numImmoParticles >= 0);
//     Q_ASSERT(genExpExample == 0 || genExpExample == 1);
//     Q_ASSERT(numCoinFlips > 0);

//     std::set<Node> occupied;
//     std::set<Node> candidates;

//     _seedOrientation = randDir();
//     Immobilizedparticles* leader = new Immobilizedparticles(Node(0, 0), -1, seedOrientation(), *this, Immobilizedparticles::State::Leader);
//     insert(leader);
//     occupied.insert(Node(0, 0));
//     numParticles--; // Decrement for initial leader particle

//     for (int i = 0; i < 6; ++i) {
//         candidates.insert(Node(0, 0).nodeInDir(i));
//     }

//     while (numParticles > 0 || numImmoParticles > 0) {
//         int randIndex = randInt(0, candidates.size());
//         auto it = candidates.begin();
//         std::advance(it, randIndex);
//         Node randomCandidate = *it;

//         if(!doesEnclosureOccur(occupied, randomCandidate)) {
//             if (randBool((double) numParticles / ((double) numParticles + (double) numImmoParticles))) {
//                 numParticles--; // Insert a non-immobilized particle
//                 insert(new Immobilizedparticles(randomCandidate, -1, randDir(), *this, Immobilizedparticles::State::Idle));
//             } else {
//                 numImmoParticles--; // Insert an immobilized particle
//                 insert(new ImmoParticle(randomCandidate)); // Assuming ImmoParticle represents an immobilized particle
//             }

//             occupied.insert(randomCandidate);
//             candidates.erase(it);

//             // Add surrounding nodes to candidates for next insertion
//             for (int i = 0; i < 6; ++i) {
//                 Node neighbor = randomCandidate.nodeInDir(i);
//                 if (occupied.find(neighbor) == occupied.end()) {
//                     candidates.insert(neighbor);
//                 }
//             }
//         }
//     }
// }

// bool doesEnclosureOccur(const std::set<Node>& occupied, Node testNode) {
//     std::queue<Node> toVisit;
//     std::set<Node> visited;

//     toVisit.push(testNode);
//     visited.insert(testNode);

//     while (!toVisit.empty()) {
//         Node currentNode = toVisit.front();
//         toVisit.pop();

//         for (int i = 0; i < 6; ++i) {
//             Node neighbor = currentNode.nodeInDir(i);

//             if (occupied.find(neighbor) == occupied.end()) {
//                 return false; // No enclosure, there's an escape route
//             }

//             if (visited.find(neighbor) == visited.end()) {
//                 toVisit.push(neighbor);
//                 visited.insert(neighbor);
//             }
//         }
//     }

//     return true; // If there's no escape route, it's enclosed
// }




ImmobilizedParticleSystem::ImmobilizedParticleSystem(int numParticles, int numImmoParticles, int genExpExample, int numCoinFlips) {
    Q_ASSERT(numParticles > 0);
    Q_ASSERT(numImmoParticles >= 0);
    Q_ASSERT(genExpExample == 0 || genExpExample == 1);
    Q_ASSERT(numCoinFlips > 0);

    std::set<Node> occupied;
    std::set<Node> candidates;

    _seedOrientation = randDir();
    Immobilizedparticles* leader = new Immobilizedparticles(Node(0, 0), -1, seedOrientation(), *this, Immobilizedparticles::State::Leader);
    insert(leader);
    occupied.insert(Node(0, 0));
    numParticles--;

    for (int i = 0; i < 6; ++i) {
        candidates.insert(Node(0, 0).nodeInDir(i));
    }

    while (numParticles > 0 || numImmoParticles > 0) {
        int randIndex = randInt(0, candidates.size());
        auto it = candidates.begin();
        std::advance(it, randIndex);
        Node randomCandidate = *it;

        if (randBool((double) numParticles / ((double) numParticles + (double) numImmoParticles))) {
            numParticles--;
            insert(new Immobilizedparticles(randomCandidate, -1, randDir(), *this, Immobilizedparticles::State::Idle));
        } else {
            // Avoid adding an immobilized particle if it is enclosed by non-immobilized particles
            bool isEnclosed = true;
            for (int i = 0; i < 6; ++i) {
                Node neighbor = randomCandidate.nodeInDir(i);
                if (occupied.find(neighbor) == occupied.end()) {
                    isEnclosed = false;
                    break;
                }
            }

            if (!isEnclosed) {  // Only insert if not enclosed by non-immobilized particles
                numImmoParticles--;
                insert(new ImmoParticle(randomCandidate));
            }
        }

        occupied.insert(randomCandidate);
        candidates.erase(it);

        for (int i = 0; i < 6; ++i) {
            Node neighbor = randomCandidate.nodeInDir(i);
            if (occupied.find(neighbor) == occupied.end()) {
                candidates.insert(neighbor);
            }
        }
    }
}


bool ImmobilizedParticleSystem::hasTerminated() const {
#ifdef QT_DEBUG
    if (!isConnected(particles)) {
        return true;
    }
#endif

    bool leaderExists = false;
    for (auto p : particles) {
        auto hp = dynamic_cast<Immobilizedparticles*>(p);
        if (hp->isInState({Immobilizedparticles::State::Idle}) || !hp->freeState || !hp->lineState || hp->isExpanded()) {
            return false;
        }
        if (hp->isInState({Immobilizedparticles::State::Leader})) {
            leaderExists = true;
        }
    }

    return leaderExists;
}

