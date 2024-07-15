
#include <set>
#include <queue>
#include <unistd.h>
#include <random>
#include "immobilizedparticles.h"
#include <QtGlobal>
#include <QOpenGLFunctions_2_0>
#include <vector>
#include <cstdio>

Immobilizedparticles::Immobilizedparticles(const Node head,
                                           const int globalTailDir,
                                           const int orientation,
                                           AmoebotSystem& system,
                                           State state)
    : AmoebotParticle(head, globalTailDir, orientation, system),
    state(state),
    moveDir(-1),
    followDir(-1),
    constructionDir(-1),
    leaderToken(false),
    tokenForwardDir(-1),
    tokenCurrentDir(-1),
    passForward(false),
    freeState(false),
    lineState(false),
    _parentDir(-1),
    _hexagonDir(state == State::Leader? 0 : -1),
    _borderColorsSet(false)
{
    _borderColors.fill(-1);
    _borderPointColors.fill(-1);
}


void Immobilizedparticles::activate() {
    printf("Activating particle. Current state: %d\n", state);
    if(isInState({State::FollowerHex,State::Lead,State::Seed, State::Finish})){
        activateHex();
    }
    try {
        if (leaderToken) {
            printf("Particle has received the leader token.\n");
            tryToBecomeLeader();
            return;
        } else if (isInState({State::Idle})) {
            printf("Particle is idle, trying to follow a neighbor.\n");

            bool hasLeaderOrFollowerNeighbor = false;

            for (int label : randLabels()) {
                if (hasNbrAtLabel(label)) {
                    auto& nbr = nbrAtLabel(label);
                    printf("Follow dir: Neighbour:: %d\n", nbr.followDir);

                    if ((nbr.isInState({State::Follower}) && !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir)))
                        || (nbr.isInState({State::Leader}) && (nbr.moveDir < 0 || !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.moveDir))))) {
                        followDir = labelToDir(label);
                        state = State::Follower;
                        printf("Following neighbor at label: %d\n", label);
                        return;
                    }else if (nbr.isInState({State::Leader})) {
                        // Handle the case where the neighbor is a leader
                        followDir = labelToDir(label); // Set the follow direction to the direction of the leader
                        state = State::Follower; // Change the particle's state to follower
                        nbr.moveDir = followDir; // Set the direction of the leader to the direction of the follower
                        return;}

                    else if (nbr.isInState({State::Leader}) || nbr.isInState({State::Follower})) {
                        hasLeaderOrFollowerNeighbor = true;
                    }
                }
            }

            if (!hasLeaderOrFollowerNeighbor) {
                for (int label : randLabels()) {
                    if (hasNbrAtLabel(label)) {
                        auto& nbr = nbrAtLabel(label);
                if (nbr.isInState({State::Immo}) || nbr.isInState({State::Idle}) || nbr.isInState({State::Cluster})) {
                    state = State::Cluster;
                    printf("No leader or follower neighbor found, changing state to Cluster.\n");
                    break;
                }
                    }
                }



            }
        } else if (isInState({State::Leader})) {
            if (moveDir == -1) {
                moveDir = randDir();
            }
            if (!areAllClusterAndIdleParticlesFollowers()) {
                printf("Waiting for all Cluster and Idle particles to become Followers.\n");

            } else {

                if (isContracted() && !canExpand(dirToHeadLabel(moveDir))) {
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
                        int label = labelOfFirstNbrInState({State::Idle, State::Follower});
                        if (label >= 0) {
                            passLeaderToken(label);
                            moveDir = -1;
                        } else {
                            // This can only happen if the Leader is the only particle and it is
                            // surrounded by objects.
                            // So basically never, unless you are testing weird edge cases.
                        }
                    }
                }
            }
        }
            else if (isInState({State::Immo})) {
            printf("Particle is immobile. Forwarding communication without changing state.\n");
            for (int label : randLabels()) {
                if (hasNbrAtLabel(label)) {
                    auto& nbr = nbrAtLabel(label);
                    if (nbr.isInState({State::Follower, State::Leader, State::Idle, State::Cluster})) {
                         printf("Immo :: Communication forwarded to neighbor: Not follower? %d\n", areAllClusterAndIdleParticlesFollowers());
                        if(!areAllClusterAndIdleParticlesFollowers()){
                        nbr.activate();
                        printf("Immo :: Communication forwarded to neighbor at label: %d\n", label);
                        }
                    }
                }
            }
        } else if (isInState({State::Cluster})) {
            printf("Particle is Cluster. Forwarding communication without changing state.\n");
            for (int label : randLabels()) {
                if (hasNbrAtLabel(label)) {
                    auto& nbr = nbrAtLabel(label);

                    if ((nbr.isInState({State::Follower}) && !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir)))
                        || (nbr.isInState({State::Leader}) && (nbr.moveDir < 0 || !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.moveDir))))) {
                        followDir = labelToDir(label);
                        state = State::Follower;
                        printf("Following neighbor at label - Try 2: %d\n", label);
                        return;
                    }
                    else if (nbr.isInState({State::Cluster, State::Immo})) {
                        printf("Performing Movement 2\n");
                        performMovement2();
                        if (state != State::Cluster) {
                            return;
                        }
                    }
                }
            }
        }


        if (!isInState({State::Idle, State::Immo, State::Cluster, State::Root, State::Retired, State::Seed, State::FollowerHex})) {
            performMovement();
        }
        // auto* immobilizedSystem = dynamic_cast<ImmobilizedParticleSystem*>(&system);
        // if (immobilizedSystem) {
        //     immobilizedSystem->printAllParticleStates();
        // }
    } catch (const std::exception& e) {
        printf("Exception occurred: %s\n", e.what());
    } catch (...) {
        printf("Unknown exception occurred.\n");
    }
    updateBoolStates();
    updateBorderColors();

}

void Immobilizedparticles::activateHex() {
    // alpha_1: idle or follower particles with a seed or retired neighbor become
    // roots and begin traversing the hexagon's surface.
    if(isInState({State::Leader})){
        state = State::Seed;
    }
    if(isInState({State::Follower})){
        state = State::FollowerHex;
    }

    if (isExpanded()) {
        if (state == State::FollowerHex) {
            if (!hasNbrInState({State::Idle}) && !hasTailFollower()) {
                contractTail();
            }
            return;
        } else if (state == State::Lead) {
            if (!hasNbrInState({State::Idle}) && !hasTailFollower()) {
                contractTail();
                updateMoveDir();
            }
            return;
        } else {
            Q_ASSERT(false);
        }
    } else {
        if (state == State::Seed) {
            return;
        } else if (state == State::Idle) {
            if (hasNbrInState({State::Seed, State::Finish})) {
                state = State::Lead;
                updateMoveDir();
                return;
            } else if (hasNbrInState({State::Lead, State::FollowerHex})) {
                state = State::FollowerHex;
                followDir = labelOfFirstNbrInState({State::Lead, State::FollowerHex});
                return;
            }
        } else if (state == State::FollowerHex) {
            if (hasNbrInState({State::Seed, State::Finish})) {
                state = State::Lead;
                updateMoveDir();
                return;
            } else if (hasTailAtLabel(followDir)) {
                auto nbr = nbrAtLabel(followDir);
                int nbrContractionDir = nbrDirToDir(nbr, (nbr.tailDir() + 3) % 6);
                push(followDir);
                followDir = nbrContractionDir;
                return;
            }
        } else if (state == State::Lead) {
            if (canFinish())  {
                state = State::Finish;
                updateConstructionDir();
                return;
            } else {
                updateMoveDir();
                if (!hasNbrAtLabel(moveDir)) {
                    expand(moveDir);
                } else if (hasTailAtLabel(moveDir)) {
                    push(moveDir);
                }
                return;
            }
        }
    }
}

bool Immobilizedparticles::hasTailFollower() const {
    auto prop = [&](const Immobilizedparticles& p) {
        return p.state == State::FollowerHex &&
               pointsAtMyTail(p, p.dirToHeadLabel(p.followDir));
    };

    return labelOfFirstNbrWithProperty<Immobilizedparticles>(prop) != -1;
}

int Immobilizedparticles::constructionReceiveDir() const {
    auto prop = [&](const Immobilizedparticles& p) {
        // Debugging: Print the current state and direction of the neighbor
        printf("Checking neighbor with state: %d, constructionDir: %d\n", p.state, p.constructionDir);

        // Check if the constructionDir is valid
        if (p.constructionDir < 0 || p.constructionDir >= 10) {
            printf("Invalid constructionDir: %d\n", p.constructionDir);
            return false;
        }

        // Check if the particle is in a contracted state and has the appropriate neighbor state
        bool isNeighborPointingAtMe = pointsAtMe(p, p.dirToHeadLabel(p.constructionDir));
        bool validNeighborState = (p.state == State::Seed || p.state == State::Finish);

        printf("isContracted: %d, isNeighborPointingAtMe: %d, validNeighborState: %d\n",
               isContracted(), isNeighborPointingAtMe, validNeighborState);

        return isContracted() && validNeighborState && isNeighborPointingAtMe;
    };

    int result = labelOfFirstNbrWithProperty<Immobilizedparticles>(prop);

    // Debugging: Print the result
    printf("constructionReceiveDir result: %d\n", result);

    return result;
}

bool Immobilizedparticles::canFinish() const {
    return constructionReceiveDir() != -1;
}

void Immobilizedparticles::updateConstructionDir() {
    // Hexagon construction.
    constructionDir = constructionReceiveDir();
    printf("Initial constructionDir: %d\n", constructionDir);

    if (constructionDir < 0 || constructionDir >= 10) {
        printf("Invalid constructionDir after constructionReceiveDir: %d\n", constructionDir);
        constructionDir = 0; // Or some valid fallback
    }

    // Adjust constructionDir based on neighbor states
    if (nbrAtLabel(constructionDir).state == State::Seed) {
        constructionDir = (constructionDir + 1) % 6;
    } else {
        constructionDir = (constructionDir + 2) % 6;
    }

    if (hasNbrAtLabel(constructionDir) &&
        nbrAtLabel(constructionDir).state == State::Finish) {
        constructionDir = (constructionDir + 1) % 6;
    }

    printf("Final constructionDir: %d\n", constructionDir);
}


int Immobilizedparticles::nextClockwiseDir(int inputDir) {
    return (inputDir + 1) % 6;
}

int Immobilizedparticles::nextCounterclockwiseDir(int inputDir) {
    return (inputDir - 1 + 6) % 6;
}

void Immobilizedparticles::tryToBecomeLeader() {
    printf("Trying to become leader.\n");
    for (int label : randLabels()) {
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
            updateBorderPointColors();
            printf("Became leader. New move direction: %d\n", moveDir);
            return;
        }
    }

    moveDir = -1;
    printf("Could not become leader, passing on leader token.\n");

    int label = 0;
    for (int l : uniqueLabels()) {
        if (labelToDir(l) == tokenCurrentDir) {
            label = l;
        }
    }

    int numLabels = isContracted() ? 6 : 10;

    if (passForward) {
        while (hasObjectAtLabel(label)) {
            label = (label + 1) % numLabels;
            passForward = false;
        }
    } else {
        do {
            label = (label - 1 + numLabels) % numLabels;
        } while (!hasObjectAtLabel(label));
        while (hasObjectAtLabel(label)) {
            label = (label + 1) % numLabels;
        }
    }

    passLeaderToken(label);
}

void Immobilizedparticles::passLeaderToken(const int label) {
    if (isExpanded()) {
        makeHeadLabel(label);
    }

    state = State::Follower;
    followDir = labelToDir(label);
    if (tokenForwardDir < 0) {
        passForward = true;
        tokenForwardDir = followDir;
    }

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
    nbr.updateBorderPointColors();

    leaderToken = false;
    tokenForwardDir = -1;
    tokenCurrentDir = -1;
    passForward = false;
    updateBorderPointColors();

    printf("Passed leader token to neighbor at label: %d\n", label);
}

void Immobilizedparticles::performMovement() {
    if (!areAllClusterAndIdleParticlesFollowers()) {
        printf("Waiting for all Cluster and Idle particles to become Followers.\n");

    }
    else
    {
    if (isExpanded() && isInState({State::Follower, State::Leader}) && !hasBlockingTailNbr()) {
        printf("Contracting tail.\n");
        contractTail();
    } else if (isContracted() && isInState({State::Follower}) && hasTailAtLabel(dirToHeadLabel(followDir))) {
        printf("Expanding towards follow direction: %d\n", followDir);
        int followLabel = dirToHeadLabel(followDir);
        auto& nbr = nbrAtLabel(followLabel);
        int nbrContractionDir = nbrDirToDir(nbr, (nbr.tailDir() + 3) % 6);
        push(followLabel);
        followDir = nbrContractionDir;
    } else if (isInState({State::Leader}) && isContracted() && !(freeState && lineState) && canExpand(dirToHeadLabel(moveDir))) {
        printf("Expanding towards move direction: %d\n", moveDir);
        expand(dirToHeadLabel(moveDir));
    }else{
        printf(" I am dead");
    }
    updateBoolStates();
    updateBorderColors();
    printf("Performed movement.\n");
    }
}

bool Immobilizedparticles::areAllClusterAndIdleParticlesFollowers() {
    auto* immobilizedSystem = dynamic_cast<ImmobilizedParticleSystem*>(&system);
    if (!immobilizedSystem) {
        return false;
    }

    for (const auto& particle : immobilizedSystem->getParticles()) {
        auto* immobileParticle = dynamic_cast<Immobilizedparticles*>(particle);
        if (immobileParticle) {
            if (immobileParticle->isInState({State::Cluster, State::Idle})) {
                return false;
            }

        }
    }
    return true;
}

void Immobilizedparticles::updateMoveDir() {
    moveDir = labelOfFirstNbrInState({State::Seed, State::Finish});
    while (hasNbrAtLabel(moveDir) && (nbrAtLabel(moveDir).state == State::Seed ||
                                      nbrAtLabel(moveDir).state == State::Finish))
    {
        moveDir = (moveDir + 5) % 6;
    }
}






void Immobilizedparticles::performMovement2() {
    State originalState = state;
    Node originalHead = head;
    const std::vector<int>& headLabels = this->headLabels();

    // Set up the random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, headLabels.size() - 1);

    const int maxIterations = 100; // Maximum number of iterations to prevent infinite loop
    int iterationCount = 0;

    while (iterationCount < maxIterations) {
        iterationCount++;

        // Randomly choose a direction from the head labels
        int randomIndex = dis(gen);
        int elem = headLabels[randomIndex];
        printf("Randomly chosen head label: %d\n", elem);

        // Check if the randomly chosen direction is free
        if (!hasNbrAtLabel(elem) && !hasObjectAtLabel(elem)) {
            expand(elem);
            printf("Expanding to randomly chosen free slot at label: %d\n", elem);

            bool connected = false;

            const std::vector<int>& headLabelsNbr = this->headLabels();
            for (int elem2 : headLabelsNbr) {
                printf("Checking neighbor at head label: %d\n", elem2);
                if (hasNbrAtLabel(elem2)) {
                    auto& nbr = nbrAtLabel(elem2);
                    if ((nbr.isInState({State::Follower}) && !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir))) ||
                        (nbr.isInState({State::Leader}) && (nbr.moveDir < 0 || !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.moveDir))))) {
                        followDir = labelToDir(elem2);
                        state = State::Follower;
                        connected = true;
                        printf("Following neighbor at label: %d\n", elem);
                        break;
                    } else if (nbr.isInState({State::Leader, State::Follower})) {
                        connected = true;
                        followDir = labelToDir(elem2);
                        //followDir = labelOfFirstNbrInState({State::Leader, State::Follower});
                        printf("Still connected to leader/follower.\n");
                        break;
                    } else if (nbr.isInState({State::Cluster})) {
                        connected = false;
                        // if (isExpanded()) {
                        //     contractHead();
                        // }
                        // state = originalState;
                        // head = originalHead;
                        // nbr.performMovement2();
                    } else {
                        connected = true;
                        followDir = labelOfFirstNbrInState({State::Leader, State::Follower});
                        printf("Still connected to leader/follower.\n");
                    }
                }
            }

            if (connected) {
                contractTail();
                printf("Expansion successful. Contracting tail.\n");
                break;
            } else {
                printf("Expansion to label: %d would disconnect, backtracking.\n", elem);
                if (isExpanded()) {
                    contractHead();

                }
                state = originalState;
                head = originalHead;
                printf("Backtracking to original state.\n");
            }
        }

        if (isInState({State::Cluster})) {
            continue;
        }
    }

    if (iterationCount >= maxIterations) {
        printf("Maximum iterations reached. Exiting to prevent infinite loop.\n");
    }

    updateBoolStates();
    updateBorderColors();
    printf("Performed movement 2.\n");
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
    if (hasObjectNbr() || hasNbrInState({State::Idle}) || hasNbrInState({State::Cluster})) {
        freeState = false;
        lineState = false;
        printf("Free state is false.\n");
        return;
    }

    freeState = true;
    for (int label : uniqueLabels()) {
        if (hasNbrAtLabel(label)) {
            auto& nbr = nbrAtLabel(label);
            if (nbr.isInState({State::Follower}) && pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir))) {
                freeState = freeState && nbr.freeState;
                printf("Neighbor is a follower. Free state updated to %d.\n", freeState);
            }
        }
    }


    // Step 2: Update lineState

    // The lineState only becomes relevant when the particle is free from all objects.
    if (!freeState) {
        lineState = false;
        printf("Line state is false.\n");
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
            printf("Parent direction does not match follow direction. Line state is false.\n");
            return;
        }
    }

    lineState = true;
    for (int label : uniqueLabels()) {
        if (hasNbrAtLabel(label)) {
            auto& nbr = nbrAtLabel(label);
            if (nbr.isInState({State::Follower}) && pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir))) {
                lineState = lineState && nbr.lineState;
                printf("Neighbor is a follower. Line state updated to %d.\n", lineState);
            }
        }
    }
}

std::vector<int> Immobilizedparticles::randLabels() {
    std::vector<int> result = uniqueLabels();
    RandomNumberGenerator::shuffle(std::begin(result), std::end(result));
    return result;
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
        case State::Immo:     return "Immo";
        case State::Cluster:  return "Cluster";
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


void Immobilizedparticles::setState(Immobilizedparticles::State newState) {
    state = newState;
}
void ImmobilizedParticleSystem::printAllParticleStates() const{
        for (const auto& particle : particles) {
            auto immobileParticle = dynamic_cast<const Immobilizedparticles*>(particle);
            if (immobileParticle) {
                printf("%s\n", immobileParticle->inspectionText().toStdString().c_str());
            }
        }
    }

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
            insert(new Immobilizedparticles(randomCandidate, -1,1, *this, Immobilizedparticles::State::Idle));
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
                Immobilizedparticles* immoParticle = new Immobilizedparticles(randomCandidate, -1,1, *this, Immobilizedparticles::State::Immo);
                insert(immoParticle);
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

// bool ImmobilizedParticleSystem::hasTerminated() const {
// #ifdef QT_DEBUG
//     if (!isConnected(particles)) {
//         return true;
//     }
// #endif

//     bool terminationConditionMet = false;

//     // This process repeats until all particles are in the Retired or Seed states.
//     while (!terminationConditionMet) {
//         terminationConditionMet = true;
//         for (auto p : particles) {
//             auto hp = dynamic_cast<Immobilizedparticles*>(p);
//             // Check if particle is in an invalid state that prevents termination
//             if (hp->isInState({Immobilizedparticles::State::Idle}) ||
//                 hp->isInState({Immobilizedparticles::State::Cluster}) ||
//                 !hp->freeState || !hp->lineState || hp->isExpanded()) {
//                 return false;  // Not terminated
//             }
//             // Check if particle is in a state that requires further processing
//             if (hp->isInState({Immobilizedparticles::State::Leader}) ||
//                 hp->isInState({Immobilizedparticles::State::Follower}) ||
//                 hp->isInState({Immobilizedparticles::State::FollowerHex}) ||
//                 hp->isInState({Immobilizedparticles::State::Root})) {
//                 terminationConditionMet = false;
//             }
//         }

//         if (!terminationConditionMet) {
//             for (auto p : particles) {
//                 auto hp = dynamic_cast<Immobilizedparticles*>(p);
//                 // Check if particle is in a state that requires activation
//                 if (hp->isInState({Immobilizedparticles::State::Leader}) ||
//                     hp->isInState({Immobilizedparticles::State::Follower}) ||
//                     hp->isInState({Immobilizedparticles::State::FollowerHex}) ||
//                     hp->isInState({Immobilizedparticles::State::Root})) {
//                     // Call activateHex() specifically for these states
//                     //hp->activateHex();
//                 }
//             }
//         }
//     }

//     return true;  // All particles are in Retired or Seed states
// }


bool ImmobilizedParticleSystem::hasTerminated() const {
#ifdef QT_DEBUG
    if (!isConnected(particles)) {
        return true;
    }
#endif

    bool leaderExists = false;
    for (auto p : particles) {
        auto hp = dynamic_cast<Immobilizedparticles*>(p);
        if (hp->isInState({Immobilizedparticles::State::Idle}) || hp->isInState({Immobilizedparticles::State::Cluster}) || !hp->freeState || !hp->lineState || hp->isExpanded()) {
            return false;
        }
        if(hp->isInState({Immobilizedparticles::State::Leader}) || hp->isInState({Immobilizedparticles::State::Follower})){

                if (hp->isInState({Immobilizedparticles::State::Leader})) {
                    leaderExists = true;

                }
               //hp->activateHex();
            }


}

if (leaderExists) {
    for (auto p : particles) {
        auto hp = dynamic_cast<Immobilizedparticles*>(p);
        if (hp->isInState({Immobilizedparticles::State::Leader}) || hp->isInState({Immobilizedparticles::State::Follower})) {
            hp->activateHex();
        }
    }
}

    return leaderExists;
}



//Particle Apperance

int Immobilizedparticles::particleColor() const {
    // The inner color of a particle.
    if (leaderToken) {
        return 0xff8800; // orange
    }
    else if(isInState({State::Immo})){
            return 0xFF0000;


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
    case State::Immo:         return 0xFF0000; //red
    case State::Cluster:      return 0x39FF14; // Fluorescent Green
    case State::FollowerHex:       return 0x00FFEF; // Fluorescent Blue
    case State::ClusterLead:  return 0xFF1493; // Fluorescent PinkGreen
    case State::ClusterMark:  return 0x00FFEF;
    case State::Seed:      return 0xFF1493; //pink
    case State::Root:      return 0xff0000;  // red
    case State::Retired:   return 0x000000;  // black
    case State::Finish:   return 0x000000; //black
    case State::Lead:   return 0xff0000; //red
    case State::Idle:   return 0xff8800;
            // Fluorescent Blue
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






int Immobilizedparticles::nextHexagonDir(int orientation) const {
    // First, find a head label that points to a seed or retired neighbor.
    int hexagonLabel;
    for (int label : headLabels()) {
        if (hasNbrAtLabel(label)
            && (nbrAtLabel(label).state == State::Seed
                || nbrAtLabel(label).state == State::Retired)) {
            hexagonLabel = label;
            break;
        }
    }

    // Next, find the label that points along the hexagon's surface in a traversal
    // with the specified orientation. Perhaps counterintuitively, this means that
    // we search from the above label in the opposite orientation for the first
    // unoccupied or non-seed/retired neighbor.
    int numLabels = isContracted() ? 6 : 10;
    while (hasNbrAtLabel(hexagonLabel)
           && (nbrAtLabel(hexagonLabel).state == State::Seed
               || nbrAtLabel(hexagonLabel).state == State::Retired))
        hexagonLabel = (hexagonLabel + orientation + numLabels) % numLabels;

    // Convert this label to a direction before returning.
    return labelToDir(hexagonLabel);
}

bool Immobilizedparticles::canRetire() const {
    auto prop = [&](const Immobilizedparticles& p) {
        printf("Inside Can Retire");
        return (p.state == State::Seed || p.state == State::Retired)
               && pointsAtMe(p, p._hexagonDir);
    };
    printf("Outside Can Retire");

    return labelOfFirstNbrWithProperty<Immobilizedparticles>(prop) != -1;
}

bool Immobilizedparticles::hasTailChild() const {
    auto prop = [&](const Immobilizedparticles& p) {
        return p._parentDir != -1
               && pointsAtMyTail(p, p.dirToHeadLabel(p._parentDir));
    };

    return labelOfFirstNbrWithProperty<Immobilizedparticles>(prop) != -1;
}

const std::vector<int> Immobilizedparticles::conTailChildLabels() const {
    std::vector<int> labels;
    for (int label : tailLabels())
        if (hasNbrAtLabel(label)
            && nbrAtLabel(label).isContracted()
            && nbrAtLabel(label)._parentDir != -1
            && pointsAtMyTail(nbrAtLabel(label), nbrAtLabel(label)._parentDir))
            labels.push_back(label);

    return labels;
}
