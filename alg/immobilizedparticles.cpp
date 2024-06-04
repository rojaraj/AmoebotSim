#include <set>
#include <queue>
#include <unistd.h>
#include "immobilizedparticles.h"
#include <QtGlobal>
#include <QOpenGLFunctions_2_0>

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
    printf("Activating particle. Current state: %d\n", state);

    try {
        if (leaderToken) {
            printf("Particle has received the leader token.\n");
            tryToBecomeLeader();
            return;
        } else if (isInState({State::Idle})) {
            printf("Particle is idle, trying to follow a neighbor.\n");

            for (int label : randLabels()) {
                if (hasNbrAtLabel(label)) {
                    auto& nbr = nbrAtLabel(label);
                    if ((nbr.isInState({State::Follower}) && !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir)))
                        || (nbr.isInState({State::Leader}) && (nbr.moveDir < 0 || !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.moveDir))))) {
                        followDir = labelToDir(label);
                        state = State::Follower;
                        printf("Following neighbor at label: %d\n", label);
                        break;
                    }
                    else if (nbr.isInState({State::Immo}) || nbr.isInState({State::Idle}) || nbr.isInState({State::Cluster})) {
                        state = State::Cluster;
                        break;
                    }
                }
            }
        } else if (isInState({State::Leader})) {
            printf("Particle is a leader.\n");

            if (moveDir == -1) {
                moveDir = randDir();
                printf("Random move direction chosen: %d\n", moveDir);
            }

            if (!areAllClusterAndIdleParticlesFollowers()) {
                printf("Waiting for all Cluster and Idle particles to become Followers.\n");
                return;
            }

            if (isContracted() && !canExpand(dirToHeadLabel(moveDir))) {
                printf("Cannot expand in current move direction: %d\n", moveDir);
                bool changed = false;
                for (int label : randLabels()) {
                    moveDir = labelToDir(label);
                    if (!hasNbrAtLabel(label) && !hasObjectAtLabel(label)) {
                        if (isExpanded()) {
                            makeHeadLabel(label);
                        }
                        changed = true;
                        printf("Changed move direction to: %d\n", moveDir);
                        break;
                    }
                }
                if (!changed) {
                    int label = labelOfFirstNbrInState({State::Idle, State::Follower});
                    if (label >= 0) {
                        passLeaderToken(label);
                        moveDir = -1;
                        printf("Passed leader token to label: %d\n", label);
                    } else {
                        printf("Leader is surrounded by ImmoParticles\n");
                    }
                }
            }
        } else if (isInState({State::Immo})) {
            printf("Particle is immobile. Forwarding communication without changing state.\n");
            for (int label : randLabels()) {
                if (hasNbrAtLabel(label)) {
                    auto& nbr = nbrAtLabel(label);
                    if (nbr.isInState({State::Follower, State::Leader, State::Idle})) {
                        nbr.activate();
                        printf("Communication forwarded to neighbor at label: %d\n", label);
                    }
                }
            }
        } else if (isInState({State::Cluster})) {
            printf("Particle is Cluster. Forwarding communication without changing state.\n");
            for (int label : randLabels()) {
                if (hasNbrAtLabel(label)) {
                    auto& nbr = nbrAtLabel(label);
                    if (nbr.isInState({State::Cluster, State::Immo}) && isInState({State::Cluster})) {
                        performMovement2();
                        if (state != State::Cluster) {
                            break;
                        }
                    }

                    if ((nbr.isInState({State::Follower}) && !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir)))
                        || (nbr.isInState({State::Leader}) && (nbr.moveDir < 0 || !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.moveDir))))) {
                        followDir = labelToDir(label);
                        state = State::Follower;
                        printf("Following neighbor at label - Try 2: %d\n", label);
                        break;
                    }
                }
            }
        }

        if (!isInState({State::Idle, State::Immo, State::Cluster})) {
            performMovement();
        }
        auto* immobilizedSystem = dynamic_cast<ImmobilizedParticleSystem*>(&system);
        if (immobilizedSystem) {
            immobilizedSystem->printAllParticleStates();
        }
    } catch (const std::exception& e) {
        printf("Exception occurred: %s\n", e.what());
    } catch (...) {
        printf("Unknown exception occurred.\n");
    }
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
    }
    updateBoolStates();
    updateBorderColors();
    printf("Performed movement.\n");
}

bool Immobilizedparticles::areAllClusterAndIdleParticlesFollowers() {
    auto* immobilizedSystem = dynamic_cast<ImmobilizedParticleSystem*>(&system);
    if (!immobilizedSystem) {
        return false;
    }

    for (const auto& particle : immobilizedSystem->getParticles()) {
        auto* immobileParticle = dynamic_cast<Immobilizedparticles*>(particle);
        if (immobileParticle) {
            if (immobileParticle->isInState({State::Cluster, State::Idle}) && !immobileParticle->isInState({State::Follower})) {
                return false;
            }
        }
    }
    return true;
}






void Immobilizedparticles::performMovement2() {
    State originalState = state;
    Node originalHead = head;

    int dir = 0;

    int iterationCount = 0; // Counter variable to limit iterations

    while (iterationCount < 12) { // Limit iterations to avoid infinite loops
        int label = dirToHeadLabel(dir);

        // Ensure the label is valid before proceeding
        if (label < 0 || label >= 6) {
            dir = nextClockwiseDir(dir);
            iterationCount++;
            continue;
        }

        if (!hasNbrAtLabel(label) && !hasObjectAtLabel(label)) {
            moveDir = dir;
            if (canExpand(moveDir)) {
                expand(moveDir);
                printf("Expanding in clockwise direction to label: %d\n", label);

                bool connected = false;
                int checkDir = nextCounterclockwiseDir(dir);

                const std::vector<int>& headLabels = this->headLabels();
                for (int elem : headLabels) {
                    printf("%d ", elem);
                    if (hasNbrAtLabel(elem)) {
                        auto& nbr = nbrAtLabel(elem);
                        if ((nbr.isInState({State::Follower}) && !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir)))
                            || (nbr.isInState({State::Leader}) && (nbr.moveDir < 0 || !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.moveDir))))) {
                            followDir = labelToDir(label);
                            state = State::Follower;
                            printf("Following neighbor at label: %d\n", label);
                            break;
                        }
                        else if (nbr.isInState({State::Leader, State::Follower})){
                            connected = true;
                            printf("Still Connected :D\n");
                            break;
                        }
                        else {
                            connected = true;
                            printf("Still Connected :D\n");
                            //dir=0;

                        }
                    }
                }
                printf("\n");

                if (connected) {
                    contractTail();
                    printf("Expansion successful. Contracting tail.\n");
                    break;
                } else {
                    printf("Expansion to label: %d would disconnect, backtracking.\n", label);
                    if (isExpanded()) {
                        contractHead();
                        // Update the particle's state and position
                    }
                    state = originalState;
                    head = originalHead;

                    printf("Backtracking.%d\n", headMarkDir());
                }
            }
        }
        dir = randDir();
        iterationCount++;
        printf("Code reached here.\n");
    }
    updateBoolStates();
    updateBorderColors();
    printf("Performed movement 2.\n");
}



// #include <set>
// #include <queue> // Include this to use std::queue
// #include <unistd.h> // For sleep function
// #include "immobilizedparticles.h"
// #include <QtGlobal>
// #include <QOpenGLFunctions_2_0>



// Immobilizedparticles::Immobilizedparticles(const Node head,
//                                            const int globalTailDir,
//                                            const int orientation,
//                                            AmoebotSystem& system,
//                                            State state)
//     : AmoebotParticle(head, globalTailDir, orientation, system),
//     state(state),
//     moveDir(-1),
//     followDir(-1),
//     leaderToken(false),
//     tokenForwardDir(-1),
//     tokenCurrentDir(-1),
//     passForward(false),
//     freeState(false),
//     lineState(false),
//     _borderColorsSet(false)
// {
//     _borderColors.fill(-1);
//     _borderPointColors.fill(-1);
// }

// void Immobilizedparticles::activate() {


//     printf("Activating particle. Current state: %d\n", state);


//     if (leaderToken) {
//         printf("Particle has received the leader token.\n");

//         tryToBecomeLeader();
//         return;
//     } else if (isInState({State::Idle})) {
//         printf("Particle is idle, trying to follow a neighbor.\n");
//             //comment
//         for (int label : randLabels()) {
//             if (hasNbrAtLabel(label)) {
//                 auto& nbr = nbrAtLabel(label);
//                 if ((nbr.isInState({State::Follower}) && !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir)))
//                     || (nbr.isInState({State::Leader}) && (nbr.moveDir < 0 || !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.moveDir))))) {
//                     //nbr.dirToHeadLabel(nbr.followDir): Gets the label in the direction the neighbor is following.
//                     //pointsAtMe(nbr, ...): Checks if the given label direction points at the current particle.
//                     followDir = labelToDir(label);
//                     state = State::Follower;
//                     printf("Following neighbor at label: %d\n", label);
//                         //comment
//                     break;
//                 }
//                 else if (nbr.isInState({State::Immo})|| nbr.isInState({State::Idle}))  {
//                     state = State::Cluster;
//                     break;
//                 }
//                 else if (nbr.isInState({State::Cluster}))  {
//                     state = State::Cluster;
//                     break;
//                 }
//                 //     // Array of labels corresponding to clockwise directions
//                 //     std::vector<int> clockwiseLabels = {1, 2, 3, 4, 5, 0}; // Adjust this based on your specific label system

//                 //     // Iterate through each label in a clockwise manner
//                 //     for (int label : clockwiseLabels) {

//                 //         int dir = labelToDir(label);
//                 //         // Convert label to direction
//                 //         if (!hasNbrAtLabel(label) && !hasObjectAtLabel(label)) {
//                 //             moveDir = dir; // Set the move direction to the free position
//                 //             //int targetLabel = dirToHeadLabel(moveDir);
//                 //             if (canExpand(moveDir)) { // Ensure expansion is possible

//                 //                 // Backup current state in case we need to backtrack
//                 //                 auto originalState = state;
//                 //                 auto originalHead = head;

//                 //                 // Expand to the free position
//                 //                 expand(moveDir);
//                 //                 printf("Expanding in clockwise direction to label: %d\n", label);

//                 //                 bool connected = false;
//                 //                 for (int neighborLabel : clockwiseLabels) {
//                 //                     if ( hasNbrAtLabel(neighborLabel)) {
//                 //                         connected = true;
//                 //                         break;
//                 //                     }
//                 //                 }

//                 //                 // If the particle is still connected, contract the tail
//                 //                 if (connected) {
//                 //                     contractTail();
//                 //                     printf("Expansion successful. Contracting tail.\n");
//                 //                     break; // Exit the loop after finding the first free position
//                 //                 } else {
//                 //                     // Backtrack to original state if disconnected
//                 //                     printf("Expansion to label: %d would disconnect, backtracking.\n", label);

//                 //                     // Revert the expansion to original state
//                 //                     contractTail(); // Ensure the particle is in its initial state before re-expansion
//                 //                     printf("Contracted \n");
//                 //                     // Undo the expansion
//                 //                     int moveDirInverted = (dir + 3) % 6;

//                 //                     expand(moveDirInverted); // Expand back to the original position
//                 //                     break;
//                 //                     // Restore original state
//                 //                     state = originalState;
//                 //                     head = originalHead;
//                 //                 }
//                 //             }
//                 //         }
//                 //     }
//                 // }





//             }
//         }
//     } else if (isInState({State::Leader})) {
//         printf("Particle is a leader.\n");
//             //comment
//         if (moveDir == -1) {
//             moveDir = randDir();
//             printf("Random move direction chosen: %d\n", moveDir);
//                 //comment
//         }
//         if (isContracted() && !canExpand(dirToHeadLabel(moveDir))) {
//             printf("Cannot expand in current move direction: %d\n", moveDir);
//                 //comment
//             bool changed = false;
//             for (int label : randLabels()) {
//                 moveDir = labelToDir(label);
//                 if (!hasNbrAtLabel(label) && !hasObjectAtLabel(label)) {
//                     if (isExpanded()) {
//                         makeHeadLabel(label);
//                     }
//                     changed = true;
//                     printf("Changed move direction to: %d\n", moveDir);
//                         //comment
//                     break;
//                 }
//             }
//             if (!changed) {
//                 int label = labelOfFirstNbrInState({State::Idle, State::Follower});
//                 if (label >= 0) {
//                     passLeaderToken(label);
//                     moveDir = -1;
//                     printf("Passed leader token to label: %d\n", label);
//                         //comment
//                 } else {
//                     printf("Leader is surrounded by ImmoParticles\n");
//                         //comment
//                 }
//             }
//         }
//     }
//     else if (isInState({State::Immo})) {
//         printf("Particle is immobile. Forwarding communication without changing state.\n");
//         // Forward communication to neighbors without changing state
//         for (int label : randLabels()) {
//             if (hasNbrAtLabel(label)) {
//                 auto& nbr = nbrAtLabel(label);
//                 // Check if neighbor is suitable to forward communication
//                 if (nbr.isInState({State::Follower, State::Leader, State::Idle})) {
//                     // Forward communication to neighbor
//                     nbr.activate();
//                     printf("Communication forwarded to neighbor at label: %d\n", label);
//                     // Add any additional handling or logging here
//                 }
//                 // else if(nbr.isInState({State::Immo})){
//                 //     nbr.activate();
//                 //     break;

//                 // }

//             }

//         }

//     }
//     else if (isInState({State::Cluster})) {
//         printf("Particle is Cluster. Forwarding communication without changing state.\n");
//         // Forward communication to neighbors without changing state
//         for (int label : randLabels()) {
//             if (hasNbrAtLabel(label)) {
//                 auto& nbr = nbrAtLabel(label);
//                 if(!nbr.isInState({State::Cluster, State::Immo})){
//                     performMovement2();
//                     }
//                 }

//                 }
//             }

//     if (!isInState({State::Idle})) {
//         performMovement();
//     }
//     auto* immobilizedSystem = dynamic_cast<ImmobilizedParticleSystem*>(&system);
//     if (immobilizedSystem) {

//         immobilizedSystem->printAllParticleStates();
//     }
// }

// int Immobilizedparticles::nextClockwiseDir(int inputDir) {
//     return (inputDir + 1) % 6;
// }

// int Immobilizedparticles::nextCounterclockwiseDir(int inputDir) {
//     return (inputDir - 1 + 6) % 6;
// }


// void Immobilizedparticles::tryToBecomeLeader() {
//     printf("Trying to become leader.\n");
//         //comment
//     for (int label : randLabels()) {
//         moveDir = labelToDir(label);
//         if ((isExpanded() && !hasNbrAtLabel(label) && !hasObjectAtLabel(label)) || canExpand(labelToDir(label))) {
//             if (isExpanded()) {
//                 makeHeadLabel(label);
//             }
//             state = State::Leader;
//             leaderToken = false;
//             tokenForwardDir = -1;
//             tokenCurrentDir = -1;
//             passForward = false;
//             updateBorderPointColors(); // Only necessary for the visualisation.
//             printf("Became leader. New move direction: %d\n", moveDir);
//                 //comment
//             return;
//         }
//     }

//     moveDir = -1;
//     printf("Could not become leader, passing on leader token.\n");
//         //comment

//     int label = 0;
//     for (int l : uniqueLabels()) {
//         if (labelToDir(l) == tokenCurrentDir) {
//             label = l;
//         }
//     }

//     int numLabels = isContracted() ? 6 : 10;

//     if (passForward) {
//         while (hasObjectAtLabel(label)) {
//             label = (label + 1) % numLabels;
//             passForward = false;
//         }
//     } else {
//         do {
//             label = (label - 1 + numLabels) % numLabels;
//         } while (!hasObjectAtLabel(label));
//         while (hasObjectAtLabel(label)) {
//             label = (label + 1) % numLabels;
//         }
//     }

//     passLeaderToken(label);
// }

// void Immobilizedparticles::passLeaderToken(const int label) {
//     if (isExpanded()) {
//         makeHeadLabel(label);
//     }

//     state = State::Follower;
//     followDir = labelToDir(label);
//     if (tokenForwardDir < 0) {
//         passForward = true;
//         tokenForwardDir = followDir;
//     }

//     auto& nbr = nbrAtLabel(label);
//     nbr.leaderToken = true;
//     nbr.tokenForwardDir = dirToNbrDir(nbr, tokenForwardDir);
//     nbr.tokenCurrentDir = dirToNbrDir(nbr, followDir);
//     nbr.passForward = passForward;
//     if (nbr.tokenForwardDir == nbr.tokenCurrentDir && tokenForwardDir != tokenCurrentDir) {
//         if (randBool()) {
//             nbr.passForward = true;
//         }
//     }
//     nbr.updateBorderPointColors(); // Only necessary for the visualisation.

//     leaderToken = false;
//     tokenForwardDir = -1;
//     tokenCurrentDir = -1;
//     passForward = false;
//     updateBorderPointColors(); // Only necessary for the visualisation.

//     printf("Passed leader token to neighbor at label: %d\n", label);
//         //comment
// }

// void Immobilizedparticles::performMovement() {
//     if (isExpanded() && isInState({State::Follower, State::Leader}) && !hasBlockingTailNbr()) {
//         printf("Contracting tail.\n");
//             //comment
//         contractTail();
//     } else if (isContracted() && isInState({State::Follower}) && hasTailAtLabel(dirToHeadLabel(followDir))) {
//         printf("Expanding towards follow direction: %d\n", followDir);
//             //comment
//         int followLabel = dirToHeadLabel(followDir);
//         auto& nbr = nbrAtLabel(followLabel);
//         int nbrContractionDir = nbrDirToDir(nbr, (nbr.tailDir() + 3) % 6);
//         push(followLabel);
//         followDir = nbrContractionDir;
//     } else if (isInState({State::Leader}) && isContracted() && !(freeState && lineState) && canExpand(dirToHeadLabel(moveDir))) {
//         printf("Expanding towards move direction: %d\n", moveDir);
//             //comment
//         expand(dirToHeadLabel(moveDir));
//     }
//     updateBoolStates();
//     updateBorderColors(); // Only necessary for the visualisation.
//     printf("Performed movement.\n");
//         //comment
// }

// void Immobilizedparticles::performMovement2() {
//     // State originalState = state;
//     // Node originalHead = head;
//     // int dir = 0;  // Starting direction, can be any valid initial direction.

//     // for (int i = 0; i < 6; ++i) {
//     //     int label = dirToHeadLabel(dir);

//     //     if (!hasNbrAtLabel(label) && !hasObjectAtLabel(label)) {
//     //         moveDir = dir;
//     //         if (canExpand(moveDir)) {
//     //             expand(moveDir);
//     //             printf("Expanding in clockwise direction to label: %d\n", label);

//     //             bool connected = false;
//     //             int checkDir = dir;

//     //             for (int j = 0; j < 6; ++j) {
//     //                 int neighborLabel = dirToHeadLabel(checkDir);
//     //                 if (hasNbrAtLabel(neighborLabel)) {
//     //                     auto& nbr = nbrAtLabel(neighborLabel);
//     //                     if (nbr.isInState({State::Leader, State::Follower})) {
//     //                         state = State::Follower;
//     //                         followDir = checkDir;
//     //                         connected = true;
//     //                         printf("Connected to neighbor at label: %d and changed state to Follower.\n", neighborLabel);
//     //                         break;
//     //                     }
//     //                 }
//     //                 checkDir = nextClockwiseDir(checkDir);
//     //             }

//     //             if (connected) {
//     //                 contractTail();
//     //                 printf("Expansion successful. Contracting tail.\n");
//     //                 break;
//     //             } else {
//     //                 printf("Expansion to label: %d would disconnect, backtracking.\n", label);
//     //                 contractTail();
//     //                 int moveDirInverted = nextCounterclockwiseDir(nextCounterclockwiseDir(dir));
//     //                 expand(moveDirInverted);
//     //                 state = originalState;
//     //                 head = originalHead;
//     //             }
//     //         }
//     //     }
//     //     dir = nextClockwiseDir(dir);
//     // }

//     //updateBoolStates();
//     //updateBorderColors(); // Only necessary for the visualization.
//     printf("Performed movement 2.\n");
// }

// #include <set>
// #include <queue> // Include this to use std::queue
// #include <unistd.h> // For sleep function
// #include "immobilizedparticles.h"
// #include <QtGlobal>
// #include <QOpenGLFunctions_2_0>



// Immobilizedparticles::Immobilizedparticles(const Node head,
//                                            const int globalTailDir,
//                                            const int orientation,
//                                            AmoebotSystem& system,
//                                            State state)
//     : AmoebotParticle(head, globalTailDir, orientation, system),
//     state(state),
//     moveDir(-1),
//     followDir(-1),
//     leaderToken(false),
//     tokenForwardDir(-1),
//     tokenCurrentDir(-1),
//     passForward(false),
//     freeState(false),
//     lineState(false),
//     _borderColorsSet(false)
// {
//     _borderColors.fill(-1);
//     _borderPointColors.fill(-1);
// }

// void Immobilizedparticles::activate() {


//     printf("Activating particle. Current state: %d\n", state);


//     if (leaderToken) {
//         printf("Particle has received the leader token.\n");

//         tryToBecomeLeader();
//         return;
//     } else if (isInState({State::Idle})) {
//         printf("Particle is idle, trying to follow a neighbor.\n");
//          //comment
//         for (int label : randLabels()) {
//             if (hasNbrAtLabel(label)) {
//                 auto& nbr = nbrAtLabel(label);
//                 if ((nbr.isInState({State::Follower}) && !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir)))
//                     || (nbr.isInState({State::Leader}) && (nbr.moveDir < 0 || !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.moveDir))))) {
//                     //nbr.dirToHeadLabel(nbr.followDir): Gets the label in the direction the neighbor is following.
//                     //pointsAtMe(nbr, ...): Checks if the given label direction points at the current particle.
//                     followDir = labelToDir(label);
//                     state = State::Follower;
//                     printf("Following neighbor at label: %d\n", label);
//                      //comment
//                     break;
//                 }
//                 else if (nbr.isInState({State::Immo})) {
//                     break;
//                 }
//                 //     // Array of labels corresponding to clockwise directions
//                 //     std::vector<int> clockwiseLabels = {1, 2, 3, 4, 5, 0}; // Adjust this based on your specific label system

//                 //     // Iterate through each label in a clockwise manner
//                 //     for (int label : clockwiseLabels) {

//                 //         int dir = labelToDir(label);
//                 //         // Convert label to direction
//                 //         if (!hasNbrAtLabel(label) && !hasObjectAtLabel(label)) {
//                 //             moveDir = dir; // Set the move direction to the free position
//                 //             //int targetLabel = dirToHeadLabel(moveDir);
//                 //             if (canExpand(moveDir)) { // Ensure expansion is possible

//                 //                 // Backup current state in case we need to backtrack
//                 //                 auto originalState = state;
//                 //                 auto originalHead = head;

//                 //                 // Expand to the free position
//                 //                 expand(moveDir);
//                 //                 printf("Expanding in clockwise direction to label: %d\n", label);

//                 //                 bool connected = false;
//                 //                 for (int neighborLabel : clockwiseLabels) {
//                 //                     if ( hasNbrAtLabel(neighborLabel)) {
//                 //                         connected = true;
//                 //                         break;
//                 //                     }
//                 //                 }

//                 //                 // If the particle is still connected, contract the tail
//                 //                 if (connected) {
//                 //                     contractTail();
//                 //                     printf("Expansion successful. Contracting tail.\n");
//                 //                     break; // Exit the loop after finding the first free position
//                 //                 } else {
//                 //                     // Backtrack to original state if disconnected
//                 //                     printf("Expansion to label: %d would disconnect, backtracking.\n", label);

//                 //                     // Revert the expansion to original state
//                 //                     contractTail(); // Ensure the particle is in its initial state before re-expansion
//                 //                     printf("Contracted \n");
//                 //                     // Undo the expansion
//                 //                     int moveDirInverted = (dir + 3) % 6;

//                 //                     expand(moveDirInverted); // Expand back to the original position
//                 //                     break;
//                 //                     // Restore original state
//                 //                     state = originalState;
//                 //                     head = originalHead;
//                 //                 }
//                 //             }
//                 //         }
//                 //     }
//                 // }





//             }
//         }
//     } else if (isInState({State::Leader})) {
//         printf("Particle is a leader.\n");
//          //comment
//         if (moveDir == -1) {
//             moveDir = randDir();
//             printf("Random move direction chosen: %d\n", moveDir);
//              //comment
//         }
//         if (isContracted() && !canExpand(dirToHeadLabel(moveDir))) {
//             printf("Cannot expand in current move direction: %d\n", moveDir);
//              //comment
//             bool changed = false;
//             for (int label : randLabels()) {
//                 moveDir = labelToDir(label);
//                 if (!hasNbrAtLabel(label) && !hasObjectAtLabel(label)) {
//                     if (isExpanded()) {
//                         makeHeadLabel(label);
//                     }
//                     changed = true;
//                     printf("Changed move direction to: %d\n", moveDir);
//                      //comment
//                     break;
//                 }
//             }
//             if (!changed) {
//                 int label = labelOfFirstNbrInState({State::Idle, State::Follower});
//                 if (label >= 0) {
//                     passLeaderToken(label);
//                     moveDir = -1;
//                     printf("Passed leader token to label: %d\n", label);
//                      //comment
//                 } else {
//                     printf("Leader is surrounded by ImmoParticles\n");
//                      //comment
//                 }
//             }
//         }
//     }
//     else if (isInState({State::Immo})) {
//         printf("Particle is immobile. Forwarding communication without changing state.\n");
//         // Forward communication to neighbors without changing state
//         for (int label : randLabels()) {
//             if (hasNbrAtLabel(label)) {
//                 auto& nbr = nbrAtLabel(label);
//                 // Check if neighbor is suitable to forward communication
//                 if (nbr.isInState({State::Follower, State::Leader, State::Idle})) {
//                     // Forward communication to neighbor
//                     nbr.activate();
//                     printf("Communication forwarded to neighbor at label: %d\n", label);
//                     // Add any additional handling or logging here
//                 }
//                 // else if(nbr.isInState({State::Immo})){
//                 //     nbr.activate();
//                 //     break;

//                 // }

//             }

//         }

//     }
//     if (!isInState({State::Idle})) {
//         performMovement();
//     }
//     auto* immobilizedSystem = dynamic_cast<ImmobilizedParticleSystem*>(&system);
//     if (immobilizedSystem) {

//         immobilizedSystem->printAllParticleStates();
//     }
// }

// int Immobilizedparticles::nextClockwiseDir(int inputDir) {
//     return (inputDir + 1) % 6;
// }

// int Immobilizedparticles::nextCounterclockwiseDir(int inputDir) {
//     return (inputDir - 1 + 6) % 6;
// }


// void Immobilizedparticles::tryToBecomeLeader() {
//     printf("Trying to become leader.\n");
//      //comment
//     for (int label : randLabels()) {
//         moveDir = labelToDir(label);
//         if ((isExpanded() && !hasNbrAtLabel(label) && !hasObjectAtLabel(label)) || canExpand(labelToDir(label))) {
//             if (isExpanded()) {
//                 makeHeadLabel(label);
//             }
//             state = State::Leader;
//             leaderToken = false;
//             tokenForwardDir = -1;
//             tokenCurrentDir = -1;
//             passForward = false;
//             updateBorderPointColors(); // Only necessary for the visualisation.
//             printf("Became leader. New move direction: %d\n", moveDir);
//              //comment
//             return;
//         }
//     }

//     moveDir = -1;
//     printf("Could not become leader, passing on leader token.\n");
//      //comment

//     int label = 0;
//     for (int l : uniqueLabels()) {
//         if (labelToDir(l) == tokenCurrentDir) {
//             label = l;
//         }
//     }

//     int numLabels = isContracted() ? 6 : 10;

//     if (passForward) {
//         while (hasObjectAtLabel(label)) {
//             label = (label + 1) % numLabels;
//             passForward = false;
//         }
//     } else {
//         do {
//             label = (label - 1 + numLabels) % numLabels;
//         } while (!hasObjectAtLabel(label));
//         while (hasObjectAtLabel(label)) {
//             label = (label + 1) % numLabels;
//         }
//     }

//     passLeaderToken(label);
// }

// void Immobilizedparticles::passLeaderToken(const int label) {
//     if (isExpanded()) {
//         makeHeadLabel(label);
//     }

//     state = State::Follower;
//     followDir = labelToDir(label);
//     if (tokenForwardDir < 0) {
//         passForward = true;
//         tokenForwardDir = followDir;
//     }

//     auto& nbr = nbrAtLabel(label);
//     nbr.leaderToken = true;
//     nbr.tokenForwardDir = dirToNbrDir(nbr, tokenForwardDir);
//     nbr.tokenCurrentDir = dirToNbrDir(nbr, followDir);
//     nbr.passForward = passForward;
//     if (nbr.tokenForwardDir == nbr.tokenCurrentDir && tokenForwardDir != tokenCurrentDir) {
//         if (randBool()) {
//             nbr.passForward = true;
//         }
//     }
//     nbr.updateBorderPointColors(); // Only necessary for the visualisation.

//     leaderToken = false;
//     tokenForwardDir = -1;
//     tokenCurrentDir = -1;
//     passForward = false;
//     updateBorderPointColors(); // Only necessary for the visualisation.

//     printf("Passed leader token to neighbor at label: %d\n", label);
//      //comment
// }

// void Immobilizedparticles::performMovement() {
//     if (isExpanded() && isInState({State::Follower, State::Leader}) && !hasBlockingTailNbr()) {
//         printf("Contracting tail.\n");
//          //comment
//         contractTail();
//     } else if (isContracted() && isInState({State::Follower}) && hasTailAtLabel(dirToHeadLabel(followDir))) {
//         printf("Expanding towards follow direction: %d\n", followDir);
//          //comment
//         int followLabel = dirToHeadLabel(followDir);
//         auto& nbr = nbrAtLabel(followLabel);
//         int nbrContractionDir = nbrDirToDir(nbr, (nbr.tailDir() + 3) % 6);
//         push(followLabel);
//         followDir = nbrContractionDir;
//     } else if (isInState({State::Leader}) && isContracted() && !(freeState && lineState) && canExpand(dirToHeadLabel(moveDir))) {
//         printf("Expanding towards move direction: %d\n", moveDir);
//          //comment
//         expand(dirToHeadLabel(moveDir));
//     }
//     updateBoolStates();
//     updateBorderColors(); // Only necessary for the visualisation.
//     printf("Performed movement.\n");
//      //comment
// }




// #include <set>
// #include <queue> // Include this to use std::queue
// #include "immobilizedparticles.h"
// #include <QtGlobal>


// // Function declaration before it's used in the constructor
// bool doesEnclosureOccur(const std::set<Node>& occupied, Node testNode);
// //bool isPathClear(const std::set<Node>& occupied, const Node& start, const Node& end);

// Immobilizedparticles::Immobilizedparticles(const Node head,
//                                            const int globalTailDir,
//                                            const int orientation,
//                                            AmoebotSystem& system,
//                                            State state)
//     : AmoebotParticle(head, globalTailDir, orientation, system),
//     state(state),
//     moveDir(-1),
//     followDir(-1),
//     leaderToken(false),
//     tokenForwardDir(-1),
//     tokenCurrentDir(-1),
//     passForward(false),
//     freeState(false),
//     lineState(false),
//     _borderColorsSet(false)
// {
//     _borderColors.fill(-1);
//     _borderPointColors.fill(-1);
// }


// void Immobilizedparticles::activate() {

//     // This the main procedure that runs when a particle is activated.

//     if (leaderToken) {
//         // p has received the leader token.
//         // If p has an empty neighbour node, it can become the Leader.
//         // Otherwise, it needs to pass on the token.
//         tryToBecomeLeader();
//         return;
//         // The return is only here because it looks nicer when the particle does not start moving as soon as it becomes Leader.
//     } else if (isInState({State::Idle})) {
//         // p tries to follow any neighbour that is either the Leader or a Follower.
//         for (int label : randLabels()) {
//             if (hasNbrAtLabel(label)) {
//                 auto& nbr = nbrAtLabel(label);
//                 if ((nbr.isInState({State::Follower}) && !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir)))
//                     || (nbr.isInState({State::Leader})   && (nbr.moveDir < 0 || !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.moveDir))))) {
//                     followDir = labelToDir(label);
//                     state = State::Follower;
//                     break;
//                 }
//             }
//         }
//     } else if (isInState({State::Leader})) {
//         if (moveDir == -1) {
//             moveDir = randDir();
//         }
//         if (isContracted() && !canExpand(dirToHeadLabel(moveDir))) {
//             // p cannot move in its moveDir. Try to change moveDir to point towards an empty node.
//             bool changed = false;
//             for (int label : randLabels()) {
//                 moveDir = labelToDir(label);
//                 if (!hasNbrAtLabel(label) && !hasObjectAtLabel(label)) {
//                     if (isExpanded()) {
//                         makeHeadLabel(label);
//                     }
//                     changed = true;
//                     break;
//                 }
//             }

//             if (!changed) {
//                 // p's node has no empty neighbouring node.
//                 // p needs to pass on the leader token to a suitable random neighbour.
//                 int label = labelOfFirstNbrInState({State::Idle, State::Follower});
//                 if (label >= 0) {
//                     passLeaderToken(label);
//                     moveDir = -1;
//                 } else {
//                     // This can only happen if the Leader is the only particle and it is
//                     // surrounded by ImmoParticles.
//                     // So basically never, unless you are testing weird edge cases.
//                 }
//             }
//         }
//     }

//     if (!isInState({State::Idle})) {
//         performMovement();
//     }

//     return;
// }


// void Immobilizedparticles::tryToBecomeLeader() {
//     for (int label : randLabels()) {
//         // Try to find a movement direction that is not occupied.
//         moveDir = labelToDir(label);
//         if ((isExpanded() && !hasNbrAtLabel(label) && !hasObjectAtLabel(label)) || canExpand(labelToDir(label))) {
//             if (isExpanded()) {
//                 makeHeadLabel(label);
//             }
//             state = State::Leader;
//             leaderToken = false;
//             tokenForwardDir = -1;
//             tokenCurrentDir = -1;
//             passForward = false;
//             updateBorderPointColors(); // Only necessary for the visualisation.
//             return;
//         }
//     }

//     // Reset moveDir since the particle has not become the Leader.
//     moveDir = -1;

//     // Could not become Leader: Pass on leader token to a suitable neighbour.

//     // Find the label that corresponds to the current token direction.
//     int label = 0;
//     for (int l : uniqueLabels()) {
//         if (labelToDir(l) == tokenCurrentDir) {
//             label = l;
//         }
//     }

//     int numLabels = isContracted() ? 6 : 10;

//     if (passForward) {
//         // Try to pass the token towards the designated token direction.
//         while (hasObjectAtLabel(label)) {
//             // Cannot move towards the designated token direction, take the next possible
//             // label with a node that is not occupied by an object.
//             label = (label + 1) % numLabels;
//             passForward = false;
//         }
//     } else {
//         // Pass the token around the neighbouring object.
//         do {
//             // Set label to point towards the object that p is moving around.
//             label = (label - 1 + numLabels) % numLabels;
//         } while (!hasObjectAtLabel(label));
//         while (hasObjectAtLabel(label)) {
//             // Set label to the first node that is no longer occupied by an object.
//             label = (label + 1) % numLabels;
//         }
//     }

//     passLeaderToken(label);
// }



// void Immobilizedparticles::passLeaderToken(const int label) {
//     if (isExpanded()) {
//         makeHeadLabel(label);
//     }

//     // Follow the particle that will receive the particle.
//     state = State::Follower;
//     followDir = labelToDir(label);
//     if (tokenForwardDir < 0) {
//         // If p was a Leader, the designated token direction needs to be set.
//         // Initialise to the direction that the token is passed to.
//         passForward = true;
//         tokenForwardDir = followDir;
//     }

//     // Pass the token with all its variables.
//     auto& nbr = nbrAtLabel(label);
//     nbr.leaderToken = true;
//     nbr.tokenForwardDir = dirToNbrDir(nbr, tokenForwardDir);
//     nbr.tokenCurrentDir = dirToNbrDir(nbr, followDir);
//     nbr.passForward = passForward;
//     if (nbr.tokenForwardDir == nbr.tokenCurrentDir && tokenForwardDir != tokenCurrentDir) {
//         if (randBool()) {
//             nbr.passForward = true;
//         }
//     }
//     nbr.updateBorderPointColors(); // Only necessary for the visualisation.

//     // Reset internal variables that are only required when p has the leader token.
//     leaderToken = false;
//     tokenForwardDir = -1;
//     tokenCurrentDir = -1;
//     passForward = false;
//     updateBorderPointColors(); // Only necessary for the visualisation.
// }

// void Immobilizedparticles::performMovement() {
//     // Fairly obvious: p tries to expand if contracted and contract if expanded.
//     // The usual constraints apply.
//     if (isExpanded() && isInState({State::Follower, State::Leader}) && !hasBlockingTailNbr()) {
//         contractTail();
//     } else if (isContracted() && isInState({State::Follower}) && hasTailAtLabel(dirToHeadLabel(followDir))) {
//         int followLabel = dirToHeadLabel(followDir);
//         auto& nbr = nbrAtLabel(followLabel);
//         int nbrContractionDir = nbrDirToDir(nbr, (nbr.tailDir() + 3) % 6);
//         push(followLabel);
//         followDir = nbrContractionDir;
//     } else if (isInState({State::Leader}) && isContracted() && !(freeState && lineState) && canExpand(dirToHeadLabel(moveDir))) {
//         expand(dirToHeadLabel(moveDir));
//     }
//     updateBoolStates();
//     updateBorderColors(); // Only necessary for the visualisation.
// }

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
    if (hasObjectNbr() || hasNbrInState({State::Idle}) || hasNbrInState({State::Cluster}) || hasNbrInState({State::Immo})) {
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

// ImmobilizedParticleSystem::ImmobilizedParticleSystem(int numParticles, int numImmoParticles, int genExpExample, int numCoinFlips) {
//     Q_ASSERT(numParticles > 0);
//     Q_ASSERT(numImmoParticles >= 0);
//     Q_ASSERT(genExpExample == 0 || genExpExample == 1);
//     Q_ASSERT(numCoinFlips > 0);

//     std::set<Node> occupied;
//     std::set<Node> candidates;

//     _seedOrientation = randDir();
//     Immobilizedparticles* leader = nullptr;

//     for (int i = 0; i < 6; ++i) {
//         candidates.insert(Node(0, 0).nodeInDir(i));
//     }

//     while (numParticles > 0 || numImmoParticles > 0) {
//         int randIndex = randInt(0, candidates.size());
//         auto it = candidates.begin();
//         std::advance(it, randIndex);
//         Node randomCandidate = *it;

//         if (randBool((double) numParticles / ((double) numParticles + (double) numImmoParticles))) {
//             numParticles--;
//             Immobilizedparticles* newParticle = new Immobilizedparticles(randomCandidate, -1, randDir(), *this, Immobilizedparticles::State::Idle);
//             insert(newParticle);
//             leader = newParticle;  // Update leader to the last added non-immobilized particle
//         } else {
//             // Avoid adding an immobilized particle if it is enclosed by non-immobilized particles
//             bool isEnclosed = true;
//             for (int i = 0; i < 6; ++i) {
//                 Node neighbor = randomCandidate.nodeInDir(i);
//                 if (occupied.find(neighbor) == occupied.end()) {
//                     isEnclosed = false;
//                     break;
//                 }
//             }

//             if (!isEnclosed) {  // Only insert if not enclosed by non-immobilized particles
//                 numImmoParticles--;
//                 insert(new ImmoParticle(randomCandidate));
//             }
//         }

//         occupied.insert(randomCandidate);
//         candidates.erase(it);

//         for (int i = 0; i < 6; ++i) {
//             Node neighbor = randomCandidate.nodeInDir(i);
//             if (occupied.find(neighbor) == occupied.end()) {
//                 candidates.insert(neighbor);
//             }
//         }
//     }

//     if (leader != nullptr) {
//         leader->setState(Immobilizedparticles::State::Leader);
//     }
// }

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

// void ImmobilizedParticleSystem::printAllParticleStates() const{
//     for (const auto& particle : particles) {
//         auto immobileParticle = dynamic_cast<const Immobilizedparticles*>(particle);
//         if (immobileParticle) {
//             printf("%s\n", immobileParticle->inspectionText().toStdString().c_str());
//         }
//     }
// }






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
//     numParticles--;

//     for (int i = 0; i < 6; ++i) {
//         candidates.insert(Node(0, 0).nodeInDir(i));
//     }

//     while (numParticles > 0 || numImmoParticles > 0) {
//         int randIndex = randInt(0, candidates.size());
//         auto it = candidates.begin();
//         std::advance(it, randIndex);
//         Node randomCandidate = *it;

//         if (randBool((double) numParticles / ((double) numParticles + (double) numImmoParticles))) {
//             numParticles--;
//             insert(new Immobilizedparticles(randomCandidate, -1, randDir(), *this, Immobilizedparticles::State::Idle));
//         } else {
//             // Avoid adding an immobilized particle if it is enclosed by non-immobilized particles
//             bool isEnclosed = true;
//             for (int i = 0; i < 6; ++i) {
//                 Node neighbor = randomCandidate.nodeInDir(i);
//                 if (occupied.find(neighbor) == occupied.end()) {
//                     isEnclosed = false;
//                     break;
//                 }
//             }

//             if (!isEnclosed) {  // Only insert if not enclosed by non-immobilized particles
//                 numImmoParticles--;
//                 insert(new ImmoParticle(randomCandidate));
//                 }
//         }

//         occupied.insert(randomCandidate);
//         candidates.erase(it);

//         for (int i = 0; i < 6; ++i) {
//             Node neighbor = randomCandidate.nodeInDir(i);
//             if (occupied.find(neighbor) == occupied.end()) {
//                 candidates.insert(neighbor);
//             }
//         }
//     }
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
        if (hp->isInState({Immobilizedparticles::State::Idle}) || !hp->freeState || !hp->lineState || hp->isExpanded()) {
            return false;
        }
        if (hp->isInState({Immobilizedparticles::State::Leader})) {
            leaderExists = true;
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
    case State::Immo:  return 0xFF0000; //red
    case State::Cluster:  return  0xFFCC99; //unknown
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
