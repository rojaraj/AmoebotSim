#include <set>
#include <unistd.h>
#include "immobilizedparticles.h"
#include <QtGlobal>
#include <QOpenGLFunctions_2_0>
#include <vector>
#include <cstdio>
#include <array>

Immobilizedparticles::Immobilizedparticles(const Node head,
                                           const int globalTailDir,
                                           const int orientation,
                                           AmoebotSystem& system,
                                           State state)
    : AmoebotParticle(head, globalTailDir, orientation, system),
    state(state),
    moveDir(-1),
    followDir1(-1), // For target tree
    followDir2(-1), // For main tree
    phase(Phase::LeaderElection),
    isImmo(false),
    isIdle(false),
    tree2Ack(false),
    allChildrenAck(false),
    leaderToken(false),
    tokenForwardDir(-1),
    tokenCurrentDir(-1),
    passForward(false),
    freeState(false),
    lineState(false),
    _parentDir(-1),
    constructionDir(state == State::Seed ? 0 : -1),
    _hexagonDir(state == State::Seed ? 0 : -1),
    _borderColorsSet(false),
    currentAgent(0)
{
    _leaderborderColorLabels.fill(-1);
    _leaderborderPointColorLabels.fill(-1);
}

void Immobilizedparticles::activate() {
    updateParticleStatesAndPhases();

    switch (phase) {
    case Phase::LeaderElection:
        startLeaderElection(); // Electing a leader before constructing the tree
        break;

    case Phase::InitializeTrees:
        initializeTrees(); // Setting directions for the tree with leader, immobilized, and idle particles
        break;

    case Phase::CompleteTargetTree:
        changeIdleState(); // Changing states in the main tree and creating the target tree
        break;

    case Phase::LeaderMovement:
        performLeaderMovement(); // Moving leader and followers away from immobilized particles
        break;

    case Phase::HexagonFormation:
        processactivateHex(); // Forming hexagon shape from the target tree
        break;

    default:
        // Handle unexpected phase
        printf("Error: Invalid phase encountered!");
        break;
    }
}



//----------------------------Phase 1 ----------------------------
//----------------------------Leader Election----------------------------


void Immobilizedparticles::startLeaderElection(){

    updateParticleStatesAndPhases();

    if (isInState({State::Immo})) {
        state = State::Idle;
    }
    activateLeader();
}

void Immobilizedparticles::activateLeader(){


    if (state == State::Idle) {
        // Determine the number of neighbors of the current particle.
        // If there are no neighbors, then that means the particle is the only
        // one in the system and should declare itself as the leader.
        // If it is surrounded by 6 neighbors, then it cannot participate in
        // leader election.
        // Otherwise, the particle may participate in leader election and must
        // generate agents to do so.
        int numNbrs = getNumberOfNbrs();
        if (numNbrs == 0) {
            state = State::Leader;
            return;
        } else if (numNbrs == 6) {
            state = State::Finished;
        } else {
            int agentId = 0;
            for (int dir = 0; dir < 6; dir++) {
                if (!hasNbrAtLabel(dir) && hasNbrAtLabel((dir + 1) % 6)) {
                    Q_ASSERT(agentId < 3);

                    ImmoLeaderElectionAgent* agent = new ImmoLeaderElectionAgent();
                    agent->candidateParticle = this;
                    agent->localId = agentId + 1;
                    agent->agentDir = dir;
                    agent->nextAgentDir = getNextAgentDir(dir);
                    agent->prevAgentDir = getPrevAgentDir(dir);
                    agent->agentState = State::Candidate;
                    agent->subPhase = ImmoLeaderElectionAgent::SubPhase::SegmentComparison;
                    agent->setStateColor();

                    agent->paintBackSegment(0x696969);
                    agent->paintFrontSegment(0x696969);

                    agents.push_back(agent);
                    agentId++;
                }
            }
            state = State::Candidate;
            return;
        }
    } else if (state == State::Candidate) {
        agents.at(currentAgent)->activate();
        currentAgent = (currentAgent + 1) % agents.size();

        // The following is used by a particle in the candidate state to determine
        // whether or not to declare itself as the Leader or declare itself to be in
        // the Finished state depending on the state of its agents.
        bool allFinished = true;
        for (unsigned i = 0; i < agents.size(); i++) {
            ImmoLeaderElectionAgent* agent = agents.at(i);
            if (agent->agentState != State::Finished) {
                allFinished = false;
            }
            if (agent->agentState == State::Leader) {
                state = State::Leader;
                return;
            }
        }

        if (allFinished) {
            state = State::Finished;
        }
    }
    updateParticleStatesAndPhases();

    return;
}

int Immobilizedparticles::getNextAgentDir(const int agentDir) const {
    Q_ASSERT(!hasNbrAtLabel(agentDir));

    for (int dir = 1; dir < 6; dir++) {
        if (hasNbrAtLabel((agentDir - dir + 6) % 6)) {
            return (agentDir - dir + 6) % 6;
        }
    }

    Q_ASSERT(false);
    return -1;
}

int Immobilizedparticles::getPrevAgentDir(const int agentDir) const {
    Q_ASSERT(!hasNbrAtLabel(agentDir));

    for (int dir = 1; dir < 6; dir++) {
        if (hasNbrAtLabel((agentDir + dir) % 6)) {
            return (agentDir + dir) % 6;
        }
    }

    Q_ASSERT(false);
    return -1;
}

int Immobilizedparticles::getNumberOfNbrs() const {
    int count = 0;
    for (int dir = 0; dir < 6; dir++) {
        if (hasNbrAtLabel(dir)) {
            count++;
        }
    }
    return count;
}

std::array<int, 18> Immobilizedparticles::leaderborderColorLabels() const {
    return _leaderborderColorLabels;
}

std::array<int, 6> Immobilizedparticles::leaderborderPointColorLabels() const {
    return _leaderborderPointColorLabels;
}


//----------------------------BEGIN AGENT CODE----------------------------

Immobilizedparticles::ImmoLeaderElectionAgent::ImmoLeaderElectionAgent() :
    localId(-1),
    agentDir(-1),
    nextAgentDir(-1),
    prevAgentDir(-1),
    agentState(State::Idle),
    candidateParticle(nullptr) {}

void Immobilizedparticles::ImmoLeaderElectionAgent::activate() {
    passTokensDir = randInt(0, 2);
    if (agentState == State::Candidate) {
        // Segment Comparison
        if (hasAgentToken<ActiveSegmentCleanToken>(nextAgentDir)) {
            activeClean(false);
            takeAgentToken<ActiveSegmentCleanToken>(nextAgentDir);
        }
        if (hasAgentToken<PassiveSegmentCleanToken>(prevAgentDir)) {
            passiveClean(false);
            takeAgentToken<PassiveSegmentCleanToken>(prevAgentDir);
            paintBackSegment(0x696969);
        }
        if (hasAgentToken<SegmentLeadToken>(prevAgentDir)) {
            ImmoLeaderElectionAgent* prev = prevAgent();
            if (prev != nullptr &&
                !prev->hasAgentToken<PassiveSegmentToken>(prev->nextAgentDir)) {
                takeAgentToken<SegmentLeadToken>(prevAgentDir);
                passAgentToken<PassiveSegmentToken>
                    (prevAgentDir,
                     std::make_shared<PassiveSegmentToken>(-1, true));
                paintBackSegment(0x696969);
            }
        }
        if (hasAgentToken<ActiveSegmentToken>(nextAgentDir) &&
            passTokensDir == 0) {
            if (!absorbedActiveToken) {
                bool isFinal =
                    peekAgentToken<ActiveSegmentToken>(nextAgentDir)->isFinal;
                ImmoLeaderElectionAgent* next = nextAgent();
                if (isFinal) {
                    takeAgentToken<ActiveSegmentToken>(nextAgentDir);
                    passAgentToken<FinalSegmentCleanToken>
                        (nextAgentDir,
                         std::make_shared<FinalSegmentCleanToken>(-1, true));
                } else if (next != nullptr &&
                           !next->hasAgentToken<PassiveSegmentCleanToken>
                            (next->prevAgentDir)) {
                    passAgentToken<PassiveSegmentCleanToken>
                        (nextAgentDir, std::make_shared<PassiveSegmentCleanToken>());
                    passiveClean(true);
                    generatedCleanToken = true;
                    candidateParticle->putToken
                        (std::make_shared<ActiveSegmentCleanToken>(nextAgentDir));
                    activeClean(true);
                    absorbedActiveToken = true;
                    isCoveredCandidate = true;
                    agentState = State::Demoted;
                    setStateColor();
                    return;
                }
            } else {
                Q_ASSERT(false);
                passAgentToken<ActiveSegmentToken>
                    (prevAgentDir, std::make_shared<ActiveSegmentToken>());
            }
        }

        // Coin Flipping
        if (hasAgentToken<CandidacyAnnounceToken>(prevAgentDir) &&
            passTokensDir == 1) {
            takeAgentToken<CandidacyAnnounceToken>(prevAgentDir);
            passAgentToken<CandidacyAckToken>
                (prevAgentDir, std::make_shared<CandidacyAckToken>());
            paintBackSegment(0x696969);
            if (waitingForTransferAck) {
                gotAnnounceBeforeAck = true;
            } else {
                gotAnnounceInCompare = true;
            }
        }

        // Solitude Verification
        // Here we check whether or not the SolitudeActiveToken has a set local_id
        // to avoid the possibility that the current agent might incorrectly
        // assume that the SolitudeActiveToken owned by the particle is meant for
        // the agent to pass back (and set the local_id value for); for example, in
        // the case where a candidate agent's nextAgentDir and prevAgentDir are the
        // same.
        if (passTokensDir == 1 &&
            hasAgentToken<SolitudeActiveToken>(prevAgentDir) &&
            peekAgentToken<SolitudeActiveToken>(prevAgentDir)->local_id == -1) {
            peekAgentToken<SolitudeActiveToken>(prevAgentDir)->local_id = localId;
            passAgentToken<SolitudeActiveToken>
                (prevAgentDir, takeAgentToken<SolitudeActiveToken>(prevAgentDir));
            paintBackSegment(0x696969);
        }

        if (hasAgentToken<SolitudeNegativeXToken>(nextAgentDir)) {
            peekAgentToken<SolitudeNegativeXToken>(nextAgentDir)->isSettled = true;
        }

        if (hasAgentToken<SolitudePositiveXToken>(nextAgentDir)) {
            peekAgentToken<SolitudePositiveXToken>(nextAgentDir)->isSettled = true;
        }

        if (hasAgentToken<SolitudeNegativeYToken>(nextAgentDir)) {
            peekAgentToken<SolitudeNegativeYToken>(nextAgentDir)->isSettled = true;
        }

        if (hasAgentToken<SolitudePositiveYToken>(nextAgentDir)) {
            peekAgentToken<SolitudePositiveYToken>(nextAgentDir)->isSettled = true;
        }

        if (subPhase == SubPhase::SegmentComparison) {
            if (passTokensDir == 1 &&
                hasAgentToken<PassiveSegmentToken>(nextAgentDir)) {
                ImmoLeaderElectionAgent* prev = prevAgent();
                if (prev != nullptr &&
                    !prev->hasAgentToken<ActiveSegmentToken>(prev->nextAgentDir)) {
                    bool isFinalCheck =
                        takeAgentToken<PassiveSegmentToken>(nextAgentDir)->isFinal;
                    passAgentToken<ActiveSegmentToken>
                        (prevAgentDir,
                         std::make_shared<ActiveSegmentToken>(-1, isFinalCheck));
                    if (isFinalCheck) {
                        paintFrontSegment(0x696969);
                    }
                }
            } else if (hasAgentToken<FinalSegmentCleanToken>(prevAgentDir)) {
                bool coveredCandidateCheck =
                    takeAgentToken<FinalSegmentCleanToken>(prevAgentDir)->
                    hasCoveredCandidate;
                if (!coveredCandidateCheck && !gotAnnounceInCompare) {
                    agentState = State::Demoted;
                    setStateColor();
                } else {
                    subPhase = SubPhase::CoinFlipping;
                    setStateColor();
                }
                comparingSegment = false;
                gotAnnounceInCompare = false;
                return;
            } else if (!comparingSegment && passTokensDir == 0) {
                passAgentToken<SegmentLeadToken>
                    (nextAgentDir, std::make_shared<SegmentLeadToken>());
                paintFrontSegment(0xff0000);
                comparingSegment = true;
            }
        } else if (subPhase == SubPhase::CoinFlipping) {
            if (hasAgentToken<CandidacyAckToken>(nextAgentDir)) {
                takeAgentToken<CandidacyAckToken>(nextAgentDir);
                paintFrontSegment(0x696969);
                if (!gotAnnounceBeforeAck) {
                    agentState = State::Demoted;
                } else {
                    subPhase = SubPhase::SolitudeVerification;
                }
                setStateColor();
                waitingForTransferAck = false;
                gotAnnounceBeforeAck = false;
                return;
            } else if (!waitingForTransferAck && passTokensDir == 0 && randBool()) {
                passAgentToken<CandidacyAnnounceToken>
                    (nextAgentDir, std::make_shared<CandidacyAnnounceToken>());
                paintFrontSegment(0xffa500);
                waitingForTransferAck = true;
            }
        } else if (subPhase == SubPhase::SolitudeVerification) {
            if (!createdLead && passTokensDir == 0) {
                passAgentToken<SolitudeActiveToken>
                    (nextAgentDir, std::make_shared<SolitudeActiveToken>());
                candidateParticle->putToken
                    (std::make_shared<SolitudePositiveXToken>(nextAgentDir, true));
                paintFrontSegment(0x00bfff);
                createdLead = true;
                hasGeneratedTokens = true;
            } else if (hasAgentToken<SolitudeActiveToken>(nextAgentDir) &&
                       peekAgentToken<SolitudeActiveToken>
                           (nextAgentDir)->local_id != -1) {
                int checkX = checkSolitudeXTokens();
                int checkY = checkSolitudeYTokens();
                bool isSole =
                    peekAgentToken<SolitudeActiveToken>(nextAgentDir)->isSoleCandidate;
                int id = peekAgentToken<SolitudeActiveToken>(nextAgentDir)->local_id;
                if (isSole && localId == id && checkX == 2 && checkY == 2) {
                    agentState = State::SoleCandidate;
                } else if (checkX == 1 || checkY == 1) {
                    // It should never reach this state since all solitude vector tokens
                    // that reach a candidate agent are settled
                    Q_ASSERT(false);
                    return;
                } else {
                    subPhase = SubPhase::SegmentComparison;
                }
                takeAgentToken<SolitudeActiveToken>(nextAgentDir);
                createdLead = false;
                cleanSolitudeVerificationTokens();
                setStateColor();
                return;
            }
        }
    } else if (agentState == State::Demoted) {
        ImmoLeaderElectionAgent* next = nextAgent();
        ImmoLeaderElectionAgent* prev = prevAgent();

        // Segment Comparison Tokens
        if (hasAgentToken<PassiveSegmentCleanToken>(prevAgentDir)) {
            passiveClean(false);
            if (passTokensDir == 0 && next != nullptr &&
                !next->hasAgentToken<PassiveSegmentCleanToken>(next->prevAgentDir)) {
                passAgentToken<PassiveSegmentCleanToken>
                    (nextAgentDir, takeAgentToken<PassiveSegmentCleanToken>
                     (prevAgentDir));
            }
            paintBackSegment(0x696969);
            paintFrontSegment(0x696969);
        }
        if (hasAgentToken<ActiveSegmentCleanToken>(nextAgentDir)) {
            activeClean(generatedCleanToken);
            if (passTokensDir == 1 && prev != nullptr &&
                !prev->hasAgentToken<ActiveSegmentCleanToken>(prev->nextAgentDir)) {
                passAgentToken<ActiveSegmentCleanToken>
                    (prevAgentDir,
                     takeAgentToken<ActiveSegmentCleanToken>(nextAgentDir));
                generatedCleanToken = false;
            }
        }
        // Here, we must take care not to pass a segment lead token into an agent
        // which also has the passive segment clean token. This is because a
        // segment lead token which is behind a passive segment clean token is not
        // meant to be cleaned by that passive clean token, i.e., the segment lead
        // token was not generated by the same agent that generated the passive
        // clean token.
        if (passTokensDir == 0 &&
            hasAgentToken<SegmentLeadToken>(prevAgentDir) &&
            next != nullptr &&
            !next->hasAgentToken<PassiveSegmentCleanToken>(next->prevAgentDir)) {
            passAgentToken<SegmentLeadToken>
                (nextAgentDir, takeAgentToken<SegmentLeadToken>(prevAgentDir));
            candidateParticle->putToken(
                std::make_shared<PassiveSegmentToken>(nextAgentDir, false));
            paintBackSegment(0xff0000);
            paintFrontSegment(0xff0000);
        }
        if (passTokensDir == 1 &&
            hasAgentToken<PassiveSegmentToken>(nextAgentDir) && prev != nullptr &&
            !prev->hasAgentToken<PassiveSegmentToken>(prev->nextAgentDir)) {
            if (peekAgentToken<PassiveSegmentToken>(nextAgentDir)->isFinal) {
                paintFrontSegment(0x696969);
                paintBackSegment(0x696969);
            }
            passAgentToken<PassiveSegmentToken>
                (prevAgentDir, takeAgentToken<PassiveSegmentToken>(nextAgentDir));
        }
        if (hasAgentToken<ActiveSegmentToken>(nextAgentDir)) {
            if (passTokensDir == 0 && !absorbedActiveToken) {
                if (takeAgentToken<ActiveSegmentToken>(nextAgentDir)->isFinal) {
                    passAgentToken<FinalSegmentCleanToken>
                        (nextAgentDir, std::make_shared<FinalSegmentCleanToken>());
                } else {
                    absorbedActiveToken = true;
                }
            } else if (passTokensDir == 1 && absorbedActiveToken &&
                       prev != nullptr &&
                       !prev->hasAgentToken<ActiveSegmentToken>(prev->nextAgentDir) &&
                       !prev->hasAgentToken<ActiveSegmentCleanToken>
                        (prev->nextAgentDir)) {
                passAgentToken<ActiveSegmentToken>
                    (prevAgentDir, takeAgentToken<ActiveSegmentToken>(nextAgentDir));
            }
        }
        if (passTokensDir == 0 &&
            hasAgentToken<FinalSegmentCleanToken>(prevAgentDir)) {
            if (isCoveredCandidate) {
                peekAgentToken<FinalSegmentCleanToken>
                    (prevAgentDir)->hasCoveredCandidate = isCoveredCandidate;
            }
            absorbedActiveToken = false;
            isCoveredCandidate = false;
            passAgentToken<FinalSegmentCleanToken>
                (nextAgentDir, takeAgentToken<FinalSegmentCleanToken>(prevAgentDir));
        }

        // Coin Flipping Tokens
        if (passTokensDir == 0 &&
            hasAgentToken<CandidacyAnnounceToken>(prevAgentDir) &&
            next != nullptr &&
            !next->hasAgentToken<PassiveSegmentCleanToken>(next->prevAgentDir)) {
            passAgentToken<CandidacyAnnounceToken>
                (nextAgentDir, takeAgentToken<CandidacyAnnounceToken>(prevAgentDir));
            paintBackSegment(0xffa500);
            paintFrontSegment(0xffa500);
        }
        if (passTokensDir == 1 &&
            hasAgentToken<CandidacyAckToken>(nextAgentDir)) {
            passAgentToken<CandidacyAckToken>
                (prevAgentDir, takeAgentToken<CandidacyAckToken>(nextAgentDir));
            paintFrontSegment(0x696969);
            paintBackSegment(0x696969);
        }

        // Solitude Verification Tokens
        if (passTokensDir == 0 &&
            hasAgentToken<SolitudeActiveToken>(prevAgentDir) &&
            peekAgentToken<SolitudeActiveToken>(prevAgentDir)->local_id == -1 &&
            next != nullptr &&
            !next->hasAgentToken<PassiveSegmentCleanToken>(next->prevAgentDir) &&
            !hasGeneratedTokens) {
            std::shared_ptr<SolitudeActiveToken> token =
                takeAgentToken<SolitudeActiveToken>(prevAgentDir);
            std::pair<int, int> generatedPair = augmentDirVector(token->vector);
            generateSolitudeVectorTokens(generatedPair);
            token->vector = generatedPair;
            paintBackSegment(0x00bfff);
            paintFrontSegment(0x00bfff);
            passAgentToken<SolitudeActiveToken>(nextAgentDir, token);
            hasGeneratedTokens = true;
        } else if (passTokensDir == 1 &&
                   hasAgentToken<SolitudeActiveToken>(nextAgentDir) &&
                   peekAgentToken<SolitudeActiveToken>
                       (nextAgentDir)->local_id != -1 &&
                   hasGeneratedTokens) {
            int checkX = checkSolitudeXTokens();
            int checkY = checkSolitudeYTokens();
            if ((checkX == 2 && checkY == 2) ||
                !peekAgentToken<SolitudeActiveToken>(nextAgentDir)->isSoleCandidate) {
                passAgentToken<SolitudeActiveToken>
                    (prevAgentDir, takeAgentToken<SolitudeActiveToken>(nextAgentDir));
                cleanSolitudeVerificationTokens();
            } else if (checkX == 0 || checkY == 0) {
                std::shared_ptr<SolitudeActiveToken> token =
                    takeAgentToken<SolitudeActiveToken>(nextAgentDir);
                token->isSoleCandidate = false;
                passAgentToken<SolitudeActiveToken>(prevAgentDir, token);
                cleanSolitudeVerificationTokens();
            }
        }

        if (hasAgentToken<SolitudeNegativeXToken>(nextAgentDir) &&
            !peekAgentToken<SolitudeNegativeXToken>(nextAgentDir)->isSettled &&
            hasGeneratedTokens) {
            if (passTokensDir == 1 && prev != nullptr &&
                !prev->hasAgentToken<SolitudeNegativeXToken>(prev->nextAgentDir)) {
                passAgentToken<SolitudeNegativeXToken>
                    (prevAgentDir,
                     takeAgentToken<SolitudeNegativeXToken>(nextAgentDir));
            } else if (prev != nullptr &&
                       prev->hasAgentToken<SolitudeNegativeXToken>
                       (prev->nextAgentDir) &&
                       prev->peekAgentToken<SolitudeNegativeXToken>
                       (prev->nextAgentDir)->isSettled) {
                peekAgentToken<SolitudeNegativeXToken>(nextAgentDir)->isSettled = true;
            }
        }

        if (hasAgentToken<SolitudePositiveXToken>(nextAgentDir) &&
            !peekAgentToken<SolitudePositiveXToken>(nextAgentDir)->isSettled &&
            hasGeneratedTokens) {
            if (passTokensDir == 1 && prev != nullptr &&
                !prev->hasAgentToken<SolitudePositiveXToken>(prev->nextAgentDir)) {
                passAgentToken<SolitudePositiveXToken>
                    (prevAgentDir,
                     takeAgentToken<SolitudePositiveXToken>(nextAgentDir));
            } else if (prev != nullptr &&
                       prev->hasAgentToken<SolitudePositiveXToken>
                       (prev->nextAgentDir) &&
                       prev->peekAgentToken<SolitudePositiveXToken>
                       (prev->nextAgentDir)->isSettled) {
                peekAgentToken<SolitudePositiveXToken>(nextAgentDir)->isSettled = true;
            }
        }

        if (hasAgentToken<SolitudeNegativeYToken>(nextAgentDir) &&
            !peekAgentToken<SolitudeNegativeYToken>(nextAgentDir)->isSettled &&
            hasGeneratedTokens) {
            if (passTokensDir == 1 && prev != nullptr &&
                !prev->hasAgentToken<SolitudeNegativeYToken>(prev->nextAgentDir)) {
                passAgentToken<SolitudeNegativeYToken>
                    (prevAgentDir,
                     takeAgentToken<SolitudeNegativeYToken>(nextAgentDir));
            } else if (prev != nullptr &&
                       prev->hasAgentToken<SolitudeNegativeYToken>
                       (prev->nextAgentDir) &&
                       prev->peekAgentToken<SolitudeNegativeYToken>
                       (prev->nextAgentDir)->isSettled) {
                peekAgentToken<SolitudeNegativeYToken>(nextAgentDir)->isSettled = true;
            }
        }

        if (hasAgentToken<SolitudePositiveYToken>(nextAgentDir) &&
            !peekAgentToken<SolitudePositiveYToken>(nextAgentDir)->isSettled &&
            hasGeneratedTokens) {
            if (passTokensDir == 1 && prev != nullptr &&
                !prev->hasAgentToken<SolitudePositiveYToken>(prev->nextAgentDir)) {
                passAgentToken<SolitudePositiveYToken>
                    (prevAgentDir,
                     takeAgentToken<SolitudePositiveYToken>(nextAgentDir));
            } else if (prev != nullptr &&
                       prev->hasAgentToken<SolitudePositiveYToken>
                       (prev->nextAgentDir) &&
                       prev->peekAgentToken<SolitudePositiveYToken>
                       (prev->nextAgentDir)->isSettled) {
                peekAgentToken<SolitudePositiveYToken>(nextAgentDir)->isSettled = true;
            }
        }

        if (passTokensDir == 0 && hasAgentToken<BorderTestToken>(prevAgentDir)) {
            std::shared_ptr<BorderTestToken> token =
                takeAgentToken<BorderTestToken>(prevAgentDir);
            token->borderSum = addNextBorder(token->borderSum);
            passAgentToken<BorderTestToken>(nextAgentDir, token);
            paintBackSegment(-1);
            paintFrontSegment(-1);
            agentState = State::Finished;
            setStateColor();
        }

    } else if (agentState == State::SoleCandidate) {
        if (!testingBorder) {
            std::shared_ptr<BorderTestToken> token =
                std::make_shared<BorderTestToken>(prevAgentDir, addNextBorder(0));
            passAgentToken(nextAgentDir, token);
            paintFrontSegment(-1);
            testingBorder = true;
        } else if (hasAgentToken<BorderTestToken>(prevAgentDir) &&
                   peekAgentToken<BorderTestToken>(prevAgentDir)->borderSum != -1) {
            int borderSum = takeAgentToken<BorderTestToken>(prevAgentDir)->borderSum;
            paintBackSegment(-1);
            if (borderSum == 1) {
                agentState = State::Leader;
            } else if (borderSum == 4) {
                agentState = State::Finished;
            } else {
                Q_ASSERT(false);
            }
            setStateColor();
            testingBorder = false;
            return;
        }
    }
}

void Immobilizedparticles::ImmoLeaderElectionAgent::activeClean(bool first) {
    if (!first) {
        if (hasAgentToken<ActiveSegmentToken>(nextAgentDir)) {
            takeAgentToken<ActiveSegmentToken>(nextAgentDir);
        }
        if (agentState != State::Candidate) {
            if (hasAgentToken<FinalSegmentCleanToken>(prevAgentDir)) {
                takeAgentToken<FinalSegmentCleanToken>(prevAgentDir);
            }
        }
    } else {
        if (hasAgentToken<FinalSegmentCleanToken>(prevAgentDir)) {
            takeAgentToken<FinalSegmentCleanToken>(prevAgentDir);
        }
    }
    absorbedActiveToken = false;
}

void Immobilizedparticles::ImmoLeaderElectionAgent::passiveClean(bool first) {
    // If the current agent performing the passive clean is not the initial
    // candidate agent which generated the passive clean token
    if (!first) {
        if (hasAgentToken<SolitudeActiveToken>(prevAgentDir) &&
            peekAgentToken<SolitudeActiveToken>(prevAgentDir)->local_id == -1) {
            takeAgentToken<SolitudeActiveToken>(prevAgentDir);
        }
        if (hasAgentToken<SegmentLeadToken>(prevAgentDir)) {
            takeAgentToken<SegmentLeadToken>(prevAgentDir);
        }
        if (hasAgentToken<CandidacyAnnounceToken>(prevAgentDir)) {
            takeAgentToken<CandidacyAnnounceToken>(prevAgentDir);
        }
        // Condition for non-candidate agents
        if (agentState != State::Candidate) {
            if (hasAgentToken<PassiveSegmentToken>(nextAgentDir)) {
                takeAgentToken<PassiveSegmentToken>(nextAgentDir);
            }
            if (hasAgentToken<SolitudeActiveToken>(nextAgentDir) &&
                peekAgentToken<SolitudeActiveToken>(nextAgentDir)->local_id != -1) {
                takeAgentToken<SolitudeActiveToken>(nextAgentDir);
            }
            if (hasAgentToken<CandidacyAckToken>(nextAgentDir)) {
                takeAgentToken<CandidacyAckToken>(nextAgentDir);
            }
            if (hasGeneratedTokens) {
                cleanSolitudeVerificationTokens();
            }
        }
    } else { // Condition for the ex-candidate which generated the cleaning tokens
        if (hasAgentToken<PassiveSegmentToken>(nextAgentDir)) {
            takeAgentToken<PassiveSegmentToken>(nextAgentDir);
        }
        if (hasAgentToken<CandidacyAckToken>(nextAgentDir)) {
            takeAgentToken<CandidacyAckToken>(nextAgentDir);
        }
        if (hasAgentToken<SolitudeActiveToken>(nextAgentDir) &&
            peekAgentToken<SolitudeActiveToken>(nextAgentDir)->local_id != -1) {
            takeAgentToken<SolitudeActiveToken>(nextAgentDir);
        }
        if (hasGeneratedTokens) {
            cleanSolitudeVerificationTokens();
        }
    }
    paintFrontSegment(0x696969);
    paintBackSegment(0x696969);
}

std::pair<int, int> Immobilizedparticles::ImmoLeaderElectionAgent::
    augmentDirVector(std::pair<int, int> vector) {
    unsigned int offset = (nextAgentDir - ((prevAgentDir + 3) % 6) + 6) % 6;
    const std::array<std::pair<int, int>, 6> vectors =
        { std::make_pair(1, 0), std::make_pair(0, 1), std::make_pair(-1, 1),
         std::make_pair(-1, 0), std::make_pair(0, -1), std::make_pair(1, -1) };

    for (unsigned i = 0; i < vectors.size(); ++i) {
        if (vector == vectors.at(i)) {
            return vectors.at((i + offset) % 6);
        }
    }

    Q_ASSERT(false);
    return std::make_pair(0, 0);
}

void Immobilizedparticles::ImmoLeaderElectionAgent::
    generateSolitudeVectorTokens(std::pair<int, int> vector) {
    // We initialize the solitude vector tokens with an origin direction
    // of nextAgentDir because this is the only direction from which these
    // tokens can come from, so it does not matter whether or not these tokens
    // were generated by the agent or if they were passed to the agent.
    switch(vector.first) {
    case -1:
        candidateParticle->putToken
            (std::make_shared<SolitudeNegativeXToken>(nextAgentDir, false));
        break;
    case 0:
        break;
    case 1:
        candidateParticle->putToken
            (std::make_shared<SolitudePositiveXToken>(nextAgentDir, false));
        break;
    default:
        Q_ASSERT(false);
        break;
    }
    switch(vector.second) {
    case -1:
        candidateParticle->putToken
            (std::make_shared<SolitudeNegativeYToken>(nextAgentDir, false));
        break;
    case 0:
        break;
    case 1:
        candidateParticle->putToken
            (std::make_shared<SolitudePositiveYToken>(nextAgentDir, false));
        break;
    default:
        Q_ASSERT(false);
        break;
    }
}

int Immobilizedparticles::ImmoLeaderElectionAgent::checkSolitudeXTokens() const {
    if (hasAgentToken<SolitudePositiveXToken>(nextAgentDir) &&
        hasAgentToken<SolitudeNegativeXToken>(nextAgentDir)) {
        if (peekAgentToken<SolitudePositiveXToken>(nextAgentDir)->isSettled &&
            peekAgentToken<SolitudeNegativeXToken>(nextAgentDir)->isSettled) {
            return 2;
        } else {
            return 1;
        }
    } else if (hasAgentToken<SolitudePositiveXToken>(nextAgentDir)) {
        if (peekAgentToken<SolitudePositiveXToken>(nextAgentDir)->isSettled) {
            return 0;
        } else {
            return 1;
        }
    } else if (hasAgentToken<SolitudeNegativeXToken>(nextAgentDir)) {
        if (peekAgentToken<SolitudeNegativeXToken>(nextAgentDir)->isSettled) {
            return 0;
        } else {
            return 1;
        }
    } else {
        return 2;
    }
}

int Immobilizedparticles::ImmoLeaderElectionAgent::checkSolitudeYTokens() const {
    if (hasAgentToken<SolitudePositiveYToken>(nextAgentDir) &&
        hasAgentToken<SolitudeNegativeYToken>(nextAgentDir)) {
        if (peekAgentToken<SolitudePositiveYToken>(nextAgentDir)->isSettled &&
            peekAgentToken<SolitudeNegativeYToken>(nextAgentDir)->isSettled) {
            return 2;
        } else {
            return 1;
        }
    } else if (hasAgentToken<SolitudePositiveYToken>(nextAgentDir)) {
        if (peekAgentToken<SolitudePositiveYToken>(nextAgentDir)->isSettled) {
            return 0;
        } else {
            return 1;
        }
    } else if (hasAgentToken<SolitudeNegativeYToken>(nextAgentDir)) {
        if (peekAgentToken<SolitudeNegativeYToken>(nextAgentDir)->isSettled) {
            return 0;
        } else {
            return 1;
        }
    } else {
        return 2;
    }
}

void Immobilizedparticles::ImmoLeaderElectionAgent::
    cleanSolitudeVerificationTokens() {
    if (hasAgentToken<SolitudePositiveXToken>(nextAgentDir)) {
        takeAgentToken<SolitudePositiveXToken>(nextAgentDir);
    }
    if (hasAgentToken<SolitudePositiveYToken>(nextAgentDir)) {
        takeAgentToken<SolitudePositiveYToken>(nextAgentDir);
    }
    if (hasAgentToken<SolitudeNegativeXToken>(nextAgentDir)) {
        takeAgentToken<SolitudeNegativeXToken>(nextAgentDir);
    }
    if (hasAgentToken<SolitudeNegativeYToken>(nextAgentDir)) {
        takeAgentToken<SolitudeNegativeYToken>(nextAgentDir);
    }
    paintFrontSegment(0x696969);
    paintBackSegment(0x696969);
    hasGeneratedTokens = false;
}

int Immobilizedparticles::ImmoLeaderElectionAgent::
    addNextBorder(int currentSum) const {
    // adjust offset in modulo 6 to be compatible with modulo 5 computations
    int offsetMod6 = (prevAgentDir + 3) % 6 - nextAgentDir;
    if(4 <= offsetMod6 && offsetMod6 <= 5) {
        offsetMod6 -= 6;
    } else if(-5 <= offsetMod6 && offsetMod6 <= -3) {
        offsetMod6 += 6;
    }

    return (currentSum + offsetMod6 + 5) % 5;
}

template <class TokenType>
bool Immobilizedparticles::ImmoLeaderElectionAgent::
    hasAgentToken(int agentDir) const{
    auto prop = [agentDir](const std::shared_ptr<TokenType> token) {
        return token->origin == agentDir;
    };
    return candidateParticle->hasToken<TokenType>(prop);
}

template <class TokenType>
std::shared_ptr<TokenType>
    Immobilizedparticles::ImmoLeaderElectionAgent::
    peekAgentToken(int agentDir) const {
    auto prop = [agentDir](const std::shared_ptr<TokenType> token) {
        return token->origin == agentDir;
    };
    return candidateParticle->peekAtToken<TokenType>(prop);
}

template <class TokenType>
std::shared_ptr<TokenType>
Immobilizedparticles::ImmoLeaderElectionAgent::takeAgentToken(int agentDir) {
    auto prop = [agentDir](const std::shared_ptr<TokenType> token) {
        return token->origin == agentDir;
    };
    return candidateParticle->takeToken<TokenType>(prop);
}

template <class TokenType>
void Immobilizedparticles::ImmoLeaderElectionAgent::
    passAgentToken(int agentDir, std::shared_ptr<TokenType> token) {
    Immobilizedparticles* nbr = &candidateParticle->nbrAtLabel(agentDir);
    int origin = -1;
    for (int i = 0; i < 6; i++) {
        if (nbr->hasNbrAtLabel(i) && &nbr->nbrAtLabel(i) == candidateParticle) {
            origin = i;
            break;
        }
    }
    Q_ASSERT(origin != -1);
    token->origin = origin;
    nbr->putToken(token);
}

Immobilizedparticles::ImmoLeaderElectionAgent*
Immobilizedparticles::ImmoLeaderElectionAgent::nextAgent() const {
    Immobilizedparticles* nextNbr =
        &candidateParticle->nbrAtLabel(nextAgentDir);
    int originLabel = -1;
    for (int i = 0; i < 6; i++) {
        if (nextNbr->hasNbrAtLabel(i) &&
            &nextNbr->nbrAtLabel(i) == candidateParticle) {
            originLabel = i;
            break;
        }
    }
    Q_ASSERT(originLabel != -1);
    for (ImmoLeaderElectionAgent* agent : nextNbr->agents) {
        if (agent->prevAgentDir == originLabel) {
            return agent;
        }
    }
    Q_ASSERT(nextNbr->agents.size() == 0);
    return nullptr;
}

Immobilizedparticles::ImmoLeaderElectionAgent*
Immobilizedparticles::ImmoLeaderElectionAgent::prevAgent() const {
    Immobilizedparticles* prevNbr =
        &candidateParticle->nbrAtLabel(prevAgentDir);
    int originLabel = -1;
    for (int i = 0; i < 6; i++) {
        if (prevNbr->hasNbrAtLabel(i) &&
            &prevNbr->nbrAtLabel(i) == candidateParticle) {
            originLabel = i;
            break;
        }
    }
    Q_ASSERT(originLabel != -1);
    for (ImmoLeaderElectionAgent* agent : prevNbr->agents) {
        if (agent->nextAgentDir == originLabel) {
            return agent;
        }
    }
    Q_ASSERT(prevNbr->agents.size() == 0);
    return nullptr;
}

void Immobilizedparticles::ImmoLeaderElectionAgent::setStateColor() {
    int globalizedDir = candidateParticle->localToGlobalDir(agentDir);
    switch (agentState) {
    case State::Candidate:
        setSubPhaseColor();
        break;
    case State::Demoted:
        candidateParticle->_leaderborderPointColorLabels.at(globalizedDir)= 0x696969;
        break;
    case State::Immo:
        candidateParticle->_leaderborderPointColorLabels.at(globalizedDir) = 0xFF0000;
        break;
    case State::SoleCandidate:
        candidateParticle->_leaderborderPointColorLabels.at(globalizedDir) = 0x00ff00;
        break;
    case State::Leader:
        candidateParticle->_leaderborderPointColorLabels.at(globalizedDir) = -1;
        break;
    case State::Finished:
        candidateParticle->_leaderborderPointColorLabels.at(globalizedDir) = -1;
        break;
    default:
        break;
    }
}

void Immobilizedparticles::ImmoLeaderElectionAgent::setSubPhaseColor() {
    int globalizedDir = candidateParticle->localToGlobalDir(agentDir);
    switch (subPhase) {
    case SubPhase::SegmentComparison:
        candidateParticle->_leaderborderPointColorLabels.at(globalizedDir) = 0xff0000;
        break;
    case SubPhase::CoinFlipping:
        candidateParticle->_leaderborderPointColorLabels.at(globalizedDir) = 0xffa500;
        break;
    case SubPhase::SolitudeVerification:
        candidateParticle->_leaderborderPointColorLabels.at(globalizedDir) = 0x00bfff;
        break;
    default:
        Q_ASSERT(false);
        break;
    }
}

void Immobilizedparticles::ImmoLeaderElectionAgent::paintFrontSegment(const int color) {
    // Must use localToGlobalDir method to reconcile the difference between the
    // local orientation of the particle and the global orientation used by
    // drawing
    int tempDir = candidateParticle->localToGlobalDir(agentDir);
    int tempNextDir = candidateParticle->localToGlobalDir(nextAgentDir);
    while (tempDir != (tempNextDir + 1) % 6) {
        if ((tempDir + 5) % 6 != tempNextDir) {
            candidateParticle->_leaderborderColorLabels.at((3 * tempDir + 17) % 18) =
                color;
        }
        tempDir = (tempDir + 5) % 6;
    }

}

void Immobilizedparticles::ImmoLeaderElectionAgent::paintBackSegment(const int color) {
    // Must use localToGlobalDir method to reconcile the difference between the
    // local orientation of the particle and the global orientation used by
    // drawing
    candidateParticle->_leaderborderColorLabels.at(
        3 * candidateParticle->localToGlobalDir(agentDir) + 1) = color;
}

//----------------------------END AGENT CODE----------------------------





//----------------------------Phase 2----------------------------
//----------------------------Initialize Tree----------------------------


void Immobilizedparticles::initializeTrees() {

    // Update the states of all particles
    updateParticleStatesAndPhases();

    for (int label : uniqueLabels()) {   // Iterate over random labels
        if (hasNbrAtLabel(label)) {
            auto& nbr = nbrAtLabel(label);
            if (label != -1) {
                // If the neighbor is a Leader, set followDir1 and followDir2
                if (nbr.state == State::Leader) {
                    followDir1 = label;
                    if (followDir2 == -1) {
                        followDir2 = label;
                    }
                }
                // If neighbor has followDir2 set, and followDir2 isn't set yet
                else if (nbr.followDir2 != -1) {
                    if (followDir2 == -1) {
                        followDir2 = label;
                    }
                }

                // If particle is not immobilized and the neighbor has followDir1 set
                else if (!isImmobilized() && nbr.followDir1 != -1) {
                    followDir1 = label;
                }
            }
        }
    }

    // Handle tree2Ack logic - check if all children have acknowledged

    for (int label : childLabels()) {    // Iterate over child labels
        if (hasNbrAtLabel(label)) {
            auto& child = nbrAtLabel(label);
            if (!child.tree2Ack) {       // Check if the child has acknowledged
                allChildrenAck = true;
                break;
            }
        }
    }

    // Check if all children have acknowledged and neighbors have their followDir2 set
    if (!allChildrenAck && !hasNbrWithFollowDir2Unset()) {
        tree2Ack = true;
    }

}


bool Immobilizedparticles::hasNbrWithFollowDir2Unset()  {

    for (int label : uniqueLabels()) {
        // Check if there is a neighbor at the current label
        if (hasNbrAtLabel(label)) {
            // Get the neighbor particle at the current label
            const auto& nbr = nbrAtLabel(label);
            // Check if the neighbor's followDir2 is unset (equals -1)
            if (nbr.followDir2 == -1) {
                return true; // Return true as soon as we find such a neighbor
            }
        }
    }
    // If no neighbor with followDir2 unset is found, return false
    return false;
}


bool Immobilizedparticles::isImmobilized() const {
    return isInState({State::Immo});
}



//----------------------------End Initialize Tree Phase----------------------------


//----------------------------Phase 3----------------------------
//----------------------------CompleteTargetTree----------------------------


void Immobilizedparticles::changeIdleState() {

    if (isInState({State::Idle}))  {
        for (int label : uniqueLabels()) {
            if (hasNbrAtLabel(label)) {
                auto& nbr = nbrAtLabel(label);
                if(followDir2 != -1){
                    //checking if leader is not pointing to the idle particle
                    if (nbr.isInState({State::Leader})){ //&& !pointsAtMe(nbr, nbr.followDir2)) {
                        state = State::Follower;
                        followDir1 = labelToDir(label);
                        followDir2 = labelToDir(label);
                        break;
                    }
                }
            }
        }
    }

    if (isInState({State::Idle}))  {
        for (int label : uniqueLabels()) {
            if (hasNbrAtLabel(label)) {
                auto& nbr = nbrAtLabel(label);
                if(nbr.followDir2 != -1){
                    if(nbr.isInState({State::Follower}) && !pointsAtMe(nbr, nbr.followDir2)) {
                        state = State::Follower;
                        followDir1 = labelToDir(label);
                        followDir2 = labelToDir(label);
                        break;
                    }
                }
            }
        }
    }

    if (isInState({State::Idle})) {
        for (int label : uniqueLabels()) {
            if (hasNbrAtLabel(label)) {
                auto& nbr = nbrAtLabel(label);
                state = State::Marker;
                if(nbr.followDir2 != -1){
                    if (nbr.isInState({State::Leader}) && !pointsAtMe(nbr, nbr.followDir2)) {
                        state = State::Follower;
                        followDir1 = labelToDir(label);
                        followDir2 = labelToDir(label);
                        break;
                    }   else if(nbr.isInState({State::Leader})) {
                        state = State::Follower;
                        followDir1 = labelToDir(label);
                        followDir2 = labelToDir(label);
                        break;
                    }
                }
            }
        }
    }


    if (isInState({State::Marker})) {
        for (int label : uniqueLabels()) {
            if (hasNbrAtLabel(label)) {
                auto& nbr = nbrAtLabel(label);
                if(nbr.followDir2 != -1){
                    if(nbr.isInState({State::Follower}) && !pointsAtMe(nbr, nbr.followDir2)) {
                        state = State::Follower;
                        followDir1 = labelToDir(label);
                        followDir2 = labelToDir(label);
                        break;
                    }
                }
            }
        }
    }

    if (isInState({State::Immo})) {
        for (int label : uniqueLabels()) {
            if (hasNbrAtLabel(label)) {
                auto& nbr = nbrAtLabel(label);
                if(nbr.followDir2 != -1){
                    if(nbr.isInState({State::Immo, State::Follower, State::Leader}) && !pointsAtMe(nbr, nbr.followDir2)) {
                        followDir2 = labelToDir(label);
                        break;
                    }
                }
            }
        }
    }

    updateBorderPointColors();

    // Only perform movement after all idle particles have been processed

    if (hasCompletedchangeIdleState()) {
        performMarkerMovement();
    }
}



bool Immobilizedparticles::hasCompletedchangeIdleState() const {
    // Check if the current particle itself is in the Idle state
    if (isInState({State::Idle})) {
        return false;
    }

    // Iterate through neighbors to check if any of them are in the Idle state
    for (int label : uniqueLabels()) {
        if (hasNbrAtLabel(label)) {
            const auto& neighbor = nbrAtLabel(label);
            // If any neighbor is in the Idle state, return false
            if (neighbor.isInState({State::Idle})) {
                return false;
            }
        }
    }

    // If no Idle state is found among the particle and its neighbors, return true
    return true;
}



void Immobilizedparticles::performMarkerMovement() {

    // Check if the particle is in the Marker state and attempt to transition to Follower
    if (isInState({State::Marker})) {
        for (int label : uniqueLabels()) {
            if (hasNbrAtLabel(label)) {
                auto& nbr = nbrAtLabel(label);
                if (nbr.followDir2 != -1) {
                    if (nbr.isInState({State::Leader}) ){
                        state = State::Follower;
                        followDir1 = labelToDir(label);
                        followDir2 = labelToDir(label);
                        break;
                    }
                }
            }
        }
    }

    // Repeat the check for Follower neighbors
    if (isInState({State::Marker})) {
        for (int label : uniqueLabels()) {
            if (hasNbrAtLabel(label)) {
                auto& nbr = nbrAtLabel(label);
                if (nbr.followDir2 != -1) {
                    if (nbr.isInState({State::Follower}) && !pointsAtMe(nbr, nbr.followDir2)) {
                        state = State::Follower;
                        followDir1 = labelToDir(label);
                        followDir2 = labelToDir(label);
                        break;
                    }
                }
            }
        }
    }

    // If the particle is still in the Marker state, try to perform a movement
    if (state == State::Marker) {
        bool connected = false;

        updateBorderPointColors();

        while (safeToMove()){

            if (safeToMove()) {  // Check if the particle is safe to move
                int currentDir = followDir2;

                // Loop through all possible counterclockwise directions
                for (int i = 0; i < 6; ++i) {
                    int nextDir = nextCounterclockwiseDir(currentDir);

                    // Check if the particle can expand in the next counterclockwise direction
                    if (!hasNbrAtLabel(nextDir) && canExpand(nextDir) && safeToMove()) {
                        expand(nextDir);  // Expand in the valid direction
                        break;
                    }
                    currentDir = nextDir;  // Move to the next counterclockwise direction
                }

                // Check neighbors to see if connection can be made
                for (int label : randLabelsHead()) {
                    if (hasNbrAtLabel(label)) {
                        auto& nbr = nbrAtLabel(label);
                        if (isInState({State::Marker})) {
                            if (nbr.isInState({State::Leader}) && !pointsAtMe(nbr, nbr.followDir2)) {
                                state = State::Follower;
                                followDir1 = labelToDir(label);
                                followDir2 = labelToDir(label);
                                connected = true;
                                break;
                            }
                        }
                    }
                }
                if (connected) {
                    if (isExpanded()) {
                        contractTail();
                    }
                    return;
                }

                for (int label : randLabelsHead()) {
                    if (hasNbrAtLabel(label)) {
                        auto& nbr = nbrAtLabel(label);
                        if (isInState({State::Marker})) {
                            if (nbr.isInState({State::Follower}) && !pointsAtMe(nbr, nbr.followDir2)) {
                                state = State::Follower;
                                followDir1 = labelToDir(label);
                                followDir2 = labelToDir(label);
                                connected = true;
                                break;
                            }
                        }
                    }
                }
                if (connected) {
                    if (isExpanded()) {
                        contractTail();
                    }
                    return;
                }

                for (int label : randLabelsHead()) {
                    if (hasNbrAtLabel(label)) {
                        auto& nbr = nbrAtLabel(label);
                        if (isInState({State::Marker})) {
                            if (nbr.isInState({State::Immo})) {
                                followDir2 = labelToDir(label);
                                connected = true;
                                break;
                            }
                        }
                    }
                }

                if (connected) {
                    if (isExpanded()) {
                        contractTail();
                    }
                    return;
                }

                for (int label : randLabelsHead()) {
                    if (hasNbrAtLabel(label)) {
                        auto& nbr = nbrAtLabel(label);
                        if (isInState({State::Marker})) {
                            if (nbr.isInState({State::Marker}) && !pointsAtMe(nbr, nbr.followDir2)) {
                                followDir2 = labelToDir(label);
                                connected = true;
                                break;
                            }
                        }
                    }
                }

                // Handle connection or backtracking
                if (connected) {
                    if (isExpanded()) {
                        contractTail();
                    }
                    return;
                } else {
                    if (isExpanded()) {
                        contractHead();
                    }

                    // Check the state, safety, and expansion conditions
                    if (isInState({State::Marker}) && safeToMove()) {
                        int currentDir = followDir2;
                        int nextDir = nextCounterclockwiseDir(currentDir);

                        // If canExpand returns false and no neighbor exists at nextDir
                        if (hasNbrAtLabel(nextDir) && !canExpand(nextDir)) {
                            followDir2 = nextDir;
                        }
                    }
                }
            }
        }
    }

    // Transition to the next phase if movement is complete
    updateParticleStatesAndPhases();
    // if (hasCompletedperformMarkerMovement()) {
    //     phase = Phase::LeaderMovement;
    // }
}


bool Immobilizedparticles::safeToMove() const {
    // Check if the particle has no children and is in the Marker state
    if (isInState({State::Marker}) && isLeaf()) {
        return true;
    }

    // Check if all descendants are in the Immo state
    if (isInState({State::Marker}) && allDescendantsImmo()) {
        return true;
    }
    for (int label : childLabels2()) {
        if (hasNbrAtLabel(label)) {
            const auto& child = nbrAtLabel(label);
            if (child.isInState({State::Marker})) {
                // Ensure child marker has moved
                return false;  // Child marker hasn't moved yet
            }
        }
    }
    // If the particle is in a loop, it's not safe to move
    if (isInState({State::Marker}) && isInLoop()) {
        return true; // If in a loop, it's  safe to move
    }
    // Ensure child markers have moved before the parent marker moves


    // If none of the conditions are met, the particle is not safe to move
    return false;
}

bool Immobilizedparticles::isLeaf() const {
    for (int label : uniqueLabels()) {
        if (hasNbrAtLabel(label)) {
            auto& nbr = nbrAtLabel(label);
            if(nbr.followDir2 != -1){
                if (pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir2))) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool Immobilizedparticles::allDescendantsImmo() const {
    for (int label : childLabels2()) {
        if (hasNbrAtLabel(label)) {
            const auto& child = nbrAtLabel(label);

            if (!child.isInState({State::Immo})) {
                return false;
            }
            if (!child.allDescendantsImmo()) {
                return false;
            }

        }
    }
    return true;
}

bool Immobilizedparticles::isInLoop() const {
    return isInLoopHelper(this);
}


bool Immobilizedparticles::isInLoopHelper(const Immobilizedparticles* root) const {
    for (int label : childLabels2()) {
        if (hasNbrAtLabel(label)) {
            const auto& child = nbrAtLabel(label);
            if (&child == root) {
                return true; // Current particle is a descendant (loop detected)
            }
            // Recursively check the child's descendants
            if (child.isInLoopHelper(root)) {
                return true;
            }
        }
    }
    return false;
}

std::vector<int> Immobilizedparticles::childLabels2() const {
    std::vector<int> children;

    for (int label : uniqueLabels()) {
        if (hasNbrAtLabel(label)) {
            const auto& nbr = nbrAtLabel(label);
            if(nbr.followDir2 != -1){
                if (pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir2))) {
                    children.push_back(label);
                }
            }
        }
    }

    return children;
}


bool Immobilizedparticles::hasCompletedperformMarkerMovement() const {

    // Token to track whether any particle is still in the Marker state
    if(isInState({State::Marker})) {
        return false;
    }

    for (int label : uniqueLabels()) {
        // Check if there is a particle at the current label
        if (hasNbrAtLabel(label)) {
            // Get the particle at the current label
            const auto& particle = nbrAtLabel(label);

            // Check if the particle is in the Marker state
            if (particle.isInState({State::Marker})) {
                return false;
            }
        }
    }
    return true;
}





//----------------------------Phase 4----------------------------
//----------------------------Leader Movement----------------------------




void Immobilizedparticles::performLeaderMovement() {
    // Check if all particles in the neighborhood are in target states
    if (areAllParticlesInTargetStates()) {
        if (leaderToken) {
            tryToBecomeLeader();
            return;
        } else if (isInState({State::Leader})) {
            if (moveDir == -1) {
                moveDir = randDir();
            }

            // Check if all Marker and Idle particles are Followers
            if (!areAllMarkerAndIdleParticlesFollowers()) {
            } else {
                if (isContracted() && !canExpand(dirToHeadLabel(moveDir))) {
                    bool changed = false;
                    for (int label : uniqueLabels()) {
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
                        int label = labelOfFirstNbrInState({State::Follower});
                        if (label >= 0) {
                            passLeaderToken(label);
                            moveDir = -1;
                        } else {
                            // This case should only happen if the Leader is isolated and surrounded by immobilized particles
                        }
                    }
                }
            }
        }

        // Perform movement if the particle is not in any of the following states
        if (!isInState({State::Idle, State::Immo, State::Marker, State::Root, State::Retired, State::Seed, State::FollowerHex})) {
            performMovement();
        }

        // Move particle away from target tree
        moveAwayTargettree();
    }


    // Switch phase to HexagonFormation if conditions are met
    auto* immobilizedSystem = dynamic_cast<ImmobilizedParticleSystem*>(&system);
    if (immobilizedSystem->checkAndSwitchToHexagonFormationPhase()) {
        phase = Phase::HexagonFormation;
    }
}

void Immobilizedparticles::tryToBecomeLeader() {

    // Try to become the leader by expanding
    for (int label : uniqueLabels()) {
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
            return;
        }
    }

    // If no suitable follower was found, use default behavior to pass the leader token
    moveDir = -1;

    int label = 0;
    for (int l : uniqueLabels()) {
        if (labelToDir(l) == tokenCurrentDir) {
            label = l;
        }
    }

    int numLabels = isContracted() ? 6 : 10;

    if (passForward) {
        // Try to pass the token towards the designated token direction.
        while (hasNbrAtLabel(label) && nbrAtLabel(label).isInState({State::Immo})) {
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
        } while (!hasNbrAtLabel(label) || nbrAtLabel(label).isInState({State::Immo}));
        while (hasNbrAtLabel(label) && nbrAtLabel(label).isInState({State::Immo})) {
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


    state = State::Follower;
    followDir1 = labelToDir(label);
    if (tokenForwardDir < 0) {
        passForward = true;
        tokenForwardDir = followDir1;
    }

    auto& nbr = nbrAtLabel(label);
    nbr.leaderToken = true;
    nbr.tokenForwardDir = dirToNbrDir(nbr, tokenForwardDir);
    nbr.tokenCurrentDir = dirToNbrDir(nbr, followDir1);
    nbr.passForward = passForward;
    if (nbr.tokenForwardDir == nbr.tokenCurrentDir && tokenForwardDir != tokenCurrentDir) {
        if (randBool()) {
            nbr.passForward = true;
        }
    }
    leaderToken = false;
    tokenForwardDir = -1;
    tokenCurrentDir = -1;
    passForward = false;
    updateBorderPointColors();

}

void Immobilizedparticles::performMovement() {
    if (areAllMarkerAndIdleParticlesFollowers()) {
        if (isExpanded() && isInState({State::Follower, State::Leader}) && !hasBlockingTailNbr()) {
            contractTail();
        } else if (isContracted() && isInState({State::Follower}) && hasTailAtLabel(dirToHeadLabel(followDir1))) {
            int followLabel = dirToHeadLabel(followDir1);
            auto& nbr = nbrAtLabel(followLabel);
            int nbrContractionDir = nbrDirToDir(nbr, (nbr.tailDir() + 3) % 6);
            push(followLabel);
            followDir1 = nbrContractionDir;
            followDir2 = nbrContractionDir;
        } else if (isInState({State::Leader}) && isContracted() && !(freeState && lineState) && canExpand(dirToHeadLabel(moveDir))) {
            expand(dirToHeadLabel(moveDir));
        }
        moveAwayTargettree();
        updateBorderPointColors();
    }
}

bool Immobilizedparticles::hasBlockingTailNbr() const {
    for (int label : uniqueLabels()) {
        if (hasNbrAtLabel(label)) {
            auto& nbr = nbrAtLabel(label);
            if (nbr.isInState({State::Idle}) || (nbr.isInState({State::Follower}) && pointsAtMyTail(nbr, nbr.dirToHeadLabel(nbr.followDir1)))) {
                return true;
            }
        }
    }

    return false;
}


bool Immobilizedparticles::areAllMarkerAndIdleParticlesFollowers() {
    auto* immobilizedSystem = dynamic_cast<ImmobilizedParticleSystem*>(&system);
    if (!immobilizedSystem) {
        return false;
    }

    for (const auto& particle : immobilizedSystem->getParticles()) {
        auto* immobileParticle = dynamic_cast<Immobilizedparticles*>(particle);
        if (immobileParticle) {
            if (immobileParticle->isInState({State::Marker, State::Idle})) {
                return false;
            }

        }
    }
    return true;
}


void Immobilizedparticles::moveAwayTargettree() {
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
            //if(followDir1 != -1){
            if (nbr.isInState({State::Follower}) && pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir1))) {
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
        auto& nbr = nbrAtLabel(dirToHeadLabel(followDir1));
        if (nbr.isInState({State::Leader}) && nbr.moveDir >= 0) {
            parentDir = nbrDirToDir(nbr, nbr.moveDir);
        } else if (nbr.isInState({State::Follower})) {
            parentDir = nbrDirToDir(nbr, nbr.followDir1);
        }

        if (parentDir < 0 || parentDir != followDir1) {
            lineState = false;
            return;
        }
    }

    lineState = true;
    for (int label : uniqueLabels()) {
        if (hasNbrAtLabel(label)) {
            auto& nbr = nbrAtLabel(label);
            if (nbr.isInState({State::Follower}) && pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir1))) {
                lineState = lineState && nbr.lineState;
            }
        }
    }
}



bool ImmobilizedParticleSystem::checkAndSwitchToHexagonFormationPhase() {
    bool leaderExists = false;

    // First pass: Check if all particles are in the appropriate states and conditions are met
    for (auto p : particles) {
        auto hp = dynamic_cast<Immobilizedparticles*>(p);
        if (!hp) continue;

        // Check if the particle is in an inappropriate state or condition
        if (hp->isInState({Immobilizedparticles::State::Idle}) ||
            hp->isInState({Immobilizedparticles::State::Marker}) ||
            !hp->freeState || !hp->lineState || hp->isExpanded()) {
            return false;
        }

        // Check if there is a Leader or Follower particle
        if (hp->isInState({Immobilizedparticles::State::Leader}) || hp->isInState({Immobilizedparticles::State::Follower})) {
            if (hp->isInState({Immobilizedparticles::State::Leader})) {
                leaderExists = true;
            }
        }
    }

    //Second pass: If a leader exists and all conditions are met, update phase and switch to HexagonFormation phase
    if (leaderExists) {
        for (auto p : particles) {
            auto hp = dynamic_cast<Immobilizedparticles*>(p);
            if (!hp) continue;

            if (hp->isInState({Immobilizedparticles::State::Leader}) || hp->isInState({Immobilizedparticles::State::Follower})) {
                // Update the phase of each particle to HexagonFormation
                hp->phase = Immobilizedparticles::Phase::HexagonFormation;
            }
        }
        return true;
    }

    return false;
}

bool Immobilizedparticles::areAllParticlesInTargetStates() const {
    // Check the state of the current particle
    if (!isInState({State::Immo, State::Leader, State::Follower})) {
        return false;
    }

    // Iterate through neighbors and check their states
    for (int label : uniqueLabels()) {
        if (hasNbrAtLabel(label)) {
            const auto& neighbor = nbrAtLabel(label);
            if (!neighbor.isInState({State::Immo, State::Leader, State::Follower})) {
                return false;
            }
        }
    }

    return true;
}



//----------------------------End Leader Movement----------------------------




//----------------------------Phase 5----------------------------
//----------------------------Hexagon Formation----------------------------

void Immobilizedparticles::processactivateHex(){
    if(isInState({State::FollowerHex,State::Lead,State::Seed, State::Finish})){
        activateHex();
    }
    if(isInState({State::Leader})){
        state = State::Seed;
    }
    if(isInState({State::Follower})){
        state = State::FollowerHex;
    }
}

void Immobilizedparticles::activateHex() {

    if(isInState({State::Leader})){
        state = State::Seed;
    }
    if(isInState({State::Follower})){
        state = State::FollowerHex;
    }
    if (state == State::Seed){
        constructionDir = 0;
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
                followDir1 = labelOfFirstNbrInState({State::Lead, State::FollowerHex});
                return;
            }
        } else if (state == State::FollowerHex) {
            if (hasNbrInState({State::Seed, State::Finish})) {
                state = State::Lead;
                updateMoveDir();
                return;
            } else if(hasTailAtLabel(followDir1)) {
                auto nbr = nbrAtLabel(followDir1);
                int nbrContractionDir = nbrDirToDir(nbr, (nbr.tailDir() + 3) % 6);
                push(followDir1);
                followDir1 = nbrContractionDir;
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
               pointsAtMyTail(p, p.dirToHeadLabel(p.followDir1));
    };
    return labelOfFirstNbrWithProperty<Immobilizedparticles>(prop) != -1;
}

void Immobilizedparticles::updateConstructionDir() {
    // Hexagon construction.
    constructionDir = constructionReceiveDir();

    if (constructionDir < 0 || constructionDir >= 10) {
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
}

void Immobilizedparticles::updateMoveDir() {
    moveDir = labelOfFirstNbrInState({State::Seed, State::Finish});
    while (hasNbrAtLabel(moveDir) && (nbrAtLabel(moveDir).state == State::Seed ||
                                      nbrAtLabel(moveDir).state == State::Finish))
    {
        moveDir = (moveDir + 5) % 6;
    }
}

bool Immobilizedparticles::canFinish() const {
    return constructionReceiveDir() != -1;
}

int Immobilizedparticles::constructionReceiveDir() const {
    auto prop = [&](const Immobilizedparticles& p) {

        // Check if the constructionDir is valid
        if (p.constructionDir < 0 || p.constructionDir >= 10) {
            return false;
        }

        // Check if the particle is in a contracted state and has the appropriate neighbor state
        bool isNeighborPointingAtMe = pointsAtMe(p, p.dirToHeadLabel(p.constructionDir));
        bool validNeighborState = (p.state == State::Seed || p.state == State::Finish);

        return isContracted() && validNeighborState && isNeighborPointingAtMe;
    };

    int result = labelOfFirstNbrWithProperty<Immobilizedparticles>(prop);

    return result;
}


//----------------------------End Hexagon Formation----------------------------


//----------------------------Additional Functions----------------------------


void Immobilizedparticles::updateParticleStatesAndPhases() {
    // Update particle state for LeaderElection phase
    if (phase == Immobilizedparticles::Phase::LeaderElection) {
        for (int label : uniqueLabels()) {
            if (hasNbrAtLabel(label)) {

                if (isInState({Immobilizedparticles::State::Idle})) {
                    isIdle = true;
                }

                if (isInState({Immobilizedparticles::State::Immo})) {
                    isImmo = true;
                }
            }
        }
    }
    // Update particle states to InitializeTrees phase
    if(phase == Immobilizedparticles::Phase::LeaderElection){
    if(isInState({State::Leader}) || isInState({State::Finished}))
    {
        phase = Phase::InitializeTrees;
    }}

    // Update particle states for InitializeTrees phase
    if (phase == Immobilizedparticles::Phase::InitializeTrees) {
        if (isIdle && state != Immobilizedparticles::State::Leader) {
            state = Immobilizedparticles::State::Idle;
            headMarkColor();
        } else if (isImmo && state != Immobilizedparticles::State::Leader) {
            state = Immobilizedparticles::State::Immo;
            headMarkColor();
        }
    }


    // If followDir2 is valid, transition to CompleteTargetTree phase
    if (followDir2 != -1) {
        if (phase == Immobilizedparticles::Phase::InitializeTrees) {
            phase = Immobilizedparticles::Phase::CompleteTargetTree;
        }
    }

    if(state != Immobilizedparticles::State::Idle && phase == Immobilizedparticles::Phase::CompleteTargetTree){
        if (hasCompletedperformMarkerMovement()) {
            phase = Phase::LeaderMovement;
        }
    }
}


std::vector<int> Immobilizedparticles::childLabels() const {
    std::vector<int> children;
    for (int label : uniqueLabels()) {
        if (hasNbrAtLabel(label)) {
            const auto& nbr = nbrAtLabel(label);

            if(nbr.followDir2 != -1){
                if (nbr.followDir2 == label) {
                    children.push_back(label);
                }}
        }
    }
    return children;
}


int Immobilizedparticles::nextClockwiseDir(int inputDir) {
    return (inputDir + 1) % 6;
}

int Immobilizedparticles::nextCounterclockwiseDir(int inputDir) const{
    return (inputDir +5) % 6;
}

std::vector<int> Immobilizedparticles::randLabelsHead() {
    std::vector<int> result = this->headLabels();
    RandomNumberGenerator::shuffle(std::begin(result), std::end(result));
    return result;
}


std::vector<int> Immobilizedparticles::randLabels() {
    std::vector<int> result = uniqueLabels();
    RandomNumberGenerator::shuffle(std::begin(result), std::end(result));
    return result;
}


QString Immobilizedparticles::inspectionText() const {
    QString text;
    QString indent = "    ";
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
        case State::Marker:  return "Marker";
        case State::Single:  return "Single";  // Hex Formation
        case State::Seed:     return "Seed";  // Hex Formation
        case State::FollowerHex:  return "FollowerHex";  // Hex Formation
        case State::Retired:  return "Retired";  // Hex Formation
        case State::Lead:   return "lead";  // Hex Formation
        case State::Finish: return "finish"; // Hex Formation
        case State::Candidate:      return "candidate"; // leader election
        case State::SoleCandidate:  return "sole candidate"; // leader election
        case State::Demoted:        return "demoted"; // leader election
        case State::Finished:       return "finished"; // leader election
        default:              return "no state";
        }
    }();
    text += "\n";
    text += "phase: ";
    text += [this]() {
        switch (phase) {
        case Phase::LeaderElection :   return "LeaderElection";
        case Phase::InitializeTrees :   return "InitializeTrees";
        case Phase::CompleteTargetTree :   return "CompleteTargetTree";
        case Phase::CheckTargetTree :   return "CheckTargetTree";
        case Phase::LeaderMovement :   return "LeaderMovement";
        case Phase::HexagonFormation :   return "HexagonFormation";

        default:              return "no phase";
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

    text += "allDescendantsImmo: " + QString(allDescendantsImmo() ? "true" : "false") + "\n";
    text += "isLeaf: " + QString(isLeaf() ? "true" : "false") + "\n";
    text += "SafetoMove: " + QString(safeToMove() ? "true" : "false") + "\n";

    text += "hasnbratlabel: " + QString( hasNbrAtLabel(nextCounterclockwiseDir(followDir2)) ? "true" : "false") + "\n";
    text += "Canexpand: " + QString(canExpand(nextCounterclockwiseDir(followDir2)) ? "true" : "false") + "\n";

    text += "\n";
    text += "constructionDir: " + QString::number(constructionDir) + "\n";
    text += "moveDir: " + QString::number(moveDir) + "\n";
    text += "followDir: " + QString::number(followDir) + "\n";
    text += "followDir1: " + QString::number(followDir1) + "\n";
    text += "followDir2: " + QString::number(followDir2) + "\n";
    // for leader election
    text += "number of agents: " + QString::number(agents.size()) + "\n";
    for (ImmoLeaderElectionAgent* agent : agents) {
        text += [agent, indent](){
            switch(agent->agentState) {
            case State::Demoted:        return indent + "demoted\n";
            case State::Candidate:
                switch(agent->subPhase) {
                case Immobilizedparticles::ImmoLeaderElectionAgent::SubPhase::
                    SegmentComparison:      return indent + "segment comparison\n";
                case Immobilizedparticles::ImmoLeaderElectionAgent::SubPhase::
                    CoinFlipping:           return indent + "coin flipping\n";
                case Immobilizedparticles::ImmoLeaderElectionAgent::SubPhase::
                    SolitudeVerification:   return indent + "solitude verification\n";
                }
            case State::SoleCandidate:  return indent + "sole candidate\n";
            default: return indent + "invalid\n";
            }
        }();
        text += indent + indent + "agent dir: " + QString::number(agent->agentDir) +
                "\n";
        text += indent + indent + "next agent dir: " +
                QString::number(agent->nextAgentDir) + "\n";
        text += indent + indent + "prev agent dir: " +
                QString::number(agent->prevAgentDir) + "\n";
    }
    text += "has leader election tokens: " +
            QString::number(hasToken<LeaderElectionToken>()) + "\n";
    text += "has solitude active token: " +
            QString::number(hasToken<SolitudeActiveToken>()) + "\n";
    text += "has " + QString::number(countTokens<SolitudePositiveXToken>()) +
            " positive x tokens\n";
    text += "has " + QString::number(countTokens<SolitudeNegativeXToken>()) +
            " negative x tokens\n";
    text += "has " + QString::number(countTokens<SolitudePositiveYToken>()) +
            " positive y tokens\n";
    text += "has " + QString::number(countTokens<SolitudeNegativeYToken>()) +
            " negative y tokens\n";
    text += "has passive clean token: " +
            QString::number(hasToken<PassiveSegmentCleanToken>()) + "\n";

    text += "\n";
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
    return std::find(states.begin(), states.end(), state) != states.end();
}




bool Immobilizedparticles::canRetire() const {
    auto prop = [&](const Immobilizedparticles& p) {
        return (p.state == State::Seed || p.state == State::Retired)
               && pointsAtMe(p, p._hexagonDir);
    };

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

//----------------------------Particle Apperance----------------------------

int Immobilizedparticles::particleColor() const {
    if (leaderToken) {
        return 0x39FF14; // Green
    } else if (isInState({State::Immo})) {
        return 0x000000; // Black
    } else if (isInState({State::Idle}) || lineState) {
        return 0x000000; // Black
    } else if (freeState) {
        return 0x696969; // Grey
    }
    return 0xffffff; // White
}

int Immobilizedparticles::headMarkColor() const {
    if (state == State::Leader) {
        return 0x00ff00; // Green
    } else if (state == State::Immo) {
        return 0xFF0000; // Red
    } else if (state == State::Idle) {
        return 0x000000; // Black
    } else if (state == State::Follower) {
        return 0x0000FF; // Blue
    } else if (state == State::Marker) {
        return 0xFF8800; // Fluorescent Orange
    } else if (state == State::Seed) {
        return 0xFFFF00; // yellow
    } else if (state == State::Root) {
        return 0xFF0000; // Red
    } else if (state == State::Retired) {
        return 0x808080; // Grey
    } else if (state == State::FollowerHex) {
        return 0x8B4513; // Brown
    } else if (state == State::Finish) {
        return 0xa100ff; // Purple
    }else if (state == State::Lead) {
        return 0x696969; // Purple
    }
    return -1;
}

int Immobilizedparticles::headMarkDir() const {
    // This function is used when a particle needs a direction marker at its head.
    // Here, it is used to visualise the particle's follow / movement direction.

    if (isInState({State::Follower})) {
        return followDir1;
    }
    else if(isInState({State::Idle})){
        return followDir2;
    }
    else if(isInState({State::Immo})){
        return followDir2;
    }
    else if(isInState({State::Marker})){
        return followDir2;
    }
    else if (isInState({State::Seed})){
        return moveDir;
    }
    else if (isInState({State::FollowerHex})){
        return followDir1;
    }
    else if (isInState({State::Finish})){
        return constructionDir;
    }
    else if (state == State::Leader && ( phase == Phase::LeaderMovement || phase == Phase::HexagonFormation)) {
        return moveDir;
    }
    else if (state == State::Leader) {
        return -1;
    }
    else{
        return followDir2;
    }
}

int Immobilizedparticles::tailMarkColor() const {
    return headMarkColor();
}

std::array<int, 18> Immobilizedparticles::borderColors() const {
    return _leaderborderColorLabels;
}

std::array<int, 6> Immobilizedparticles::borderPointColors() const {
    return _leaderborderPointColorLabels;
}

void Immobilizedparticles::updateBorderPointColors(){
    _leaderborderPointColorLabels.fill(-1);
    // Check if the particle is in the Follower state and the phase is either InitializeTrees or CompleteTargetTree
    if (isInState({State::Follower})   && (phase == Phase::InitializeTrees || phase == Phase::CompleteTargetTree)){  // && !isExpanded()) {
        if (followDir1 != -1) {
            _leaderborderPointColorLabels[localToGlobalDir(followDir1)] = 0x0000ff; // Blue for followDir1
        }
    }
    if (leaderToken) {
        Q_ASSERT(tokenCurrentDir >= 0);
        Q_ASSERT(tokenForwardDir >= 0);
        _leaderborderPointColorLabels[localToGlobalDir(tokenCurrentDir)] = 0xa0a0a0; // light grey
        if (passForward) {
            _leaderborderPointColorLabels[localToGlobalDir(tokenForwardDir)] = 0x00cc00; // green
        } else {
            _leaderborderPointColorLabels[localToGlobalDir(tokenForwardDir)] = 0xff0000; // red
        }
    }
}

//----------------------------Particle System----------------------------

ImmobilizedParticleSystem::ImmobilizedParticleSystem(int numParticles, int numImmoParticles, int genExpExample, int numCoinFlips) {
    Q_ASSERT(numParticles > 0);
    Q_ASSERT(numImmoParticles >= 0);
    Q_ASSERT(genExpExample == 0 || genExpExample == 1);
    Q_ASSERT(numCoinFlips > 0);

    std::set<Node> occupied;
    std::set<Node> candidates;

    _seedOrientation = randDir();

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
            insert(new Immobilizedparticles(randomCandidate, -1,randDir(), *this, Immobilizedparticles::State::Idle));
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
                Immobilizedparticles* immoParticle = new Immobilizedparticles(randomCandidate, -1,randDir(), *this, Immobilizedparticles::State::Immo);
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


bool ImmobilizedParticleSystem::hasTerminated() const {
#ifdef QT_DEBUG
    if (!isConnected(particles)) {
        return true;
    }
#endif
    for (auto p : particles) {
        auto hp = dynamic_cast<Immobilizedparticles*>(p);
        if (hp && hp->state != Immobilizedparticles::State::Seed &&
            hp->state != Immobilizedparticles::State::Finish &&
            hp->state != Immobilizedparticles::State::Immo) {
            return false;
        }
    }

    printf("Termination condition met: All particles are in Seed, Retired, or Immo states.\n");
    return true;
}
