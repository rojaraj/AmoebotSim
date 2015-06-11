#include <set>
#include <random>

#include "compact2.h"
#include "sim/particle.h"
#include "sim/system.h"

namespace Compact2
{
Compact2Flag::Compact2Flag()
    : isParent(false),
      oldFollowDir(-1)
{
}

Compact2Flag::Compact2Flag(const Compact2Flag& other)
    : Flag(other),
      state(other.state),
      isParent(other.isParent),
      oldFollowDir(other.oldFollowDir)
{
}

Compact2::Compact2(const State _state)
    : state(_state),
      followDir(-1)
{
    setState(_state);
}

Compact2::Compact2(const Compact2& other)
    : AlgorithmWithFlags(other),
      state(other.state),
      followDir(other.followDir)
{
}

Compact2::~Compact2()
{
}

System* Compact2::instance(const unsigned int size, const double holeProb)
{
    System* system = new System();
    std::set<Node> occupied, candidates;

    // Create Seed Particle
    system->insert(Particle(new Compact2(State::Seed), randDir(), Node(0,0), -1));
    occupied.insert(Node(0,0));

    for(int dir = 0; dir<6; dir++){
        candidates.insert(Node(0,0).nodeInDir(dir));
    }

    while(occupied.size() < size && !candidates.empty()){
        auto index = randInt(0, candidates.size());
        auto it = candidates.begin();
        while (index != 0){
            ++it;
            index--;
        }

        Node head = *it;
        candidates.erase(it);
        occupied.insert(head);

        if(randBool(holeProb)){
            continue;
        }

        for(int dir = 0; dir < 6; dir++){
            auto neighbor = head.nodeInDir(dir);
            if(occupied.find(neighbor) == occupied.end() && candidates.find(neighbor) == candidates.end()){
                candidates.insert(neighbor);
            }
        }
        // Insert new idle particle
        system->insert(Particle(new Compact2(State::Idle), randDir(), head, -1));
    }
    return system;
}

Movement Compact2::execute()
{
    if(state == State::Seed) {
        return Movement(MovementType::Empty);
    }

    if(isContracted()) {
        if(state == State::Idle) {
            followDir = neighborInStateDir(State::Seed);
            if(followDir == -1) {
                followDir = neighborInStateDir(State::Active);
            }
            if(followDir == -1) {
                return Movement(MovementType::Idle);
            } else {
                setState(State::Active);
                headMarkDir = followDir;
                setParentLabel(followDir);
                return Movement(MovementType::Idle);
            }
        } else if(state == State::Active) {
            if(hasNeighborInState(State::Idle)) {
                return Movement(MovementType::Idle);
            }
            // become a follower of leader
            if(inFlags[followDir]->state == State::Leader && !inFlags[followDir]->fromHead) {
                setState(State::Follower);
                int expandDir = followDir;
                followDir = (followDir + inFlags[followDir]->oldFollowDir - inFlags[followDir]->dir + 9) % 6;
                headMarkDir = followDir;
                setParentLabel(dirToHeadLabelAfterExpansion(followDir, expandDir));
                return Movement(MovementType::Expand, expandDir);
            }
            // become a follower of follower
            if(inFlags[followDir]->state == State::Follower && !inFlags[followDir]->fromHead) {
                setState(State::Follower);
                int expandDir = followDir;
                followDir = (followDir + inFlags[followDir]->tailDir - inFlags[followDir]->dir + 6) % 6;
                headMarkDir = followDir;
                setParentLabel(dirToHeadLabelAfterExpansion(followDir, expandDir));
                return Movement(MovementType::Expand, expandDir);
            }
            // become a leader
            if(isParent() && !isLocallyCompact()) {
                setState(State::Leader);
                setOldFollowDir(followDir);
                auto dir = emptyNeighborDir();
                int tailDirAfterExpansion = (dir + 3) % 6;
                followDir = tailDirAfterExpansion;
                headMarkDir = tailDirAfterExpansion;
                unsetParentLabel();
                return Movement(MovementType::Expand, dir);
            }
        }
    } else { // isExpanded
        if(state == State::Leader) {
            setState(State::Active);
            setParentLabel(followDir);
            return Movement(MovementType::HandoverContract, tailContractionLabel());
        } else if(state == State::Follower) {
            if(tailHasChild()) {
                setState(State::Active);
                setParentLabel(followDir);
                return Movement(MovementType::HandoverContract, tailContractionLabel());
            } else if(leaderAsChild()) {
                return Movement(MovementType::Idle);
            } else {
                setState(State::Active);
                setParentLabel(followDir);
                return Movement(MovementType::Contract, tailContractionLabel());
            }
        }
    }

    return Movement(MovementType::Idle);
}

Algorithm* Compact2::clone()
{
    return new Compact2(*this);
}

bool Compact2::isDeterministic() const
{
    return true;
}

void Compact2::setState(State _state)
{
    state = _state;
    if(state == State::Seed) {
        headMarkColor = 0x00ff00; tailMarkColor = 0x00ff00; // Green
    } else if(state == State::Leader) {
        headMarkColor = 0xcc0000; tailMarkColor = 0xcc0000; // Red
    } else if(state == State::Follower) {
        headMarkColor = 0x3366FF; tailMarkColor = 0x3366FF; // Blue
    } else if(state == State::Active) {
        headMarkColor = 0x505050; tailMarkColor = 0xe0e0e0; // Grey
    } else { // phase == Phase::Idle
        headMarkColor = -1; tailMarkColor = -1; // No color
    }

    for(int i = 0; i < 10; i++) {
        outFlags[i].state = state;
    }
}

void Compact2::setOldFollowDir(int dir)
{
    Q_ASSERT(0 <= dir && dir < 6);
    for(int label = 0; label < 10; label++) {
        outFlags[label].oldFollowDir = dir;
    }
}

void Compact2::setParentLabel(int parentLabel)
{
    Q_ASSERT(0 <= parentLabel && parentLabel < 10);

    for(int label = 0; label < 10; label++) {
        outFlags[label].isParent = (label == parentLabel);
    }
}

void Compact2::unsetParentLabel() {
    for(int label = 0; label < 10; label++) {
        outFlags[label].isParent = false;
    }
}

bool Compact2::hasNeighborInState(State _state)
{
    for(int label = 0; label < 10; label++) {
        if(inFlags[label] != nullptr) {
            const Compact2Flag& flag = *inFlags[label];
            if(flag.state == _state) {
                return true;
            }
        }
    }
    return false;
}

int Compact2::neighborInStateDir(State _state)
{
    Q_ASSERT(isContracted());
    for(int dir = 0; dir < 6; dir++) {
        if(inFlags[dir] != nullptr) {
            const Compact2Flag& flag = *inFlags[dir];
            if(flag.state == _state) {
                return dir;
            }
        }
    }
    return -1;
}

int Compact2::emptyNeighborDir()
{
    Q_ASSERT(isContracted());
    Q_ASSERT(followDir != -1);

    // FIX!
    for(int offset = 0; offset < 6; offset++) {
        int dir = (followDir + offset) % 6;
        if(inFlags[dir] == nullptr) {
            return dir;
        }
    }

    Q_ASSERT(false);
    return -1;
}

int Compact2::determineFollowDir()
{
    for(int label = 0; label < 10; label++) {
        if(inFlags[label] != nullptr) {
            const Compact2Flag& flag = *inFlags[label];
            if(flag.state == State::Leader || flag.state == State::Follower) {
                return labelToDir(label);
            }
        }
    }
    return -1;
}

bool Compact2::isLocallyCompact()
{
    Q_ASSERT(isContracted());

    int count = 0;
    for(int dir = 0; dir < 6; dir++) {
        if(inFlags[dir] != nullptr) {
            count++;
        }
    }

    if(0 == count || count == 6) {
        return true;
    } else if(count == 5) {
        return false;
    }

    int firstOccupiedDir = -1;
    for(int dir = 0; dir < 6; dir++) {
        if(inFlags[dir] != nullptr && inFlags[(dir + 5) % 6] == nullptr) {
            firstOccupiedDir = dir;
            break;
        }
    }

    int adjacentCount = 0;
    for(int offset = 0; offset < 6; offset++) {
        int dir = (firstOccupiedDir + offset) % 6;
        if(inFlags[dir] != nullptr) {
            adjacentCount++;
        } else {
            break;
        }
    }

    return adjacentCount == count;
}

bool Compact2::isParent()
{
    for(int label = 0; label < 10; label++) {
        if(inFlags[label] != nullptr && inFlags[label]->isParent) {
            return true;
        }
    }
    return false;
}

bool Compact2::tailHasChild()
{
    for(auto it = tailLabels().cbegin(); it != tailLabels().cend(); ++it) {
        auto label = *it;
        if(inFlags[label] != nullptr && inFlags[label]->isParent) {
            return true;
        }
    }
    return false;
}

bool Compact2::leaderAsChild()
{
    for(auto it = tailLabels().cbegin(); it != tailLabels().cend(); ++it) {
        auto label = *it;
        if(inFlags[label] != nullptr && inFlags[label]->state == State::Leader && inFlags[label]->dir == inFlags[label]->oldFollowDir) {
            return true;
        }
    }
    return false;
}

}
