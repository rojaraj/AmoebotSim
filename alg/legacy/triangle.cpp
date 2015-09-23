#include <set>
#include <random>
#include <QTime>
#include "alg/legacy/legacysystem.h"
#include "alg/legacy/triangle.h"
#include "sim/particle.h"
#include "sim/system.h"

namespace Triangle
{
TriFlag::TriFlag()
    : point(false),
      side(false),
      followIndicator(false)
{
}

TriFlag::TriFlag(const TriFlag& other)
    : Flag(other),
      state(other.state),
      point(other.point),
      side(other.side),
      followIndicator(other.followIndicator)
{
}

Triangle::Triangle(const State _state)
    : state(_state),
      followDir(-1)
{
    if (_state == State::Seed){
        outFlags[0].point = true;
        headMarkDir = 0;
    }
    setState(_state);
}


Triangle::Triangle(const Triangle& other)
    : AlgorithmWithFlags(other),
      state(other.state),
      followDir(other.followDir)
{
}

Triangle::~Triangle()
{
}

std::shared_ptr<LegacySystem> Triangle::instance(const unsigned int size, const double holeProb)
{
    std::shared_ptr<LegacySystem> system = std::make_shared<LegacySystem>();
    std::set<Node> occupied, candidates;

    // Create Seed Particle
    system->insertParticle(Particle(std::make_shared<Triangle>(State::Seed), randDir(), Node(0,0), -1));
    occupied.insert(Node(0,0));

    for(int dir = 0; dir<6;dir++){
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
        system->insertParticle(Particle(std::make_shared<Triangle>(State::Idle), randDir(), head, -1));
    }
    return system;
}

Movement Triangle::execute()
{
    if(isExpanded()){
        if(state == State::Follower) {
            setFollowIndicatorLabel(followDir);
        }
        if(hasNeighborInState(State::Idle) || (tailReceivesFollowIndicator() && (followIndicatorMatchState(State::Follower) || (followIndicatorMatchState(State::Leader) && state != State::Follower)))) {
            return Movement(MovementType::HandoverContract, tailContractionLabel());
        } else {
            return Movement(MovementType::Contract, tailContractionLabel());
        }
    } else {
        if (state == State::Idle){
            if (hasNeighborInState(State::Finished) || hasNeighborInState(State::Seed)){
                setState(State::Leader);
                return Movement(MovementType::Idle);
            }
            auto label = std::max(firstNeighborInState(State::Follower), firstNeighborInState(State::Leader));
            if(label != -1) {
                setState(State::Follower);
                followDir = labelToDir(label);
                setFollowIndicatorLabel(followDir);
                headMarkDir = followDir;
                return Movement(MovementType::Idle);
            } else {
                return Movement(MovementType::Empty);
            }
        } else if(state == State::Follower && !hasNeighborInState(State::Idle)) {
            if(hasNeighborInState(State::Finished)){
                setState(State::Leader);
                return Movement(MovementType::Idle);
            } else {
                if(inFlags[followDir]->isExpanded() && !inFlags[followDir]->fromHead) {
                    int expansionDir = followDir;
                    setContractDir(expansionDir);
                    followDir = updatedFollowDir();
                    headMarkDir = followDir;
                    auto temp = dirToHeadLabelAfterExpansion(followDir, expansionDir);
                    Q_ASSERT(labelToDirAfterExpansion(temp, expansionDir) == followDir);
                    setFollowIndicatorLabel(temp);
                    return Movement(MovementType::Expand, expansionDir);
                } else {
                    return Movement(MovementType::Empty);
                }
            }
        } else if(state == State::Leader && !hasNeighborInState(State::Idle)){
            headMarkDir = -1;
            int direction = isPointedAt();
            headMarkDir = direction;
            if(direction != -1){
                setState(State::Finished);
                // The following checks surroundings to ensure that the formation continues in a snaking pattern
                if(neighborInState((direction+4)%6, State::Finished) || neighborInState((direction+2)%6, State::Finished)){
                    outFlags[(direction+3)%6].point = true;
                } else{
                    if (!neighborInState((direction+1)%6, State::Finished) && !neighborInState((direction+5)%6, State::Finished) && !neighborInState((direction+5)%6, State::Seed)){
                        if (inFlags[direction]->side || neighborInState(direction,State::Seed)){
                            outFlags[(direction+5)%6].point = true;
                        } else {
                            outFlags[(direction+1)%6].point = true;
                        }
                    } else if (hasNeighborInState(State::Seed) || neighborInState((direction+5)%6, State::Finished)){
                        outFlags[(direction+2)%6].point = true;
                    } else if (neighborInState((direction+1)%6, State::Finished)){
                        outFlags[(direction+4)%6].point = true;
                        outFlags[(direction+4)%6].side = true;
                    }
                }
                return Movement(MovementType::Idle);
            } else {
                auto moveDir = getMoveDir();
                setContractDir(moveDir);
                headMarkDir = moveDir;
                return Movement(MovementType::Expand, moveDir);
            }
        } else {
            return Movement(MovementType::Empty);
        }
    }
}

std::shared_ptr<Algorithm> Triangle::blank() const
{
    return std::make_shared<Triangle>(State::Idle);
}

std::shared_ptr<Algorithm> Triangle::clone()
{
    return std::make_shared<Triangle>(*this);
}

bool Triangle::isDeterministic() const
{
    return true;
}

bool Triangle::isStatic() const
{
    return false;
}

int Triangle::isPointedAt(){
    for(int label = 0; label < 10; label++) {
        if(inFlags[label] != nullptr) {
            if(inFlags[label]->point){
                return label;
            }
        }
    }
    return -1;
}

void Triangle::setState(State _state)
{
    state = _state;
    if (state == State::Seed){
        headMarkColor = 0x00ff00; tailMarkColor = 0x00ff00; // Green
    }
    else if(state == State::Finished) {
        headMarkColor = 0x000000; tailMarkColor = 0x000000; // Black
    }
    else if(state == State::Leader) {
        headMarkColor = 0xff0000; tailMarkColor = 0xff0000; // Red
    }
    else if(state == State::Follower) {
        headMarkColor = 0x0000ff; tailMarkColor = 0x0000ff; // Blue
    }
    else { // phase == Phase::Idle
        headMarkColor = -1; tailMarkColor = -1; // No color
    }
    for(int i = 0; i < 10; i++) {
        outFlags[i].state = state;
    }
}

bool Triangle::neighborInState(int direction, State _state){
    Q_ASSERT(0 <= direction && direction <= 9);
    return (inFlags[direction] != nullptr && inFlags[direction]->state == _state);
}

bool Triangle::hasNeighborInState(State _state)
{
    return (firstNeighborInState(_state) != -1);
}

int Triangle::firstNeighborInState(State _state)
{
    for(int label = 0; label < 10; label++) {
        if(neighborInState(label, _state)) {
            return label;
        }
    }
    return -1;
}

int Triangle::getMoveDir()
{
    Q_ASSERT(isContracted());
    int objectDir = firstNeighborInState(State::Finished);
    if (hasNeighborInState(State::Seed)){
        objectDir = firstNeighborInState(State::Seed);
    }
    objectDir = (objectDir+5)%6;
    if (neighborInState(objectDir, State::Finished)){
        objectDir = (objectDir+5)%6;
    }
    return labelToDir(objectDir);
}

void Triangle::setContractDir(const int contractDir)
{
    for(int label = 0; label < 10; label++) {
        outFlags[label].contractDir = contractDir;
    }
}
int Triangle::updatedFollowDir() const
{
    int contractDir = inFlags[followDir]->contractDir;
    int offset = (followDir - inFlags[followDir]->dir + 9) % 6;
    int tempFollowDir = (contractDir + offset) % 6;
    Q_ASSERT(0 <= tempFollowDir && tempFollowDir <= 5);
    return tempFollowDir;
}

void Triangle::unsetFollowIndicator()
{
    for(int i = 0; i < 10; i++) {
        outFlags[i].followIndicator = false;
    }
}

void Triangle::setFollowIndicatorLabel(const int label)
{
    for(int i = 0; i < 10; i++) {
        outFlags[i].followIndicator = (i == label);
    }
}

bool Triangle::tailReceivesFollowIndicator() const
{
    for(auto it = tailLabels().cbegin(); it != tailLabels().cend(); ++it) {
        auto label = *it;
        if(inFlags[label] != nullptr) {
            if(inFlags[label]->followIndicator) {
                return true;
            }
        }
    }
    return false;
}

bool Triangle::followIndicatorMatchState(State _state) const
{
    for(auto it = tailLabels().cbegin(); it != tailLabels().cend(); ++it) {
        auto label = *it;
        if(inFlags[label] != nullptr) {
            if(inFlags[label]->followIndicator && inFlags[label]->state == _state) {
                return true;
            }
        }
    }
    return false;
}
}