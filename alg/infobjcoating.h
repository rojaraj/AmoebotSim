#ifndef INFOBJCOATING_H
#define INFOBJCOATING_H

#include "alg/algorithmwithflags.h"

class System;

namespace InfObjCoating
{
enum class Phase {
    Static,
    Inactive,
    Follow,
    Lead
};

class InfObjCoatingFlag : public Flag
{
public:
    InfObjCoatingFlag();
    InfObjCoatingFlag(const InfObjCoatingFlag& other);

public:
    Phase phase;
    int contractDir;
    bool followIndicator;
};

class InfObjCoating : public AlgorithmWithFlags<InfObjCoatingFlag>
{
public:
    InfObjCoating(const Phase _phase);
    InfObjCoating(const InfObjCoating& other);
    virtual ~InfObjCoating();

    static System* instance(const int numParticles, const float holeProb);

    virtual Movement execute();
    virtual Algorithm* clone();
    virtual bool isDeterministic() const;

protected:
    void setPhase(const Phase _phase);

    bool neighborIsInPhase(const int label, const Phase _phase) const;
    int firstNeighborInPhase(const Phase _phase) const;
    bool hasNeighborInPhase(const Phase _phase) const;

    void setContractDir(const int contractDir);
    int updatedFollowDir() const;

    void unsetFollowIndicator();
    void setFollowIndicatorLabel(const int label);
    bool tailReceivesFollowIndicator() const;

    int getMoveDir() const;

protected:
    Phase phase;
    int followDir;
};
}

#endif // INFOBJCOATING_H