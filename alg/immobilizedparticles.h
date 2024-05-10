//immo_shape working .h file
#ifndef Immobilizedparticles_H
#define Immobilizedparticles_H
#include <set>
#include <QString>
#include <bits/stl_list.h>
#include "core/amoebotparticle.h"
#include "core/amoebotsystem.h"

class Immobilizedparticles : public AmoebotParticle {
public:
    enum class State {
        Leader,
        Idle,
        Follower
        //Immobilized,
        //Delimiter
    };
    bool immobilized;
    // Constructs a new particle with a node position for its head, a global
    // compass direction from its head to its tail (-1 if contracted), an offset
    // for its local compass, a system which it belongs to and an initial state.
    Immobilizedparticles(const Node head, const int globalTailDir, const int orientation,
                         AmoebotSystem& system, State state);

    // Executes one particle activation.
    virtual void activate();
    // virtual void tryToPassLeaderToken();
    // virtual void processImmobilizedWakeUp();
    // virtual void processImmobilizedOrDelimiter();
    // virtual void processWakeUp();
    // virtual void processPhase1();
    // virtual void processPhase2();
    // virtual void processPhase2PreliminaryAction();
    // virtual void processPhase2Notification();
    // virtual void processPhase2Advance();
    // virtual void processPhase2AdvanceUpdated();
    // virtual void processPhase2Merge();
    // virtual void processPhase3();
    // virtual void getLineParentParticleLabelForContractedParticle();
    // virtual void getLineChildParticleLabel();
    // virtual int nextClockwiseDir(int inputDir);
    // virtual int nextCounterClockwiseDir(int inputDir);







    bool tryToBecomeFollower();
    bool isImmobilized() const;
    bool hasImmobilizedNbr() const;
    // Helper methods for the algorithm.
    virtual void tryToBecomeLeader();
    //virtual void immobilize();

    virtual void passLeaderToken(const int label);
    virtual void performMovement();
    virtual void updateBoolStates();

    virtual std::vector<int> randLabels();

    virtual bool hasBlockingTailNbr() const;

    // Functions for altering a particle's cosmetic appearance.
    // particleColor returns the color to be used for the particle.
    // headMarkColor (respectively, tailMarkColor) returns the color to be used for the ring
    // drawn around the head (respectively, tail) node. Tail color is not shown
    // when the particle is contracted. headMarkDir returns the label of the port
    // on which the black head marker is drawn.
    virtual int particleColor() const;
    virtual int headMarkColor() const;
    virtual int headMarkDir() const;
    virtual int tailMarkColor() const;


    // Returns the string to be displayed when this particle is inspected; used
    // to snapshot the current values of this particle's memory at runtime.
    virtual QString inspectionText() const;

    // Returns the _borderColors array associated with the
    // particle to draw the boundaries of the line.
    virtual std::array<int, 18> borderColors() const;
    // Returns the _borderPointColors array associated with the
    // particle to draw the directions of the token.
    virtual std::array<int, 6> borderPointColors() const;
    // Updates the _borderColors and the _borderPointColors arrays.
    void updateBorderColors();
    void updateBorderPointColors();

    // Gets a reference to the neighboring particle incident to the specified port
    // label. Crashes if no such particle exists at this label; consider using
    // hasNbrAtLabel() first if unsure.
    Immobilizedparticles& nbrAtLabel(int label) const;

    // Returns the label of the first port incident to a neighboring particle in
    // any of the specified states, starting at the (optionally) specified label
    // and continuing clockwise.
    int labelOfFirstNbrInState(std::initializer_list<State> states,
                               int startLabel = 0, bool ignoreImmobParticles = true) const;

    // Checks whether this particle has a neighbor in any of the given states.
    bool hasNbrInState(std::initializer_list<State> states) const;


    //bool doesEnclosureOccur(std::set<Node>& occupied, Node testNode);

    // Checks whether this particle's state is one of the given states.
    bool isInState(std::initializer_list<State> states) const;

protected:
    // General state of this particle.
    State state;

    // Line recovery specific variables of this particle:

    // Movement direction markers for the Leader and Followers.
    int moveDir;
    int followDir;

    // Token to be passed around when a new Leader is required.
    bool leaderToken;

    // Arbitrarily chosen 'goal' direction of the leader token. Like in the Pledge algorithm,
    // this is the token's ultimate destination direction.
    int tokenForwardDir;

    // Current direction that the token is pointing towards. If this direction marker does
    // not match the tokenForwardDir, the token's particle moves along the object closest to
    // the right of tokenCurrentDir.
    int tokenCurrentDir;

    // Ignore objects around the particle and pass the leader token forwards unless there
    // is no space.
    bool passForward;

    // Bool that marks whether a particle and all its children are 'free' from the objects.
    bool freeState;

    // Bool that marks whether a particle and all its children form a line.
    bool lineState;

    // _borderColorsSet and _borderColors are used to draw the boundaries of the hexagon layers.
    bool _borderColorsSet;

    std::array<int, 18> _borderColors;
    std::array<int, 6> _borderPointColors;

private:
    friend class ImmobilizedParticleSystem;
};

class ImmobilizedParticleSystem : public AmoebotSystem  {
public:
    // Constructs a system of ShapeFormationFaultTolerantParticles with an optionally specified
    // size (#particles) and immobility rate, i.e. the probability for each particle to be
    // immobilized in the initial configuration. The particles initially form a line.
    ImmobilizedParticleSystem(int numParticles = 75, int numImmoParticles = 75, int genExpExample = 0, int numCoinFlips = 7);

    // Checks whether or not the system's run of the ShapeFormation formation
    // algorithm has terminated (all particles finished).
    bool hasTerminated() const override;
};
#endif // Immobilizedparticles_H
