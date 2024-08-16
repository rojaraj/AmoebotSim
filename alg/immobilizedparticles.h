// //immo_shape working .h file
// //LeaderElectionParticle is changed to ImmoLeaderElectionParticle
// //LeaderElectionAgent is changed to ImmoLeaderElectionAgent
#ifndef Immobilizedparticles_H
#define Immobilizedparticles_H
#include <set>
#include <QString>
#include <array>
#include <vector>
#include "core/amoebotparticle.h"
#include "core/amoebotsystem.h"
#include <QOpenGLFunctions_2_0>
class Immobilizedparticles : public AmoebotParticle {
public:
    enum class State {
        Immo,       // for immo particle
        Leader,
        Idle,       // initial state
        Follower,
        FollowerHex,        // Member of the spanning forest but not on the forming hexagon.
        Marker,
        MergedMarker,
        Seed,
        Single,
        Retired,
        Finish,
        Lead,
        Root,        // non-immobilized particle which is not a part of the spanning forest
        Candidate, // for leader election
        SoleCandidate,
        Demoted,
        Finished
    };
    enum class Phase {
        LeaderElection,
        InitializeTrees,
        MoveToTargetTree,
        CheckTargetTree,
        LeaderMovement,
        HexagonFormation
    };
    Phase phase;
    // Constructs a new particle with a node position for its head, a global
    // compass direction from its head to its tail (-1 if contracted), an offset
    // for its local compass, a system which it belongs to and an initial state.
    Immobilizedparticles(const Node head, const int globalTailDir, const int orientation,
                         AmoebotSystem& system, State state);
    // Executes one particle activation.
    virtual void activate();
    virtual void initializeTrees();
    virtual void startLeaderElection();
    virtual void handleMoveToTargetTree();
    virtual void processParticlesWithLeaderToken();
    //virtual void changetoImmo();
    bool isLeaf() const;
    bool isImmobilized() const;

    void updateTargetTree();

    int getLineChildParticleLabel() const;


    bool hasChildParticle() const;

    void mergeWithMarker(Node& nbr);
    bool hasIdleParticles;
    bool shouldMoveToInitializeTrees;
    void broadCast();
    bool terminated;
    std::vector<int> childLabels() ;
    bool hasNbrWithFollowDir2Unset();
    //virtual void processactivateHex();
    bool tree2Ack;
    bool hasCompletedActivateLeader() const;
    bool isImmo;
    bool isIdle;
    virtual void processactivateHex();
    virtual void leaderElection();
    //NON-IMMO state change
    //void changeStateOfNonImmobilizedParticles();
    //Get neighbour labels
    //virtual std::vector<int> getNbrLabels() const;
    virtual void activateLeader();
    virtual void tryToBecomeLeader();
    virtual void activateHex();
    virtual int nextClockwiseDir(int inputDir);
    virtual int nextCounterclockwiseDir(int inputDir);
    int constructionReceiveDir() const;
    // Checks whether this particle is occupying the next position to be filled.
    bool canFinish() const;
    // Sets this particle's constructionDir to point at the next position to be
    // filled as it is finishing.
    void updateConstructionDir();
    // Updates this particle's moveDir when it is a leader to traverse the current
    // surface of the forming shape counter-clockwise.
    void updateMoveDir();
    // Checks whether this particle has an immediate child in the spanning tree
    // following its tail.
    bool hasTailFollower() const;
    virtual void passLeaderToken(const int label);
    virtual void performMovement();
    //virtual void connectCluster();
    bool areAllClusterAndIdleParticlesFollowers();
    virtual void performMovement2();
    virtual void updateBoolStates();
    //virtual void updateMoveDir();
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
    virtual std::array<int, 18> leaderborderColorLabels() const;
    virtual std::array<int, 6> leaderborderPointColorLabels() const;
    void updateBorderColors();
    void updateBorderPointColors();
    //void updateAllBorderPointColors();
    // Gets a reference to the neighboring particle incident to the specified port
    // label. Crashes if no such particle exists at this label; consider using
    // hasNbrAtLabel() first if unsure.
    Immobilizedparticles& nbrAtLabel(int label) const;
    //virtual int labelOfFirstNbrInState(std::initializer_list<State> states, int startLabel, bool ignoreErrorParticles) const;
    // Returns the label of the first port incident to a neighboring particle in
    // any of the specified states, starting at the (optionally) specified label
    // and continuing clockwise.
    // int labelOfFirstNbrInState(std::initializer_list<State> states, int startLabel = 0, bool ignoreImmobParticles = true) const;
    virtual int labelOfFirstNbrInState(std::initializer_list<State> states, int startLabel = 0, bool ignoreImmobParticles = true) const;
    // Checks whether this particle has a neighbor in any of the given states.
    bool hasNbrInState(std::initializer_list<State> states) const;
    int nextHexagonDir(int orientation) const;
    bool canRetire() const;
    bool hasTailChild() const;
    const std::vector<int>  conTailChildLabels() const;
    //     // int constructionReceiveDir() const;
    //     //bool doesEnclosureOccur(std::set<Node>& occupied, Node testNode);
    // Checks whether this particle's state is one of the given states.
    bool isInState(std::initializer_list<State> states) const;
    void setState(State newState);
    //for parent-childtree
    // void setParent(Immobilizedparticles* parent) {
    //     this->parent = parent;
    // }
    // void addChild(Immobilizedparticles* child) {
    //     this->children.push_back(child);
    // }
    //void updateConstructionDir();
    // bool canFinish() const;
    // bool hasTailFollower() const;
    // Returns the label associated with the direction which the next (resp.
    // previous) agent is according to the cycle that the agent is on (which is
    // determined by the provided agentDir parameter).
    int getNextAgentDir(const int agentDir) const;
    int getPrevAgentDir(const int agentDir) const;
    // Returns a count of the number of particle neighbors surrounding the calling
    // particle.
    int getNumberOfNbrs() const;
protected:
    // General state of this particle.
    State state;
    int _parentDir;   // Corresponds to "parent" in paper.
    int _hexagonDir;
    QOpenGLFunctions_2_0* glfn;
    // Line recovery specific variables of this particle:
    // Movement direction markers for the Leader and Followers.
    int moveDir;
    int label;
    int followDir;
    int followDir1;
    int followDir2;
    int constructionDir;
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
    // std::array<int, 18> _borderColors;
    // std::array<int, 6> _borderPointColors;
    // The LeaderElectionToken struct provides a general framework of any token
    // under the General Leader Election algorithm.
    struct LeaderElectionToken : public Token {
        // origin is used to define the direction (label) that a LeaderElectionToken
        // was received from.
        int origin;
    };
    // Tokens for Candidate Elimination via Segment Comparison
    struct SegmentLeadToken : public LeaderElectionToken {
        SegmentLeadToken(int origin = -1) {
            this->origin = origin;
        }
    };
    struct PassiveSegmentToken : public LeaderElectionToken {
        bool isFinal;
        PassiveSegmentToken(int origin = -1, bool isFinal = false) {
            this->origin = origin;
            this->isFinal = isFinal;
        }
    };
    struct ActiveSegmentToken : public LeaderElectionToken {
        bool isFinal;
        ActiveSegmentToken(int origin = -1, bool isFinal = false) {
            this->origin = origin;
            this->isFinal = isFinal;
        }
    };
    struct PassiveSegmentCleanToken : public LeaderElectionToken {
        PassiveSegmentCleanToken(int origin = -1) {
            this->origin = origin;
        }
    };
    struct ActiveSegmentCleanToken : public LeaderElectionToken {
        ActiveSegmentCleanToken(int origin = -1) {
            this->origin = origin;
        }
    };
    struct FinalSegmentCleanToken : public LeaderElectionToken {
        bool hasCoveredCandidate;
        FinalSegmentCleanToken(int origin = -1, bool hasCovered = false) {
            this->origin = origin;
            this->hasCoveredCandidate = hasCovered;
        }
    };
    // Tokens for Coin Flipping and Candidate Transferal
    struct CandidacyAnnounceToken : public LeaderElectionToken {
        CandidacyAnnounceToken(int origin = -1) {
            this->origin = origin;
        }
    };
    struct CandidacyAckToken : public LeaderElectionToken {
        CandidacyAckToken(int origin = -1) {
            this->origin = origin;
        }
    };
    // Tokens for Solitude Verification
    struct SolitudeActiveToken : public LeaderElectionToken {
        bool isSoleCandidate;
        std::pair<int, int> vector;
        int local_id;
        SolitudeActiveToken(int origin = -1,
                            std::pair<int, int> vector = std::make_pair(1, 0),
                            int local_id = -1,
                            bool isSole = true) {
            this->origin = origin;
            this->vector = vector;
            this->local_id = local_id;
            this->isSoleCandidate = isSole;
        }
    };
    struct SolitudeVectorToken : public LeaderElectionToken {
        bool isSettled;
    };
    struct SolitudePositiveXToken : public SolitudeVectorToken {
        SolitudePositiveXToken(int origin = -1, bool settled = false) {
            this->origin = origin;
            this->isSettled = settled;
        }
    };
    struct SolitudePositiveYToken : public SolitudeVectorToken {
        SolitudePositiveYToken(int origin = -1, bool settled = false) {
            this->origin = origin;
            this->isSettled = settled;
        }
    };
    struct SolitudeNegativeXToken : public SolitudeVectorToken {
        SolitudeNegativeXToken(int origin = -1, bool settled = false) {
            this->origin = origin;
            this->isSettled = settled;
        }
    };
    struct SolitudeNegativeYToken : public SolitudeVectorToken {
        SolitudeNegativeYToken(int origin = -1, bool settled = false) {
            this->origin = origin;
            this->isSettled = settled;
        }
    };
    // Token for Border Testing
    struct BorderTestToken : public LeaderElectionToken {
        int borderSum;
        BorderTestToken(int origin = -1, int borderSum = -1) {
            this->origin = origin;
            this->borderSum = borderSum;
        }
    };
private:





    friend class ImmobilizedParticleSystem;
    // The nested class LeaderElectionAgent is used to define the behavior for the
    // agents as described in the paper.
    // Note that this class needs to be public since the agents used in the Daymude node-based leader election algorithm extend it.
    class ImmoLeaderElectionAgent {
    public:
        enum class SubPhase {
            SegmentComparison = 0,
            CoinFlipping,
            SolitudeVerification
        };
        ImmoLeaderElectionAgent();
        // General variables in agent memory:
        // The particle emulating this agent assigns it a localId in [1,3] to
        // distinguish it from the other agents it may be emulating. From the
        // particle's perspective, this agent is in local direction/label agentDir.
        // The neighboring particle emulating the next (respectively, previous)
        // agent on this agent's boundary is in local direction nextAgentDir
        // (respectively, prevAgentDir). passTokensDir is used to determine if the
        // agent should pass tokens toward nextAgentDir (if 0) or prevAgentDir (if
        // 1). This is done to maintain the rule from direct write communication
        // that a particle can only write into the memory of one of its neighbors in
        // a single activation.
        int localId;
        int agentDir, nextAgentDir, prevAgentDir;
        int passTokensDir = -1;
        State agentState;
        SubPhase subPhase;
        Immobilizedparticles* candidateParticle;
        // Variables for Segment Comparison:
        // comparingSegment is true if this agent is in the Segment Comparison
        // subphase and has generated and passed a segment lead token along its
        // front segment; it is false otherwise.
        // isCoveredCandidate is set to true when this candidate agent is covered by
        // another candidate and demotes itself; i.e., when it receives an active
        // token from its front segment. This indicates to a later final segment
        // clean token that the candidate that sent it covered another candidate in
        // its Segment Comparison subphase.
        // absorbedActiveToken is true if this agent has already absorbed an active
        // token, which tells it whether to absorb active tokens or pass them
        // backwards along the cycle in the Segment Comparison subphase.
        // generatedCleanToken is true if this agent was covered and generated a
        // cleaning token. This is needed because a particle can only write into the
        // memory of one neighbor per activation, but it needs to pass cleaning
        // tokens in both directions.
        bool comparingSegment = false;
        bool isCoveredCandidate = false;
        bool absorbedActiveToken = false;
        bool generatedCleanToken = false;
        // Variables for Coin Flipping and Candidacy Transferal:
        // gotAnnounceInCompare is set to true if this agent did not succeed in
        // covering another candidate in its Segment Comparison subphase but was
        // “saved” by a candidacy transferal; instead of demoting itself, it will
        // proceed with the next subphase.
        // gotAnnounceBeforeAck is set to true if this agent is in the Coin Flipping
        // subphase and receives a candidate announcement token before it receives a
        // candidate acknowledgement token; thus, instead of demoting itself, it
        // will proceed with Solitude Verification.
        // waitingForTransferAck is true if this agent generated a candidate
        // announcement token in the Coin Flipping subphase but has not yet received
        // an acknowledgement.
        bool gotAnnounceInCompare = false;
        bool gotAnnounceBeforeAck = false;
        bool waitingForTransferAck = false;
        // Variables for Solitude Verification:
        // createdLead is true if this agent generated a solitude active token and
        // passed it forward during Solitude Verification.
        // hasGeneratedTokens is true if this agent generated solitude vector tokens
        // using the solitude active token. This is used to avoid incorrectly mixing
        // tokens of different agents on the same particle in Solitude Verification.
        bool createdLead = false;
        bool hasGeneratedTokens = false;
        // Variables for Boundary Testing:
        // testingBorder is true if this agent is the sole candidate and has begun
        // the Boundary Testing subphase.
        bool testingBorder = false;
        // The activate function is the LeaderElectionAgent equivalent of an
        // Amoebot Particle's activate function
        void activate();
        // Methods for token cleaning if a candidate is covered.
        // The boolean parameter "first" is used to determine whether or not the
        // cleaning agent is a covered candidate which has just absorbed an active
        // token and must delete/clean its tokens for the first time.
        void activeClean(bool first);
        void passiveClean(bool first);
        // Solitude Verification Methods
        // augmentDirVector takes a <int, int> pair as a parameter, which represents
        // the current vector stored in the solitude active token. This function
        // then generates the next vector according to a local coordinate system
        // (which is determined when a candidate agent in the Solitude Verification
        // subphase generates the solitude active token) based on the vector stored
        // in the solitude active token.
        std::pair<int, int> augmentDirVector(std::pair<int, int> vector);
        // generateSolitudeVectorTokens generates the solitude vector tokens
        // (SolitudePositiveXToken, SolitudeNegativeXToken, etc.) based on the given
        // parameter vector.
        void generateSolitudeVectorTokens(std::pair<int, int> vector);
        // The checkSolitudeXTokens and checkSolitudeYTokens are used to determine
        // the condition of the solitude vector tokens that an agent might own.
        // The functions will return a value contained in [0,2] depending on what
        // condition the solitude vector tokens are in:
        // 0 --> tokens are settled and there is a mismatch, i.e., the agent might
        // have a positive x token (which as settled), but no corresponding negative
        // x token.
        // 1 --> at least one of the tokens is not settled
        // 2 --> tokens are settled and there is a match or neither tokens are
        // present on the current agent.
        int checkSolitudeXTokens() const;
        int checkSolitudeYTokens() const;
        // The cleanSolitudeVerificationTokens function will clean the solitude
        // vector tokens owned by a particular agent as well as paint the
        // front and back segments gray
        void cleanSolitudeVerificationTokens();
        // Boundary Testing methods
        int addNextBorder(int currentSum) const;
        // Methods for passing, taking, and checking the ownership of tokens at the
        // agent level
        template <class TokenType>
        bool hasAgentToken(int agentDir) const;
        template <class TokenType>
        std::shared_ptr<TokenType> peekAgentToken(int agentDir) const;
        template <class TokenType>
        std::shared_ptr<TokenType> takeAgentToken(int agentDir);
        template <class TokenType>
        void passAgentToken(int agentDir, std::shared_ptr<TokenType> token);
        ImmoLeaderElectionAgent* nextAgent() const;
        ImmoLeaderElectionAgent* prevAgent() const;
        // Methods responsible for rendering the agents onto the simulator with
        // their colors changing based on the state and the subphase of the current
        // agent
        // Red --> Candidate agent in Segment Comparison Subphase
        // Yellow --> Candidate agent in Coin Flipping Subphase
        // Blue --> Candidate agent in Solitude Verification Subphase
        // Grey --> Demoted agent
        // Green --> Sole candidate
        void setStateColor();
        void setSubPhaseColor();
        // Methods responsible for painting the borders which will act as physical
        // representations of the cycle for leader election
        // Red --> Segment Comparison Phase
        // Yellow --> Coin Flipping Phase
        // Blue --> Solitude Verification Phase
        // Grey --> No phase (or, alternatively, active phase of Segment Comparison
        // phase
        //
        // Note: This methods need to be made virtual to enable dynamic method binding
        // (required when extending this class to adapt/enahnce leader election algorithm).
        virtual void paintFrontSegment(const int color);
        virtual void paintBackSegment(const int color);
    };
    // friend class LeaderElectionSystem;
    // Immobilizedparticles* parent;
    // std::vector<Immobilizedparticles*> children;
protected:
    unsigned int currentAgent;
    std::vector<ImmoLeaderElectionAgent*> agents;
    std::array<int, 18> _leaderborderColorLabels;
    std::array<int, 6> _leaderborderPointColorLabels;
};
class ImmobilizedParticleSystem : public AmoebotSystem  {
public:
    // Constructs a system of ShapeFormationFaultTolerantParticles with an optionally specified
    // size (#particles) and immobility rate, i.e. the probability for each particle to be
    // immobilized in the initial configuration. The particles initially form a line.
    ImmobilizedParticleSystem(int numParticles = 200, int numImmoParticles = 200, int genExpExample = 0, int numCoinFlips = 20);
    void printAllParticleStates() const;
    void setParticlesToImmobilized();
    const std::vector<AmoebotParticle*>& getParticles() const {
        return particles;
    }



    void statechange();
    void updateParticleStates();
    bool updateParticleStates2();
    bool hasCompletedActivateLeader() const;
    bool hasCompletedMoveToTargetTree() const;
    void updateBorderPointColors();
    void updateAllBorderPointColors();
    bool checkAndSwitchToHexagonFormationPhase();
    // Checks whether or not the system's run of the ShapeFormation formation
    // algorithm has terminated (all particles finished).
    bool hasLeader () const;
    bool areAllParticlesInTargetStates() const;
    bool hasTerminated() const override;
};
#endif // Immobilizedparticles_H
