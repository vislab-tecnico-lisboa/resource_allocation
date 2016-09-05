#include "PedestrianBelief.h"
#include "ofxMSAmcts.h"
#include "MSALoopTimer.h"
int main(int argc, char** argv)
{
    std::vector<tracking::Belief> pedestrian_beliefs;
    pedestrian_beliefs.resize(10);
    unsigned int max_millis=0;
    unsigned int simulation_depth=5;

    msa::mcts::UCT<tracking::Belief, tracking::Action> uct(max_millis, simulation_depth);
    uct.run(pedestrian_beliefs);

    return 0;
}
