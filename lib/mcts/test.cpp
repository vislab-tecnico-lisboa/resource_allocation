#include "PedestrianBelief.h"
#include "MultiplePedestrianBelief.h"
#include "ofxMSAmcts.h"
#include "MSALoopTimer.h"
int main(int argc, char** argv)
{
    unsigned int max_targets_=2;
    unsigned int total_targets_=5;
    float max_area_ratio_=0.1;

    std::vector<tracking::Belief> pedestrian_beliefs;

    pedestrian_beliefs.resize(total_targets_);

    tracking::MultipleBelief<tracking::Belief> belief(pedestrian_beliefs,max_targets_,max_area_ratio_);
    unsigned int max_millis=2000;
    unsigned int simulation_depth=5;


    //msa::mcts::UCT<tracking::Belief, tracking::Action> uct(max_millis, simulation_depth);

    msa::mcts::UCT<tracking::MultipleBelief<tracking::Belief>, tracking::MultipleAction> uct(max_millis, simulation_depth);

    uct.run(belief);

    return 0;
}
