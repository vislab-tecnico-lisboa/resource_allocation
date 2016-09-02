#include "TreeNodeT.h"
#include "PedestrianBelief.h"
#include "ofxMSAmcts.h"
#include "MSALoopTimer.h"
int main(int argc, char** argv)
{
    tracking::Belief pedestrian_belief;
    msa::LoopTimer timer;
    unsigned int iterations=0;
    unsigned int max_iterations=100;
    unsigned int max_millis=0;
    unsigned int simulation_depth=10;

    msa::mcts::UCT<tracking::Belief, tracking::Action> uct(iterations,max_iterations, max_millis, simulation_depth);


    return 0;
}
