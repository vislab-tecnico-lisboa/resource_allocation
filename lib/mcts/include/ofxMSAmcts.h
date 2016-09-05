/*
A very simple C++11 Templated MCTS (Monte Carlo Tree Search) implementation with examples for openFrameworks. 

MCTS Code Based on the Java (Simon Lucas - University of Essex) and Python (Peter Cowling, Ed Powley, Daniel Whitehouse - University of York) impelementations at http://mcts.ai/code/index.html
*/

#pragma once

#include "TreeNodeT.h"
#include "MSALoopTimer.h"
#include <cfloat>
#include <queue>
namespace msa {
namespace mcts {



struct timespec start, finish;
double elapsed;



// Belief must comply with Belief Interface (see IState.h)
// Action can be anything (which your Belief class knows how to handle)
template <class Belief, typename Action>
class UCT {
    typedef TreeNodeT<Belief, Action> TreeNode;

private:
    LoopTimer timer;
    int iterations;

public:
    float uct_k;					// k value in UCT function. default = sqrt(2)
    unsigned int max_iterations;	// do a maximum of this many iterations (0 to run till end)
    unsigned int max_millis;		// run for a maximum of this many milliseconds (0 to run till end)
    unsigned int simulation_depth;	// how many ticks (frames) to run simulation for

    //--------------------------------------------------------------
    UCT(unsigned int max_millis_=0, unsigned int simulation_depth_=10) :
        max_millis( max_millis_ ),
        simulation_depth( simulation_depth_ ),
        uct_k( sqrt(2) )
    {}


    //--------------------------------------------------------------
    const LoopTimer & get_timer() const {
        return timer;
    }

    void depth_first_expand(TreeNode* node){
        if(node->get_depth()>=simulation_depth)
            return;
        std::vector< Action > actions;
        Belief belief(node->get_belief());
        belief.get_actions(actions);

        for(int i=0; i<actions.size();++i)
        {
            // 2. EXPAND by adding all actions (if not terminal or not fully expanded)
            //if(!node->is_fully_expanded() && !node->is_terminal())
            if(!node->is_terminal())
                depth_first_expand(node->expand(actions[i]));
        }
    }

    void breath_first_expand(TreeNode* node){
        // Create a temporary queue to hold node pointers.
        std::queue<TreeNode*> queue;

        /*
             * Gotta put something in the queue initially,
             * so that we enter the body of the loop.
             */
        queue.push(node);
        std::cout << "depth:" << queue.front()->get_depth() << std::endl;

        std::vector< Action > actions;
        Belief belief(node->get_belief());
        belief.get_actions(actions);

        while (!queue.empty())
        {
            std::cout << "depth:" << queue.front()->get_depth() << std::endl;

            node=queue.front();
            queue.pop();

            if(node->get_depth()<simulation_depth&& !node->is_terminal())
            {
                for(int i=0; i<actions.size();++i)
                {
                    // 2. EXPAND by adding all actions (if not terminal or not fully expanded)
                    queue.push(node->expand(actions[i]));
                }
            }
        }

    }


    //--------------------------------------------------------------
    Action run(const std::vector<Belief>& current_belief) {
        // initialize timer
        clock_gettime(CLOCK_MONOTONIC, &start);
        timer.init();

        TreeNode* best_node = NULL;

        // For each tracker generate binary action tree
        for(int i=0; i < current_belief.size(); ++i)
        {
            // initialize root TreeNode with current belief
            TreeNode root_node(current_belief[i]);

            // 1. Start at root, dig down into tree
            //depth_first_expand(node);
            breath_first_expand(&root_node);
        }

        clock_gettime(CLOCK_MONOTONIC, &finish);
        elapsed = (finish.tv_sec - start.tv_sec);
        elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;

        std::cout << "elapsed time: "<< elapsed << std::endl;

        return Action();
    }


};
}
}
