/*
A very simple C++11 Templated MCTS (Monte Carlo Tree Search) implementation with examples for openFrameworks. 

MCTS Code Based on the Java (Simon Lucas - University of Essex) and Python (Peter Cowling, Ed Powley, Daniel Whitehouse - University of York) impelementations at http://mcts.ai/code/index.html
*/

#pragma once

#include "TreeNodeT.h"
#include "MSALoopTimer.h"
#include <cfloat>

namespace msa {
namespace mcts {

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
    UCT(unsigned int iterations_=0, unsigned int max_iterations_=100, unsigned int max_millis_=0, unsigned int simulation_depth_=10) :
        iterations(iterations_),
        max_iterations( max_iterations_ ),
        max_millis( max_millis_ ),
        simulation_depth( simulation_depth_ ),
        uct_k( sqrt(2) )
    {}


    //--------------------------------------------------------------
    const LoopTimer & get_timer() const {
        return timer;
    }

    const int get_iterations() const {
        return iterations;
    }

    //--------------------------------------------------------------
    // get best (immediate) child for given TreeNode based on uct score
    TreeNode* get_best_uct_child(TreeNode* node, float uct_k) const {
        // sanity check
        if(!node->is_fully_expanded()) return NULL;

        float best_utc_score = -std::numeric_limits<float>::max();
        TreeNode* best_node = NULL;

        // iterate all immediate children and find best UTC score
        int num_children = node->get_num_children();
        for(int i = 0; i < num_children; i++) {
            TreeNode* child = node->get_child(i);
            float uct_exploitation = (float)child->get_value() / (child->get_num_visits() + FLT_EPSILON);
            float uct_exploration = sqrt( log((float)node->get_num_visits() + 1) / (child->get_num_visits() + FLT_EPSILON) );
            float uct_score = uct_exploitation + uct_k * uct_exploration;

            if(uct_score > best_utc_score) {
                best_utc_score = uct_score;
                best_node = child;
            }
        }

        return best_node;
    }


    //--------------------------------------------------------------
    TreeNode* get_most_visited_child(TreeNode* node) const {
        int most_visits = -1;
        TreeNode* best_node = NULL;

        // iterate all immediate children and find most visited
        int num_children = node->get_num_children();
        for(int i = 0; i < num_children; i++) {
            TreeNode* child = node->get_child(i);
            if(child->get_num_visits() > most_visits) {
                most_visits = child->get_num_visits();
                best_node = child;
            }
        }

        return best_node;
    }



    //--------------------------------------------------------------
    Action run(const Belief& current_belief, unsigned int seed = 1, std::vector<Belief>* explored_beliefs = nullptr) {
        // initialize timer
        timer.init();

        // initialize root TreeNode with current belief
        TreeNode root_node(current_belief);

        TreeNode* best_node = NULL;

        // iterate
        iterations = 0;
        while(true) {
            // indicate start of loop
            timer.loop_start();

            // 1. SELECT. Start at root, dig down into tree using UCT on all fully expanded nodes
            TreeNode* node = &root_node;
            while(!node->is_terminal() && node->is_fully_expanded()) {
                node = get_best_uct_child(node, uct_k);
                //						assert(node);	// sanity check
            }

            // 2. EXPAND by adding a single child (if not terminal or not fully expanded)
            if(!node->is_fully_expanded() && !node->is_terminal()) node = node->expand();

            Belief belief(node->get_belief());

            // 3. SIMULATE (if not terminal)
            if(!node->is_terminal()) {
                Action action;
                for(int t = 0; t < simulation_depth; t++) {
                    if(belief.is_terminal()) break;

                    if(belief.get_random_action(action))
                        belief.apply_action(action);
                    else
                        break;
                }
            }

            // get rewards vector for all agents
            const std::vector<float> rewards = belief.evaluate();

            // add to history
            if(explored_beliefs) explored_beliefs->push_back(belief);

            // 4. BACK PROPAGATION
            while(node) {
                node->update(rewards);
                node = node->get_parent();
            }

            // find most visited child
            best_node = get_most_visited_child(&root_node);

            // indicate end of loop for timer
            timer.loop_end();

            // exit loop if current total run duration (since init) exceeds max_millis
            if(max_millis > 0 && timer.check_duration(max_millis)) break;

            // exit loop if current iterations exceeds max_iterations
            if(max_iterations > 0 && iterations > max_iterations) break;
            iterations++;
        }

        // return best node's action
        if(best_node) return best_node->get_action();

        // we shouldn't be here
        return Action();
    }


};
}
}
