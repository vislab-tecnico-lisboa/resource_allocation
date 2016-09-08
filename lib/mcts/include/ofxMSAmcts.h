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
        std::cout << "actions:"<< actions.size() << std::endl;
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




    /*void depth_first_expand(TreeNode* node){
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
    }*/
    void greedy_depth_first(TreeNode* node){
        if(node->get_depth()>=simulation_depth)
            return;
        //std::cout << " action: " << node->get_action() << std::endl;

        // 1. EXPAND by adding a single child (if not terminal or not fully expanded)
        if(!node->is_fully_expanded() && !node->is_terminal()) greedy_depth_first(node->expand());
        // get reward
        float reward = node->get_belief().evaluate();


        // 2. BACK PROPAGATE REWARD
        while(node) {
            node->update(reward);
            node = node->get_parent();
        }

        return;
    }




    /*Action run(const Belief & current_belief) {
        // initialize timer
        clock_gettime(CLOCK_MONOTONIC, &start);
        timer.init();

        TreeNode* best_node = NULL;

        // For each tracker generate binary action tree

        // initialize root TreeNode with current belief
        TreeNode root_node(current_belief);
        // 1. Start at root, dig down into tree
        //depth_first_expand(&root_node);

        while(true) {
            // indicate start of loop
            timer.loop_start();

            // 1. SELECT. Start at root, dig down into tree on all fully expanded nodes
            TreeNode* node = &root_node;
            while(!node->is_terminal() && node->is_fully_expanded()) {
                node = get_best_child(node);
            }
            std::cout << "estou aqui" << std::endl;
            greedy_depth_first(node);

            // indicate end of loop for timer
            timer.loop_end();

            // exit loop if current total run duration (since init) exceeds max_millis
            if(max_millis > 0 && timer.check_duration(max_millis) || node->get_depth()>=simulation_depth) break;
        }



        clock_gettime(CLOCK_MONOTONIC, &finish);
        elapsed = (finish.tv_sec - start.tv_sec);
        elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;

        std::cout << "elapsed time: "<< elapsed << std::endl;
        return get_best_child(&root_node)->get_action();

        //return Action();
    }*/


    //--------------------------------------------------------------
    // get best (immediate) child for given TreeNode based on score (PROBLEMA. ESTA SOLUÇÂO É DETERMINISTICA, TEMOS QUE TER FORMA DE NAO EXPLORAR CAMINHOS QUE JA FORAM VISITADOS
    /*TreeNode* get_best_child(TreeNode* node) const {
        // sanity check
        //if(!node->is_fully_expanded()) return NULL;

        float best_score = -std::numeric_limits<float>::max();
        TreeNode* best_node = NULL;

        // iterate all immediate children and find best score
        int num_children = node->get_num_children();
        for(int i = 0; i < num_children; i++) {
            TreeNode* child = node->get_child(i);
            float score = (float)child->get_value();

            if(score > best_score) {
                best_score = score;
                best_node = child;
            }
        }

        return best_node;
    }

    //--------------------------------------------------------------
    Action run(const Belief& current_belief, unsigned int seed = 1) {
        // initialize timer
        timer.init();

        // initialize root TreeNode with current state
        TreeNode root_node(current_belief);

        TreeNode* best_node = NULL;

        // iterate
        iterations = 0;
        while(true) {
            // indicate start of loop
            timer.loop_start();

            // 1. SELECT. Start at root, dig down into tree on all fully expanded nodes
            TreeNode* node = &root_node;
            while(!node->is_terminal() && node->is_fully_expanded()) {
                node = get_best_child(node);
            }

            // 2. EXPAND by adding a single child (if not terminal or not fully expanded)
            if(!node->is_fully_expanded() && !node->is_terminal() && node->get_depth()<=simulation_depth) node = node->expand();
            // get reward
            float reward = node->get_belief().evaluate();

            std::cout << " action: " << node->get_action() << std::endl;

            // 3. BACK PROPAGATION
            while(node) {
                node->update(reward);
                node = node->get_parent();
            }


            // indicate end of loop for timer
            timer.loop_end();

            // exit loop if current total run duration (since init) exceeds max_millis
            if(max_millis > 0 && timer.check_duration(max_millis)) break;
        }


        // return best node's action
        return get_best_child(&root_node)->get_action();

        // we shouldn't be here
        return Action();
    }*/



    //--------------------------------------------------------------
    /*Action run(const Belief & current_belief) {
        // initialize timer
        clock_gettime(CLOCK_MONOTONIC, &start);
        timer.init();

        TreeNode* best_node = NULL;

        // For each tracker generate binary action tree

        // initialize root TreeNode with current belief
        TreeNode root_node(current_belief);
        // 1. Start at root, dig down into tree
        //depth_first_expand(node);
        breath_first_expand(&root_node);


        clock_gettime(CLOCK_MONOTONIC, &finish);
        elapsed = (finish.tv_sec - start.tv_sec);
        elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;

        std::cout << "elapsed time: "<< elapsed << std::endl;

        //return Action();
    }*/







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
    Action run(const Belief& current_belief, unsigned int seed = 1) {
        // initialize timer
        timer.init();

        // initialize root TreeNode with current state
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

            std::cout << "Expand"<< std::endl;
            // 2. EXPAND by adding a single child (if not terminal or not fully expanded)
            if(!node->is_fully_expanded() && !node->is_terminal()) node = node->expand();



            // 3. SIMULATE (if not terminal)

            // Copy the belief
            Belief belief(node->get_belief());
            std::cout << "Simulate"<< std::endl;

            if(!node->is_terminal()) {
                Action action;
                for(int t = 0; t < simulation_depth; t++) {
                    if(belief.is_terminal()) break;

                    if(belief.get_random_action(action))
                    {
                        std::cout << "  simulation action:"<< action <<std::endl;
                        belief.apply_action(action);
                    }
                    else
                    {
                        break;
                    }
                }
            }

            // get rewards vector for all agents
            float reward = belief.evaluate();

            // add to history

            // 4. BACK PROPAGATION
            while(node) {
                node->update(reward);
                node = node->get_parent();
            }

            // find most visited child
            best_node = get_most_visited_child(&root_node);

            // indicate end of loop for timer
            timer.loop_end();

            // exit loop if current total run duration (since init) exceeds max_millis
            if(max_millis > 0 && timer.check_duration(max_millis)) break;

            // exit loop if current iterations exceeds max_iterations
            //if(max_iterations > 0 && iterations > max_iterations) break;
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
