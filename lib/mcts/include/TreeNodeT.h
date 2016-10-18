/*
A TreeNode in the decision tree.
I tried to keep this independent of UCT/MCTS.
Only contains information / methods related to State, Action, Parent, Children etc. 

*/

#pragma once

#include <memory>
#include <math.h>
#include <vector>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/make_shared.hpp>

namespace msa {
namespace mcts {



template <class Belief, typename Action>
class TreeNodeT : public boost::enable_shared_from_this< TreeNodeT<Belief, Action> >
{
    typedef TreeNodeT* Ptr;

public:
    //--------------------------------------------------------------
    TreeNodeT(Belief belief_, TreeNodeT* parent_ = NULL, int node_id_=0):
        parent(parent_),
        num_visits(0),
        value(0),
        depth(parent ? parent->depth + 1 : 0),
        belief(belief_),
        node_id(node_id_)
    {
        //std::cout << " depth: "<<depth << std::endl;
    }


    ~TreeNodeT()
    {
        for(int i=0; i<children.size(); ++i)
        {
            delete children[i];
        }

    }

    struct actionValue {
        Action action;
        float value;
    };

    static bool compareByValue(const actionValue & lhs, const actionValue & rhs)
    {
        return lhs.value > rhs.value;
    }


    //--------------------------------------------------------------
    // expand by adding a single child
    TreeNodeT* expand(const int & node_id_=0)
    {
        // sanity check that we're not already fully expanded
        if(is_fully_expanded()) return NULL;

        // sanity check that we don't have more children than we do have actions
        // assert(children.size() < actions.size()) ;

        // if this is the first expansion and we haven't yet got all of the possible actions
        if(actions.empty())
        {
            // retrieve list of actions from the state
            belief.get_actions(actions);

            std::vector<actionValue> action_values;

            // Prioritize actions that are associated to higher entropy regions

            // For each action
            for(int i=0;i<actions.size();++i)
            {
                actionValue action_value;
                action_value.action=actions[i];
                action_value.value=0;

                // For each object (higher entropy is better)
                for(int a=0;a<actions[i].attend.size();++a)
                {
                    action_value.value+=belief.beliefs[actions[i].attend[a]].entropy();
                }
                action_values.push_back(action_value);
            }

            // More promising actions first (higher entropy)
            std::sort(action_values.begin(), action_values.end(),compareByValue);

            // randomize the order
            //std::random_shuffle(actions.begin(), actions.end());
            for (int i=0;i<actions.size();++i)
            {
                actions[i]=action_values[i].action;
                //std::cout << actions[i]<< " ";
            }
            //std::cout << std::endl<<std::endl;

            /*for (int i=0;i<action_values.size();++i)
            {
                std::cout << action_values[i].value<< " ";
            }
            std::cout << std::endl<<std::endl;*/


            //std::cout << actions << std::endl;
            // Randomize the order based on the value
            //std::default_random_engine generator;
            //std::discrete_distribution<int> distribution( weights.begin(), weights.end()) ;
        }
        // add the next action in queue as a child
        return add_child_with_action( actions[children.size()],node_id_ );
    }

    //--------------------------------------------------------------
    // expand by adding a single child
    TreeNodeT* expand(const Action& new_action)
    {
        // add the next action in queue as a child
        return add_child_with_action(new_action);
    }

    //--------------------------------------------------------------
    void update(float & reward) {
        this->value += reward;
        num_visits++;
    }

    //--------------------------------------------------------------
    // GETTERS
    // belief of the TreeNode
    const Belief& get_belief() const { return belief; }

    // the action that led to this belief
    const Action& get_action() const { return action; }

    // all children have been expanded and simulated
    bool is_fully_expanded() const { return children.empty() == false && children.size() == actions.size(); }

    // does this TreeNode end the search (i.e. the game)
    bool is_terminal() {
         return belief.is_terminal(this->action);
    }

    // number of times the TreeNode has been visited
    int get_num_visits() const { return num_visits; }

    // number of times the TreeNode has been visited
    void set_num_visits(const int & num_visits_)  { num_visits=num_visits_; return; }

    // accumulated value (wins)
    float get_value() const { return value; }

    // how deep the TreeNode is in the tree
    int get_depth() const { return depth; }

    // get node id
    int get_id() const { return node_id; }


    // number of children the TreeNode has
    int get_num_children() const { return children.size(); }

    // get the i'th child
    TreeNodeT * get_child(int i) const { return children[i]; }

    // get parent
    TreeNodeT* get_parent() const { return parent; }

private:
    Belief belief;			// the belief of this TreeNode
    Action action;			// the action which led to the state of this TreeNode
    TreeNodeT* parent;		// parent of this TreeNode
    //int agent_id;			// agent who made the decision

    int num_visits;			// number of times TreeNode has been visited
    float value;			// value of this TreeNode
    int depth;
    int node_id;

    std::vector< Ptr > children;	// all current children
    std::vector< Action > actions;  // possible actions from this state



    //--------------------------------------------------------------
    // create a clone of the current state, apply action, and add as child
    TreeNodeT* add_child_with_action(const Action& new_action, int node_id_=0) {
        // create a new TreeNode with the same state (will get cloned) as this TreeNode
        TreeNodeT* child_node =new TreeNodeT(belief, this,node_id_);

        // set the action of the child to be the new action
        child_node->action = new_action;

        // apply the new action to the state of the child TreeNode
        child_node->belief.apply_action(new_action);

        // add to children
        children.push_back(Ptr(child_node));

        return child_node;
    }

};

}
}
