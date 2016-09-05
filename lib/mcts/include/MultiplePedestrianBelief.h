#pragma once

#include "PedestrianBelief.h"
#include "ofxMSAmcts.h"
#include <opencv2/video/tracking.hpp>
#include <iostream>

#define kNumState 6
#define kNumObs 3
#define kNumActions		2
#define kTurnRangeMin	-30
#define kTurnRangeMax	30
using namespace msa::mcts;
using namespace std;
namespace tracking {

//--------------------------------------------------------------
//--------------------------------------------------------------
class MultipleAction {
public:
    MultipleAction() {};
    MultipleAction(const int & max_attend)
    {
        attend.resize(max_attend);
    }

    std::vector<int> attend;

    friend inline std::ostream& operator<<(std::ostream& os, const MultipleAction &b)
    {
        std::cout << "[";
        for(int i=0; i< b.attend.size()-1; ++i)
        {
            os << b.attend[i] << " ";
        }
        os << b.attend[b.attend.size()-1];

        std::cout << "]";

        return os;
    }
};

std::vector<int> combination;
//--------------------------------------------------------------
//--------------------------------------------------------------
template <class Belief>
class MultipleBelief {
public:

    friend std::ostream& operator<< (std::ostream &out,
                                     const MultipleBelief<Belief> &tree) {}

    //--------------------------------------------------------------
    // MUST HAVE METHODS (INTERFACE)
    MultipleBelief( const MultipleBelief& other_) : beliefs(other_.beliefs), actions(other_.actions)
    {
    }

    MultipleBelief(const std::vector<Belief> & beliefs_, const unsigned int & max_attend_=1, const float & max_area_ratio_=1.0) :
        beliefs(beliefs_),
        max_attend(max_attend_),
        max_area_ratio(max_area_ratio_)
    {
        std::cout << "max_atend:" << max_attend << std::endl;
        std::cout << "max_area_ratio:" << max_area_ratio_ << std::endl;
        std::cout << "trackers:"<< beliefs_.size() << std::endl;
        // Compute all possible actions vector
        unsigned int possible_actions=choose(beliefs.size(),max_attend_);

        actions.reserve(possible_actions);
        for(int i=0; i<possible_actions;++i)
        {
            actions.push_back(max_attend);
        }
        // Create combinations
        create_actions(0, max_attend_);

        /*for(int i=0; i< actions.size(); ++i)
        {
            std::cout << "combination no " << i+1 << ": " << actions[i] << std::endl;
        }*/

        std::cout << "actions size:"<< actions.size() << std::endl;

    }

    unsigned int choose(unsigned int n, unsigned int k)
    {
        if (k > n) {
            return 0;
        }
        unsigned int r = 1;
        for (unsigned int d = 1; d <= k; ++d) {
            r *= n--;
            r /= d;
        }
        return r;
    }

    // whether or not this belief is terminal (reached end)
    bool is_terminal() {

    }

    // apply action to belief
    void apply_action(const MultipleAction& action)  {
        Action simple_action_;
        simple_action_.attend=true;
        for(int i=0; i<action.attend.size();++i)
        {

            beliefs[action.attend[i]].apply_action(simple_action_);
        }
    }


    // return possible actions from this state
    void get_actions(std::vector<MultipleAction>& actions_) const  {
        actions_=actions;
        std::cout << "AAACTIONS SIZE:"<< actions.size() << std::endl;
    }


    // get a random action, return false if no actions found
    bool get_random_action(Action& action) const {
        return true;
    }


    // evaluate this state and return a vector of rewards (for each agent)
    const float evaluate() const  {
        float reward;
        for(int i=0; i<beliefs.size();++i)
        {
            // Negative entropy
            reward+=-beliefs[i].evaluate();
        }

        return reward;

    }




    // OPTIONAL

    unsigned int max_attend;
    unsigned int max_area_ratio;
    std::vector<MultipleAction> actions;
    std::vector<Belief> beliefs;



    void create_actions(int offset, int k)
    {
        if (k == 0)
        {
            static int count = 0;
            for (int i = 0; i < combination.size(); ++i) {
                actions[count].attend[i]=combination[i];

            }
            ++count;
            return;
        }


        for (int i = offset; i <= beliefs.size() - k; ++i) {
            combination.push_back(i);
            create_actions(i+1, k-1);
            combination.pop_back();
        }
    }



    // return state as string (for debug purposes)
    std::string to_string() const  {

    }




    void reset() {

    }

    void draw() {

    }
};

}
