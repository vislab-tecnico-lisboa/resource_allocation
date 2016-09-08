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
    MultipleBelief( const MultipleBelief& other_) :
        beliefs(other_.beliefs),
        actions(other_.actions),
        max_attend(other_.max_attend),
        total_area(other_.total_area),
        max_area_ratio(other_.max_area_ratio),
        max_area(other_.max_area)
    {
    }

    MultipleBelief(const std::vector<Belief> & beliefs_, const unsigned int & max_attend_=1, const unsigned int & total_area_=10000000000, const float & max_area_ratio_=1.0) :
        beliefs(beliefs_),
        max_attend(max_attend_),
        total_area(total_area_),
        max_area_ratio(max_area_ratio_),
        max_area((float)total_area*max_area_ratio)
    {
        std::cout << "max_atend:" << max_attend << std::endl;
        std::cout << "total_area:" << total_area << std::endl;
        std::cout << "max_area_ratio:" << max_area_ratio_ << std::endl;
        std::cout << "max_area:" << max_area << std::endl;

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

        float area=0;

        for(int i=0; i<beliefs.size();++i)
        {
            area+=beliefs[i].compute_observation_region_area();
            //std::cout << "area:"<< area << " max area:"<< max_area << std::endl;
            /*if(area>max_area)
            {
                return true;
            }*/
        }

        return false;
    }

    // apply action to belief
    void apply_action(const MultipleAction& action)  {
        Action attend_action_;
        attend_action_.attend=true;

        Action dont_attend_action_;
        dont_attend_action_.attend=false;
        int j=0;
        for(int i=0; i<beliefs.size();++i)
        {
            if(action.attend[j]==i)
            {
                beliefs[i].apply_action(attend_action_);
                ++j;
            }
            else
            {
                beliefs[i].apply_action(dont_attend_action_);
            }

        }
        /*for(int i=0; i<action.attend.size();++i)
        {
            beliefs[action.attend[i]].apply_action(attend_action_);
        }*/
    }


    // return possible actions from this state
    void get_actions(std::vector<MultipleAction>& actions_) const  {
        actions_=actions;
    }


    // get a random action, return false if no actions found
    bool get_random_action(MultipleAction& action) const {
        int action_index=rand()%actions.size();
        action=actions[action_index];
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
    float max_area_ratio;
    unsigned int total_area;
    std::vector<MultipleAction> actions;
    std::vector<Belief> beliefs;
    float max_area;


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
