#pragma once

#include "PedestrianBelief.h"
#include "ofxMSAmcts.h"
#include <opencv2/video/tracking.hpp>
#include <iostream>

#define kNumState 6
#define kNumObs 3
#define kNumActions		2

using namespace msa::mcts;
using namespace std;
namespace tracking {

//--------------------------------------------------------------
//--------------------------------------------------------------
class MultipleAction {
public:
    MultipleAction() {

    }


    MultipleAction(const int & max_attend, const unsigned int & id_): id(id_)
    {
        //std::cout << " max_attend:"<<max_attend<< std::endl;
        attend.resize(max_attend);
    }

    unsigned int get_id()
    {
        return id;
    }



    std::vector<int> attend;
    unsigned int id;
    friend inline std::ostream& operator<<(std::ostream& os, const MultipleAction &b)
    {
        std::cout << "[";
        for(unsigned int i=0; i< b.attend.size()-1; ++i)
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

    int count;
    //--------------------------------------------------------------
    // MUST HAVE METHODS (INTERFACE)
    MultipleBelief( const MultipleBelief& other_) :
        beliefs(other_.beliefs),
        actions(other_.actions),
        max_attend(other_.max_attend),
        total_area(other_.total_area),
        max_area_ratio(other_.max_area_ratio),
        max_area(other_.max_area)
    {}

    MultipleBelief(const std::vector<Belief> & beliefs_, const unsigned int & max_attend_=1, const unsigned int & total_area_=10000000000, const float & max_area_ratio_=1.0) :
        beliefs(beliefs_),
        max_attend(max_attend_),
        total_area(total_area_),
        max_area_ratio(max_area_ratio_),
        max_area((float)total_area*max_area_ratio),
        count(0)
    {

        /*std::cout << "max_atend:" << max_attend << std::endl;
        std::cout << "total_area:" << total_area << std::endl;
        std::cout << "max_area_ratio:" << max_area_ratio_ << std::endl;
        std::cout << "max_area:" << max_area << std::endl;
        std::cout << "trackers:"<< beliefs_.size() << std::endl;*/

        // Compute all possible actions vector
        unsigned int possible_actions=choose(beliefs.size(),max_attend_);
        //std::cout << "possible actions: " << possible_actions << std::endl;
        //actions.reserve(possible_actions);
        for(int i=0; i<possible_actions;++i)
        {
            int id=i+1;
            actions.push_back(MultipleAction(max_attend,id));
        }

        // Create combinations
        create_actions(0, max_attend_);

        /*for(int i=0; i< actions.size(); ++i)
        {
            std::cout << actions[i] ;
        }
        std::cout << std::endl;*/
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
            if(area>max_area)
            {
                //std::cout << "area:"<< area << " max area:"<< max_area << std::endl;

                return true;
            }
        }
        //std::cout << "milagre"<< std::endl;
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
    }


    // return possible actions from this state
    void get_actions(std::vector<MultipleAction>& actions_) const  {
        actions_=actions;
    }


    // get a random action, return false if no actions found
    bool get_random_action(MultipleAction& action) const {
        // This should be more clever

        std::random_device rd;
        std::mt19937 gen(rd());
        std::vector<int> probabilities;

        for(int i=0; i<actions.size();++i)
        {
            probabilities.push_back(evaluateAttendingAction(actions[i]));
        }

        std::discrete_distribution<> d(probabilities.begin(),probabilities.end());


        //int action_index=rand()%actions.size();
        int action_index=d(gen);
        action=actions[action_index];
        return true;
    }


    // evaluate this state and return a vector of rewards (for each agent)
    const float evaluate() const  {
        float reward=0.0;
        for(int i=0; i<beliefs.size();++i)
        {
            //negative entropy (closer to zero is better)
            reward+=-beliefs[i].entropy();
        }

        return reward;
    }

    // evaluate attending action
    const float evaluateAttendingAction(const MultipleAction & action) const  {
        float reward=0.0;
        for(int i=0; i<action.attend.size();++i)
        {
            // Bigger entropy should be better
            reward+=beliefs[action.attend[i]].entropy();
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
