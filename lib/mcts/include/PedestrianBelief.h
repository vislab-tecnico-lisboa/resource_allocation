#pragma once

#include "ofxMSAmcts.h"
#include <opencv2/video/tracking.hpp>

#define kNumState 3
#define kNumObs 3
#define kNumActions		2
#define kTurnRangeMin	-30
#define kTurnRangeMax	30

using namespace msa::mcts;
using namespace std;
namespace tracking {
//--------------------------------------------------------------
//--------------------------------------------------------------
struct Action {
    bool attend;
};



//--------------------------------------------------------------
//--------------------------------------------------------------
class Belief {
public:

    //--------------------------------------------------------------
    // MUST HAVE METHODS (INTERFACE)

    Belief() : kalman_filter(kNumState,kNumObs,0)
    {
        // intialization of KF...
        kalman_filter.transitionMatrix = *(cv::Mat_<float>(3, 3) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
        cv::Mat_<float> measurement(kNumObs,1); measurement.setTo(cv::Scalar(0));

        kalman_filter.statePre.at<float>(0) = 0;
        kalman_filter.statePre.at<float>(1) = 0;
        kalman_filter.statePre.at<float>(2) = 0;
        setIdentity(kalman_filter.measurementMatrix);
        setIdentity(kalman_filter.processNoiseCov, cv::Scalar::all(1e-4));
        setIdentity(kalman_filter.measurementNoiseCov, cv::Scalar::all(10));
        setIdentity(kalman_filter.errorCovPost, cv::Scalar::all(.1));

        //reset();
    }

    // default constructors will do
    // copy and assignment operators should perform a DEEP clone of the given state
    //    State(const State& other);
    //    State& operator = (const State& other);


    // whether or not this state is terminal (reached end)
    bool is_terminal() const  {
        return false;
    }

    //  agent id (zero-based) for agent who is about to make a decision
    int agent_id() const {
        return 0;
    }

    // apply action to state
    void apply_action(const Action& action)  {

    }


    // return possible actions from this state
    void get_actions(std::vector<Action>& actions) const  {
        actions.resize(kNumActions);
    }


    // get a random action, return false if no actions found
    bool get_random_action(Action& action) const {

    }


    // evaluate this state and return a vector of rewards (for each agent)
    const vector<float> evaluate() const  {

    }


    // return state as string (for debug purposes)
    std::string to_string() const  {

    }


    //--------------------------------------------------------------
    // IMPLEMENTATION SPECIFIC

    cv::KalmanFilter kalman_filter;


    void reset() {

    }

    void draw() {

    }

};

}
