#pragma once

#include "ofxMSAmcts.h"
#include <opencv2/video/tracking.hpp>

#define kNumState 6
#define kNumObs 3
#define kNumActions		2
#define kTurnRangeMin	-30
#define kTurnRangeMax	30
const double uniform_const=1.0/12.0;
const double scales_per_octave=8.0;
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
        //std::cout << "CONSTANT VELOCITY KALMAN"<< std::endl;

        // intialization of Kalman filter
        kalman_filter.transitionMatrix = *(cv::Mat_<float>(kNumState, kNumState) << 1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1);
        std::cout << kalman_filter.transitionMatrix<< std::endl;
        kalman_filter.statePre.at<float>(0) = 0;
        kalman_filter.statePre.at<float>(1) = 0;
        kalman_filter.statePre.at<float>(2) = 1.0;
        kalman_filter.statePre.at<float>(3) = 0;
        kalman_filter.statePre.at<float>(4) = 0;
        kalman_filter.statePre.at<float>(5) = 1.0;

        kalman_filter.statePost.at<float>(0) = 0;
        kalman_filter.statePost.at<float>(1) = 0;
        kalman_filter.statePost.at<float>(2) = 1.0;
        kalman_filter.statePost.at<float>(3) = 0;
        kalman_filter.statePost.at<float>(4) = 0;
        kalman_filter.statePost.at<float>(5) = 1.0;

        setIdentity(kalman_filter.measurementMatrix);
        //setIdentity(kalman_filter.processNoiseCov, cv::Scalar::all(1e-4));
        //setIdentity(kalman_filter.measurementNoiseCov, cv::Scalar::all(10));


        kalman_filter.processNoiseCov = *(cv::Mat_<float>(kNumState, kNumState) <<1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1);
        //setIdentity(kalman_filter.processNoiseCov, cv::Scalar::all(0));

        kalman_filter.measurementNoiseCov = *(cv::Mat_<float>(kNumObs, kNumObs) << 0.1,0,0, 0,0.1,0, 0,0,0.1);


        setIdentity(kalman_filter.errorCovPost, cv::Scalar::all(.1));
        std::cout << "init:" <<  kalman_filter.measurementNoiseCov << std::endl;
    }

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
        std::cout << " apply action: " << action.attend<<  std::endl;
        if(action.attend)
        {
            // Predict
            mean = kalman_filter.predict();
            covariance= kalman_filter.errorCovPre;
            //Observe
            mean = kalman_filter.correct(get_measurement(mean.rowRange(0,3)));
            covariance= kalman_filter.errorCovPost;

        }
        else
        {
            // Predict
            mean = kalman_filter.predict();
            covariance= kalman_filter.errorCovPre;
        }

        evaluate();
    }


    // return possible actions from this state
    void get_actions(std::vector<Action>& actions) const  {
        actions.resize(kNumActions);
        actions[0].attend=false;
        actions[1].attend=true;
    }


    // get a random action, return false if no actions found
    bool get_random_action(Action& action) const {
        action.attend=rand()%2;
        return true;
    }


    // evaluate this state and return a vector of rewards (for each agent)
    const vector<float> evaluate() const  {

        vector<float> rewards;

        // Negative entropy
        float reward=-std::log(cv::determinant(covariance));
        rewards.push_back(reward);

        std::cout << "  reward:" << reward << std::endl;
        return rewards;

    }


    // return state as string (for debug purposes)
    std::string to_string() const  {

    }


    //--------------------------------------------------------------
    // IMPLEMENTATION SPECIFIC

    cv::KalmanFilter kalman_filter;
    cv::Mat mean;
    cv::Mat covariance;
    float min_x=52, min_y=128;

    void reset() {

    }

    void draw() {

    }

    // Observation model is dependent on the current state
    cv::Mat get_measurement(const cv::Mat & state)
    {
        int n=8.0*log2(state.at<float>(2));
        float q_scale =(pow(2,(float)(n+1)/scales_per_octave)-pow(2,(float)n/scales_per_octave));
        float q_x=min_x*q_scale*0.5;
        float q_y=min_y*q_scale*0.5;

        cv::Mat cov=(cv::Mat_<float>(3, 3) << q_x*q_x*uniform_const, 0, 0,    0, q_y*q_y*uniform_const, 0,    0, 0, q_scale*q_scale*uniform_const);
        return state;
    }
};

}
