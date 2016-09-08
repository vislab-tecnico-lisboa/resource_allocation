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


class MyKalmanFilter : public cv::KalmanFilter
{
public:

    MyKalmanFilter( const MyKalmanFilter& other )
    {
        statePre=other.statePre.clone();           //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
        statePost=other.statePost.clone();          //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
        transitionMatrix=other.transitionMatrix.clone();   //!< state transition matrix (A)
        measurementMatrix=other.measurementMatrix.clone();  //!< measurement matrix (H)
        processNoiseCov=other.processNoiseCov.clone();    //!< process noise covariance matrix (Q)
        measurementNoiseCov=other.measurementNoiseCov.clone();//!< measurement noise covariance matrix (R)
        errorCovPre=other.errorCovPre.clone();        //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
        gain=other.gain.clone();               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
        errorCovPost=other.errorCovPost.clone();       //!< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)

        // do something with bar
    }

    MyKalmanFilter(int dynamParams, int measureParams, int controlParams=0, int type=CV_32F):KalmanFilter(dynamParams, measureParams, controlParams, type)
    {}
};

//--------------------------------------------------------------
//--------------------------------------------------------------
struct Action {
    bool attend;
};



//--------------------------------------------------------------
//--------------------------------------------------------------
class Belief {
public:
    unsigned int id;
    float alpha_c;
    float alpha_s;
    //--------------------------------------------------------------
    // MUST HAVE METHODS (INTERFACE)
    Belief( const Belief& other) :
        kalman_filter(other.kalman_filter),
        mean(other.mean),
        covariance(other.covariance),
        id(other.id)
    {}

    Belief(const float & alpha_c_,
           const float & alpha_s_,
           const unsigned int & id_,
           const cv::Mat & transition_matrix_,
           const cv::Mat & measurement_matrix_,
           const cv::Mat & processNoiseCov_,
           const cv::Mat & state_,
           const cv::Mat & cov_
           ) :
        kalman_filter(kNumState,kNumObs,0),
        alpha_c(alpha_c_),
        alpha_s(alpha_s_),
        id(id_)
    {
        // intialization of Kalman filter
        kalman_filter.transitionMatrix = transition_matrix_;
        kalman_filter.measurementMatrix=measurement_matrix_;
        kalman_filter.processNoiseCov =processNoiseCov_;
        kalman_filter.statePre=state_;
        kalman_filter.statePost=state_;
        kalman_filter.errorCovPre=cov_;
        kalman_filter.errorCovPost=cov_;

        mean=kalman_filter.statePre.rowRange(0,3);
        covariance=kalman_filter.errorCovPost.rowRange(0,3).colRange(0,3);
        //std::cout << "init:" <<  kalman_filter.measurementNoiseCov << std::endl;
    }



    // whether or not this belief is terminal (reached end)
    bool is_terminal() {

    }

    //  agent id (zero-based) for agent who is about to make a decision
    int agent_id() const {
        return 0;
    }

    // apply action to state
    void apply_action(const Action& action)  {
        //IT SHOULD FIRST OBSERVE AND THEN PREDICT
        std::cout << "     id: "<< id << " action: "<< action.attend<<  std::endl;
        if(action.attend)
        {
            //Observe
            //std::cout << "before:" <<covariance<<std::endl;
            mean = kalman_filter.correct(get_measurement(mean.rowRange(0,3)+10.0));
            covariance = kalman_filter.errorCovPost.rowRange(0,3).colRange(0,3);
            //std::cout << "after:" <<covariance<<std::endl;
            //std::cout << "measurementNoiseCov:" << kalman_filter.measurementNoiseCov<<std::endl;
        }

        // Predict
        mean = kalman_filter.predict();
        covariance = kalman_filter.errorCovPre.rowRange(0,3).colRange(0,3);
        //std::cout << "  cov:"<< covariance << std::endl<< std::endl;
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
    const float evaluate() const  {


        // Negative entropy
        float reward=-std::log(cv::determinant(covariance));

        //std::cout << "  reward:" << reward << std::endl;
        return reward;

    }


    // return state as string (for debug purposes)
    std::string to_string() const  {

    }


    //--------------------------------------------------------------
    // IMPLEMENTATION SPECIFIC

    MyKalmanFilter kalman_filter;
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
        kalman_filter.measurementNoiseCov=cov;
        return state;
    }

    float compute_observation_region_area()
    {
        float scale=kalman_filter.statePre.at<float>(2);
        float centroid_uncertainty=sqrt(kalman_filter.errorCovPre.at<float>(0,0));
        float scale_uncertainty=sqrt(kalman_filter.errorCovPre.at<float>(2,2));

        int total_scale=(scale+alpha_c*centroid_uncertainty+alpha_s*scale_uncertainty);
        int n_rows=total_scale * min_y;
        int n_cols=total_scale * min_x;
        //std::cout << "  centroid_uncertainty:"<< centroid_uncertainty << " scale_uncertainty:"<< scale_uncertainty << std::endl;

        float area=n_rows*n_cols;
        //std::cout << kalman_filter.errorCovPre.at<float>(2,2) << std::endl;
        return area;
        // Region size constraint: if total area > max allowed - > terminal belief

    }
};

}
