#include "PedestrianBelief.h"
#include "MultiplePedestrianBelief.h"
#include "ofxMSAmcts.h"
#include "MSALoopTimer.h"
int main(int argc, char** argv)
{
    // Image size
    unsigned int width=1920;
    unsigned int height=1080;
    unsigned int total_area=width*height;

    //resource constraints
    unsigned int max_targets_=1;
    float max_area_ratio_=0.2;

    // region size
    float alpha_c=5;
    float alpha_s=5;



    // Initialize targets
    unsigned int total_targets_=12;
    std::vector<tracking::Belief> pedestrian_beliefs;

    pedestrian_beliefs.reserve(total_targets_);
    for(int i=0;i<total_targets_;++i)
    {
        float T = 1.0/10.0;

        // intialization of Kalman filter
        cv::Mat transitionMatrix = *(cv::Mat_<float>(kNumState, kNumState) << 1,0,0,T,0,0, 0,1,0,0,T,0, 0,0,1,0,0,T, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1);
        cv::Mat measurementMatrix(kNumObs,kNumState, CV_32F);
        setIdentity(measurementMatrix);
        cv::Mat processNoiseCov = *(cv::Mat_<float>(kNumState, kNumState) <<1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1);

        cv::Mat state(kNumState,1, CV_32F);;
        state.at<float>(0) = 200;
        state.at<float>(1) = 200;
        state.at<float>(2) = 4.0;
        state.at<float>(3) = 10;
        state.at<float>(4) = 10;
        state.at<float>(5) = 0.1;
        cv::Mat errorCovPre(kNumState,kNumState, CV_32F);

        setIdentity(errorCovPre, cv::Scalar::all(1));

        //std::cout << kalman_filter.transitionMatrix<< std::endl;
        //setIdentity(kalman_filter.processNoiseCov, cv::Scalar::all(1e-4));
        //setIdentity(kalman_filter.measurementNoiseCov, cv::Scalar::all(10));
        //setIdentity(kalman_filter.processNoiseCov, cv::Scalar::all(0));
        //kalman_filter.measurementNoiseCov = *(cv::Mat_<float>(kNumObs, kNumObs) << 0.1,0,0, 0,0.1,0, 0,0,0.1);

        pedestrian_beliefs.push_back(tracking::Belief(alpha_c,alpha_s,i,transitionMatrix,measurementMatrix,processNoiseCov,state,errorCovPre));
    }

    unsigned int max_millis=100;
    unsigned int simulation_depth=3;


    //msa::mcts::UCT<tracking::Belief, tracking::Action> uct(max_millis, simulation_depth);
    msa::mcts::UCT<tracking::MultipleBelief<tracking::Belief>, tracking::MultipleAction> uct(max_millis, simulation_depth);


    max_targets_=10;
    while(max_targets_>=2)
    {
        tracking::MultipleBelief<tracking::Belief> belief(pedestrian_beliefs,--max_targets_,total_area,max_area_ratio_);

        tracking::MultipleAction mult_action=uct.run(belief);

    }

    return 0;
}
