// Mex stuff
#include "mex.h"
#include "mc_convert/mc_convert.hpp"
#include "mex_handle.hpp"
#include "mexopencv.hpp"
#include "MxArray.hpp"
//General purpose includes
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <ctime> 
#include <stack>
#include <string>
#include <sstream>
//#include <thread>
#include <fstream>
#include <iostream>
#include <stack>
#include <ctime>

//OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/shared_ptr.hpp>

#include "mcts/include/MSALoopTimer.h"
#include "mcts/include/PedestrianBelief.h"
#include "mcts/include/MultiplePedestrianBelief.h"
#include "mcts/include/TreeNodeT.h"
#include "mcts/include/ofxMSAmcts.h"

std::stack<clock_t> tictoc_stack;
double max_targets;
double total_area;
double max_area_ratio;
int action_mode;
void tic() {
    tictoc_stack.push(clock());
}

double toc_()
{
    double time_elapsed=((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
    //std::cout << "Time elapsed: " << time_elapsed << std::endl;
    tictoc_stack.pop();
    
    return time_elapsed;
}
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Get the command string
    char cmd[64];
    if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
        mexErrMsgTxt("First input should be a command string less than 64 characters long.");
    
    // New
    if (!strcmp("new", cmd))
    {
        // Check parameters
        //std::cout << "nrhs: " << nrhs << std::endl;
        if(nrhs!=10)
        {
            mexErrMsgTxt("wrong inputs number (should be 9)");
        }
        double width=*(double *) mxGetPr(prhs[1]);
        double height=*(double *) mxGetPr(prhs[2]);
        double capacity_percentage=*(double *) mxGetPr(prhs[3]);
        double max_items=*(double *) mxGetPr(prhs[4]);
        double min_width=*(double *) mxGetPr(prhs[5]);
        double min_height=*(double *) mxGetPr(prhs[6]);
        double max_simulation_time_millis=*(double *) mxGetPr(prhs[7]);
        double simulation_depth=*(double *) mxGetPr(prhs[8]);
        double action_mode_=*(double *) mxGetPr(prhs[9]);
        action_mode=(int)action_mode_;
        std::cout << "action_mode: " << action_mode << std::endl;
        

        /*std::cout << "width:"<<width << std::endl;
        std::cout << "height:"<<height << std::endl;
        std::cout << "capacity_percentage:"<<capacity_percentage << std::endl;
        std::cout << "max_items:"<<max_items << std::endl;
        std::cout << "min_width:"<<min_width << std::endl;
        std::cout << "min_height:"<<min_height << std::endl;
        std::cout << "max_simulation_time_millis:"<<max_simulation_time_millis << std::endl;
        std::cout << "simulation_depth:"<<simulation_depth << std::endl;*/
        
        total_area=width*height;
        max_targets=max_items;
        max_area_ratio=capacity_percentage;
        //std::cout <<"capacity_percentage:" << capacity_percentage << std::endl;
        msa::mcts::UCT<tracking::MultipleBelief<tracking::Belief>, tracking::MultipleAction> *mcts_= new msa::mcts::UCT<tracking::MultipleBelief<tracking::Belief>, tracking::MultipleAction>(
                (unsigned int) max_simulation_time_millis, 
                (unsigned int) simulation_depth);

        plhs[0] = convertPtr2Mat< msa::mcts::UCT<tracking::MultipleBelief<tracking::Belief>, tracking::MultipleAction> >(mcts_);
        
        return;
    }
    
// Check there is a second input, which should be the class icleranstance handle
    if (nrhs < 2)
        mexErrMsgTxt("Second input should be a class instance handle.");
    
// Delete
    if (!strcmp("delete", cmd)) {
        // Destroy the C++ object
        destroyObject<msa::mcts::UCT<tracking::MultipleBelief<tracking::Belief>, tracking::MultipleAction> >(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    
    // Get the class instance pointer from the second input
    msa::mcts::UCT<tracking::MultipleBelief<tracking::Belief>, tracking::MultipleAction> *mcts_ = convertMat2Ptr<msa::mcts::UCT<tracking::MultipleBelief<tracking::Belief>, tracking::MultipleAction> >(prhs[1]);
       
    // Call the various class methods
    if (!strcmp("get_action", cmd))
    {
        // Check parameters
        //std::cout << nrhs << std::endl;
        if (nrhs !=6)
            mexErrMsgTxt("get action: Unexpected arguments.");
        const mwSize* size=mxGetDimensions(prhs[2]);

        cv::Mat state_means=cv::Mat(size[1],size[0],CV_64F,mxGetData(prhs[2]),0);       
        std::vector<tracking::Belief> pedestrian_beliefs;

        
        // intialization of Kalman filter
        cv::Mat transitionMatrix = *(cv::Mat_<float>(kNumState, kNumState) << 1,0,0,1,0,0, 0,1,0,0,1,0, 0,0,1,0,0,1, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1);
        cv::Mat measurementMatrix(kNumObs,kNumState, CV_32F);
        setIdentity(measurementMatrix);
        cv::Mat processNoiseCov = *(cv::Mat_<float>(kNumState, kNumState) <<1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1);

        const mwSize *dims; 
        mwIndex jcell;
        dims = mxGetDimensions(prhs[3]);
        int total_targets_=dims[0];
        pedestrian_beliefs.reserve(total_targets_);
        double alpha_c=*(double *) mxGetPr(prhs[4]);
        double alpha_s=*(double *) mxGetPr(prhs[5]);
        
        //std::cout << "alpha_c:" << alpha_c <<" alpha_s:" << alpha_s << std::endl;
        for (jcell=0; jcell<dims[0]; jcell++) {
            // Get covariance
            mxArray *cellArray;
            cellArray = mxGetCell(prhs[3],jcell);
            const mwSize* size=mxGetDimensions(cellArray);
            cv::Mat state_covariance=cv::Mat(size[1],size[0],CV_64F,mxGetData(cellArray),0);
            //std::cout << state_covariance << std::endl;
            
            unsigned int i=(unsigned int)jcell;
            cv::Mat state_mean=state_means(cv::Rect(0,i,6,1)).t();
            
            state_mean.convertTo(state_mean, CV_32F);
            state_covariance.convertTo(state_covariance, CV_32F);

            pedestrian_beliefs.push_back(tracking::Belief((float)alpha_c,(float)alpha_s,i,transitionMatrix,measurementMatrix,processNoiseCov,state_mean,state_covariance));
        }


        std::vector<int>* explored_actions(new std::vector<int>);
        std::vector<int>* explored_nodes(new std::vector<int>);
        
        tracking::MultipleBelief<tracking::Belief> belief(pedestrian_beliefs,max_targets,total_area,max_area_ratio);
        tracking::MultipleAction mult_action;
        if(action_mode==0)
        {
            std::cout << "action NORMAL mode" << std::endl;
         	tic();
            mult_action = mcts_->run(belief,explored_actions,explored_nodes);
        }else if(action_mode==1)
        {
            std::cout << "action RANDOM mode" << std::endl;
        	tic();
            mult_action = mcts_->random(belief,explored_actions,explored_nodes);
        }
        double time_elapsed=toc_();
        //std::cout << "time elapsed:" << time_elapsed << std::endl;
        plhs[0]=MxArray(mult_action.attend);
        plhs[1]=mxCreateDoubleScalar(time_elapsed);
        plhs[2]=MxArray(*explored_actions);
        plhs[3]=MxArray(*explored_nodes);

        /*for (int i=0; i< explored_nodes->size();++i)
        {
            std::cout << "" << (*explored_nodes)[i];
        }
        std::cout  << std::endl;*/
        delete explored_actions;
        delete explored_nodes;
        return;
    }
    
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}
