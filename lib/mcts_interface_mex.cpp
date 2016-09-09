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
        if(nrhs!=9)
        {
            mexErrMsgTxt("wrong inputs number (should be 8)");
        }
        std::cout << "OLA" << std::endl;
        double width=*(double *) mxGetPr(prhs[1]);
        double height=*(double *) mxGetPr(prhs[2]);
        double capacity_percentage=*(double *) mxGetPr(prhs[3]);
        double max_items=*(double *) mxGetPr(prhs[4]);
        double min_width=*(double *) mxGetPr(prhs[5]);
        double min_height=*(double *) mxGetPr(prhs[6]);
        double max_simulation_time_millis=*(double *) mxGetPr(prhs[7]);
        double simulation_depth=*(double *) mxGetPr(prhs[8]);

        msa::mcts::UCT<tracking::MultipleBelief<tracking::Belief>, tracking::MultipleAction> *mcts_= new msa::mcts::UCT<tracking::MultipleBelief<tracking::Belief>, tracking::MultipleAction>((unsigned int) max_simulation_time_millis, (unsigned int) simulation_depth);

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
    
    
    if (!strcmp("get_probability_maps", cmd))
    {
        // Check parameters
        //std::cout <<nrhs<<std::endl;
        
        if (nrhs !=2)
            mexErrMsgTxt("detect: Unexpected arguments.");
        
        //plhs[0]=MxArray(mcts_->probability_maps);
        return;
    }
    
    // Call the various class methods
    if (!strcmp("get_action", cmd))
    {
        // Check parameters
        
        if (nrhs !=5)
            mexErrMsgTxt("detect: Unexpected arguments.");
        const mwSize* size=mxGetDimensions(prhs[2]);
        //std::cout << "size:" << size[1] << " " << size[0] << std::endl;
        cv::Mat state_means=cv::Mat(size[1],size[0],CV_64F,mxGetData(prhs[2]),0);
        cv::Mat state_covariances=cv::Mat(size[1],size[0],CV_64F,mxGetData(prhs[3]),0);
        cv::Mat left_upper_corners=cv::Mat(size[1],size[0],CV_64F,mxGetData(prhs[4]),0);
        std::cout << "state_means: " << state_means << std::endl;
        //std::cout << "state_covariances: " << state_covariances << std::endl;
        //std::cout << "left_upper_corner: " << left_upper_corners << std::endl;
        
        tic();
        /*std::vector<cv::Rect> rois=mcts_->getROIS(
                state_means,
                state_variances);
        double time_elapsed=toc_();
        std::cout << "time elapsed (optimization):" << time_elapsed<< std::endl;

        plhs[0]=MxArray(rois);
        plhs[1]=mxCreateDoubleScalar(time_elapsed);*/

        return;
    }
    
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}
