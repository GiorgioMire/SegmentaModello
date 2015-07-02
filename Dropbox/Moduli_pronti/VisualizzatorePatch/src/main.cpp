/* 
 * File:   main.cpp
 * Author: giorgio
 *
 * Created on 28 giugno 2015, 15.58
 */



#include <VisualizzatorePatch.h>
#include <pcl-1.8/pcl/common/common.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>


using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
    VisualizzatorePatch vp;
    vp.setFile(argv[1]);
    
    while( !vp.Viewer->wasStopped()){
        vp.Viewer->spinOnce();
    }

    return 0;
}

