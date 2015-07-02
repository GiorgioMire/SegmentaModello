/* 
 * File:   main.cpp
 * Author: giorgio
 *
 * Created on 30 giugno 2015, 19.18
 */

#include <cstdlib>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/io/io.h>
#include <pcl-1.8/pcl/features/fpfh_omp.h>
#include <pcl-1.7/pcl/io/file_io.h>
#include <CalcolaFPFHsuPatch.h>
using namespace std;
typedef pcl::PointXYZRGB point;
/*
 * 
 */
int main(int argc, char** argv) {
    CalcolaFPFHsuPatch fpfh;
    std::string nomefile;
    
    nomefile=argv[1];
    fpfh.loadPCD(nomefile);
    fpfh.setRNormal(0.01);
    fpfh.computeNormals();
    fpfh.compute();
    fpfh.VisualizzaIstogramma();
    fpfh.SalvaDatiPatchCSV();
    
    

    return 0;
}

