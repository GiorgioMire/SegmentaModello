/* 
 * File:   CalcolaFPFHsuPatch.h
 * Author: giorgio
 *
 * Created on 30 giugno 2015, 21.42
 */

#pragma once
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/features/fpfh_omp.h>
#include <pcl-1.8/pcl/PointIndices.h>
#include <pcl-1.8/pcl/visualization/histogram_visualizer.h>
#include <pcl-1.8/pcl/io/io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/kdtree/kdtree_flann.h>
#include <VisualizzatorePatch.h>
#include <fstream>






typedef pcl::PointXYZRGB point;

class CalcolaFPFHsuPatch {
public:
    void CalcolaCentroide();
    
    void computeNormals();

    void loadNormals(pcl::PointCloud<pcl::Normal>::Ptr n);

    void loadCloud(pcl::PointCloud<point>::Ptr c){
    this->cloud=c;
    this->CalcolaCentroide();};
    
    void loadPCD(std::string nomefile);

    void compute();

    void setRNormal(float r) {
        this->RNormal = r;
    };

    CalcolaFPFHsuPatch();

    CalcolaFPFHsuPatch(const CalcolaFPFHsuPatch& orig);
    
    void VisualizzaIstogramma();
    void SalvaDatiPatchCSV();

    virtual ~CalcolaFPFHsuPatch();

private:

    float RNormal;
    std::string nomefile;
    pcl::PointCloud<point>::Ptr cloud;

    pcl::PointCloud<pcl::Normal>::Ptr normals;

    pcl::Normal NormalCentroid;

    point Centroid;

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs;
    
   
};




