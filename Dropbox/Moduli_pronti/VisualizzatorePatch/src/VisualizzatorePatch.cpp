/* 
 * File:   VisualizzatorePatch.cpp
 * Author: giorgio
 * 
 * Created on 28 giugno 2015, 16.00
 */


#include <VisualizzatorePatch.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/visualization/point_cloud_color_handlers.h>
#include <pcl-1.7/pcl/visualization/interactor_style.h>
VisualizzatorePatch::VisualizzatorePatch() 
{this->Viewer= pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer);
                          this->cloud=pcl::PointCloud<point>::Ptr(new pcl::PointCloud<point>); };

VisualizzatorePatch::VisualizzatorePatch(const VisualizzatorePatch& orig) {
}

VisualizzatorePatch::~VisualizzatorePatch() {
}

void VisualizzatorePatch::setFile(std::string file){
this->PCDFile=file;
int num_idx;
std::stringstream number;
pcl::PointCloud<point> pc;
pcl::io::loadPCDFile <pcl::PointXYZRGB> (this->PCDFile, *(this->cloud)) ;
bool primaCifra=true;
 for (unsigned int i=0; i < file.size(); i++)
 {        if (isdigit(file[i])){ if(primaCifra){num_idx=i;primaCifra=false;};
 number<<file[i];};
 }



cout<<endl<<"numero:"<<number<<endl;


std::string filePoseFrame=file.substr(0,num_idx-9)+"_SGURF"+number.str()+".txt";
cout<<endl<<"file:"<<filePoseFrame<<endl;

this->ReferenceFrameFile=filePoseFrame;
ifstream f(this->ReferenceFrameFile);
Eigen::Matrix4f frame;

for(int x=0;x<4;x++)
    for(int y=0;y<4;y++){
        
        f>>(this->frame(x,y) );};

f.close();
pcl::visualization:: PointCloudColorHandlerRGBField<point> rgb((this->cloud));
this->Viewer->addPointCloud<point>((this->cloud),rgb,"cloud");
this->Viewer->addCoordinateSystem(0.02,Eigen::Affine3f(this->frame));
this->Viewer->setBackgroundColor(0.2,0.2,0.2);



}



