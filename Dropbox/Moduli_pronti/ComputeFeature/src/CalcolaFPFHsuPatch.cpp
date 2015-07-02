/* 
 * File:   CalcolaFPFHsuPatch.cpp
 * Author: giorgio
 * 
 * Created on 30 giugno 2015, 21.42
 */

#include <CalcolaFPFHsuPatch.h>



#define DEBUG 1

CalcolaFPFHsuPatch::CalcolaFPFHsuPatch() {

#if(DEBUG==1)

    cout << "\n Costruttore \n";

#endif

    this->fpfhs = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>);
    this->cloud = pcl::PointCloud<point>::Ptr(new pcl::PointCloud<point>);
    this->normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal> ());

#if(DEBUG==1)

    cout << "\n Costruttore andato a buon fine \n";

#endif

}

CalcolaFPFHsuPatch::CalcolaFPFHsuPatch(const CalcolaFPFHsuPatch& orig) {
}

CalcolaFPFHsuPatch::~CalcolaFPFHsuPatch() {
}

void CalcolaFPFHsuPatch::compute() {

#if(DEBUG==1)

    cout << "\n Compute \n";

#endif

#if(DEBUG==1)

    cout << "\n fpfh created \n";

#endif
    pcl::FPFHEstimationOMP<point, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(this->cloud);
    fpfh.setInputNormals(this->normals);

#if(DEBUG==1)

    cout << "\n cloud and normals set \n";

#endif


#if(DEBUG==1)

    cout << "\n kdtree created\n";

#endif
    pcl::KdTreeFLANN<point>::Ptr treeflann(new pcl::KdTreeFLANN<point>);
    pcl::search::KdTree<point>::Ptr tree(new pcl::search::KdTree<point>);
    treeflann->setInputCloud(this->cloud);
#if(DEBUG==1)

    cout << "\n ksearch set \n";

#endif
    fpfh.setKSearch(this->cloud->points.size());
    fpfh.setSearchMethod(tree);

#if(DEBUG==1)

    cout << "\n point_indices created \n";

#endif
    pcl::PointIndicesPtr idx(new pcl::PointIndices);




    int K = 1;

    std::vector<int> Nearest_idx;
    std::vector<float> Dist_idx(K);

    Nearest_idx.resize(K);
    Dist_idx.resize(K);

#if(DEBUG==1)

    cout << "\n vectors idx created \n";
    cout << "\n Centroide \n";
    cout << "x\t" << this->Centroid.x << "\n";
    cout << "y\t" << this->Centroid.y << "\n";
    cout << "z\t" << this->Centroid.z << "\n";
    cout << "rgba\t" << this->Centroid.rgba << "\n";
    cout << "\n K \n";
    cout << K;
    cout << "\n Nearest idx \n";
    cout << Nearest_idx[0];



    cout << Dist_idx.size();
    cout << "\n Dist idx \n";
#endif 



    K = treeflann->nearestKSearch(this->Centroid, K, Nearest_idx, Dist_idx);

#if(DEBUG==1)

    cout << "\n nearest search performed\n";

#endif
    /*Ultimo punto aggiunto ovvero il centroide*/
    idx->indices.push_back(Nearest_idx[0]);
    fpfh.setIndices(idx);
#if(DEBUG==1)

    cout << "\n idx set\n";

#endif




    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33> ());

#if(DEBUG==1)

    cout << "\n fpfh point cloud\n";

#endif

    fpfh.compute(*(this->fpfhs));
#if(DEBUG==1)

    cout << "\n fpfh computed\n";

#endif



#if(DEBUG==1)

    cout << "\n Compute andato a buon fine \n";

#endif


}

void CalcolaFPFHsuPatch::computeNormals() {
#if(DEBUG==1)

    cout << "\n Computing Normals\n";

#endif
    pcl::NormalEstimationOMP<point, pcl::Normal> ne;
    ne.setInputCloud(this->cloud);

    pcl::search::KdTree<point>::Ptr tree(new pcl::search::KdTree<point> ());
    ne.setSearchMethod(tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(this->RNormal);
    ne.compute(*(this->normals));
#if(DEBUG==1)

    cout << "\n Computing Normals andata a buon fine\n";

#endif
};

void CalcolaFPFHsuPatch::loadNormals(pcl::PointCloud<pcl::Normal>::Ptr n) {
    this->normals = n;
};

void CalcolaFPFHsuPatch::loadPCD(std::string nomefile) {

#if(DEBUG==1)

    cout << "\n Loading PCD\n";

#endif

    pcl::io::loadPCDFile<point>(nomefile, *(this->cloud));
    this->nomefile = nomefile;

    this->CalcolaCentroide();

#if(DEBUG==1)

    cout << "\n Loading PCD andata  a buon fine \n";

#endif
};

void CalcolaFPFHsuPatch::CalcolaCentroide() {
#if(DEBUG==1)

    cout << "\n Computing Centroid  \n";

#endif

    Eigen::Vector4f c;
    pcl::compute3DCentroid<point>(*(this->cloud), c);
    this->Centroid.x = c.data()[0];
    this->Centroid.y = c.data()[1];
    this->Centroid.z = c.data()[2];


#if(DEBUG==1)

    cout << "\n Computing Centroid a buon fine \n";

#endif
}

void
CalcolaFPFHsuPatch::VisualizzaIstogramma() {
    //pcl::FPFHSignature33 hist=fpfhs->points[0];
    pcl::visualization::PCLHistogramVisualizer hview;
    hview.addFeatureHistogram<pcl::FPFHSignature33>(*(this->fpfhs), 33, "fpfh", 640, 480);

    VisualizzatorePatch vp;
    vp.setFile(this->nomefile);

    hview.spinOnce(200);
    while (!vp.Viewer->wasStopped()) {
        vp.Viewer->spinOnce(200);
    }

}

void
CalcolaFPFHsuPatch::SalvaDatiPatchCSV() {


    bool primaCifra = true;
    int num_idx;
    std::stringstream number;
    for (unsigned int i = 0; i < this->nomefile.size(); i++) {
        if (isdigit(this->nomefile[i])) {
            if (primaCifra) {
                num_idx = i;
                primaCifra = false;
            };
            number << this->nomefile[i];
        };
    }

    std::string filePoseFrame = this->nomefile.substr(0, num_idx - 9) + "_SGURF" + number.str() + ".txt";
    cout << endl << "file:" << filePoseFrame << endl;

    ifstream PoseStream(filePoseFrame);
    Eigen::Matrix4f frame;

    for (int x = 0; x < 4; x++)
        for (int y = 0; y < 4; y++) {

            PoseStream>> frame(x, y);
        };

   PoseStream.close();



    ofstream f(this->nomefile.substr(0, this->nomefile.size() - 4)+"_summary.csv");
    std::string separator=",";
    std::string newline="\n";
    
    f<<"ID"+separator;
    f<<"Centroid_x"+separator+"Centroid_y"+separator+"Centroid_z"+separator;
    f<<"Quaternion_x"+separator+"Quaternion_y"+separator+"Quaternion_z"+separator+"Quaternion_w";
    for(int i=0; i<33;i++)    f<<separator+"FPFH_"+std::to_string(i);
    
    f<<newline;
    
    Eigen::Vector3f centroid;
    centroid<<frame.block<3,1>(0,3);
    f<<number.str()+separator;
    f<<std::to_string(centroid[0])
            +separator+
            std::to_string(centroid[1])+separator+
            std::to_string(centroid[2])+separator;
    
    Eigen::Quaternionf q(frame.block<3,3>(0,0));
    f<<std::to_string(q.x())+separator+std::to_string(q.y())+separator+std::to_string(q.z())+separator+std::to_string(q.w());
    for(int i=0; i<33;i++)    f<<separator+std::to_string(this->fpfhs->points[0].histogram[i]);
    
 
    
    f.close();




}

