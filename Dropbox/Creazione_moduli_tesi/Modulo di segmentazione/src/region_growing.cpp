#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <mylibforpcl.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/console/time.h>
#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/supervoxel_clustering.h>


//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

typedef pcl::PointXYZ PointTypeIO;
typedef pcl::PointNormal PointTypeFull;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

// Funzione per visualizzare

/*_________________________________________________________*/


void
addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                  PointCloudT &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

  //Iterate through all adjacent points, and add a center point to adjacent point pair
  PointCloudT::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
  for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
  {
    points->InsertNextPoint (supervoxel_center.data);
    points->InsertNextPoint (adjacent_itr->data);
  }
  // Create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
  // Add the points to the dataset
  polyData->SetPoints (points);
  polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
  for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
    polyLine->GetPointIds ()->SetId (i,i);
  cells->InsertNextCell (polyLine);
  // Add the lines to the dataset
  polyData->SetLines (cells);
  viewer->addModelFromPolyData (polyData,supervoxel_name);
}






bool
customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  

  Eigen::Map<const Eigen::Vector3f> point_a_normal(point_a.normal), point_b_normal(point_b.normal)
  ,point_ad(point_a.data),point_bd(point_b.data);
Eigen::Vector3f diff=
(point_ad-point_bd);
std::cout<<"distanza  "<<diff.norm()<<std::endl;
  if (diff.norm()< 0.02)
  {std::cout<<"distanza accettata "<<diff.norm()<<std::endl;
   
    if (fabs (point_a_normal.dot (point_b_normal)) > (std::cos(2/180.0*3.14))){
      std::cout<<"angolo accettato coseno:"<<fabs (point_a_normal.dot (point_b_normal))<<std::endl;
      if(fabs(point_a.curvature-point_b.curvature)<0.001)
      return (true);}
  }
 
  return (false);
}




typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizzatoreptr;
typedef pcl::visualization::PCLVisualizer visualizzatore;
typedef pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorhandle;
typedef pcl::PointCloud<pcl::Normal>::Ptr norptr;

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }
// float theta=3.14;
// Eigen::Affine3f transform = Eigen::Affine3f::Identity();
// Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
// transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
// transform2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
//   pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//   // You can either apply transform_1 or transform_2; they are the same
//   pcl::transformPointCloud (*cloud, *cloud, transform*transform2);


//_______________________________________________//
  //   cloud->sensor_orientation_.w () = 0.0;
  // cloud->sensor_orientation_.x () = 1.0;
  // cloud->sensor_orientation_.y () = 0.0;
  // cloud->sensor_orientation_.z () = 0.0;
  //____________________________________________//

  visualizzaXYZ(cloud);
//______________________________________________//

  float diam=diameter<pcl::PointXYZ>(cloud);
  std::cout<<std::endl<<"il diametro è"<<diam<<std::endl;
  pcXYZptr cloudfiltered(new pcl::PointCloud<pcl::PointXYZ>);
  filtra<pcl::PointXYZ>(cloud,cloudfiltered,diam*0.08/100.0);

visualizzaXYZ(cloudfiltered);

//_____________________________________________//

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudnoplane;
cloudnoplane=FindAndSubtractPlane <pcl::PointXYZ>(cloudfiltered, diam*1/100.0, 30000);
std::cout<<cloudnoplane->size();
visualizzaXYZ(cloudnoplane);

//___________________________________________//


pcl::PointCloud<pcl::PointXYZ>::Ptr cloudoutrem(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloudnoplane);
    outrem.setRadiusSearch(diam*7.0/100.0);
    outrem.setMinNeighborsInRadius (cloudnoplane->points.size()/100.0*7.0);
    // apply filter
    outrem.filter (*cloudoutrem);

  visualizzaXYZ(cloudoutrem);




  pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);
  pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
 pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::console::TicToc tt;

  pcl::search::KdTree<PointTypeIO>::Ptr search_tree (new pcl::search::KdTree<PointTypeIO>);

  // Set up a Normal Estimation class and merge data in cloud_with_normals
  std::cerr << "Computing normals...\n", tt.tic ();
  pcl::copyPointCloud (*cloudoutrem, *cloud_with_normals);
  pcl::NormalEstimationOMP<PointTypeIO, PointTypeFull> ne;
  ne.setInputCloud (cloudoutrem);
  ne.setSearchMethod (search_tree);
  ne.setRadiusSearch (diam*1.0/100.0);
  ne.compute (*cloud_with_normals);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";

pcl::copyPointCloud (*cloud_with_normals, *normals);
visualizzanormali<pcl::PointXYZ>(cloudoutrem,normals,1,diam*1.0/100);
/*............................................................*/
/* Segmenting */

PointCloudT::Ptr cloudrgb (new PointCloudT);
pcl::copyPointCloud(*cloudoutrem,*cloudrgb);



  
  float voxel_resolution = 0.008f;
  

  float seed_resolution = diam*0.5/100.0*3;


  float color_importance = 0.0f;
 
  float spatial_importance = 0.4f;
  
  float normal_importance = 1.0f;
  
  //////////////////////////////  //////////////////////////////
  ////// This is how to use supervoxels
  //////////////////////////////  //////////////////////////////

  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
  super.setUseSingleCameraTransform (false);
  super.setInputCloud (cloudrgb);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);


  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

  pcl::console::print_highlight ("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);
  pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (255, 255, 255);

PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_centroid_cloudxyz(new 
pcl::PointCloud<pcl::PointXYZ>);

pcl::console::print_info("voxel_centroid dim: %f and %f",
voxel_centroid_cloud->size(),voxel_centroid_cloudxyz->size());

pcl::copyPointCloud(*voxel_centroid_cloud,*voxel_centroid_cloudxyz);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> nero(voxel_centroid_cloudxyz, 0, 0, 0);
 viewer->addPointCloud (voxel_centroid_cloudxyz, nero,"voxel centroids");


  viewer->setPointCloudRenderingProperties 
  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,80, "voxel centroids");
  
  
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");
  
viewer->setCameraPosition (-0.166918, 0.0997064, -0.757201, 0.0177513, -0.999335, 0.0318579);
  PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
  //viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2, "labeled voxels");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");

  PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
  //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
  //viewer->addPointCloudNormals<pcl::PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

  //pcl::console::print_highlight ("Getting supervoxel adjacency\n");
  // std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
  // super.getSupervoxelAdjacency (supervoxel_adjacency);
  // //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
  // std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
  // for ( ; label_itr != supervoxel_adjacency.end (); )
  // {
  //   //First get the label
  //   uint32_t supervoxel_label = label_itr->first;
  //   //Now get the supervoxel corresponding to the label
  //   pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

  //   //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
  //   PointCloudT adjacent_supervoxel_centers;
  //   std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
  //   for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
  //   {
  //     pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
  //     adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
  //   }
  //   //Now we make a name for this polygon
  //   std::stringstream ss;
  //   ss << "supervoxel_" << supervoxel_label;
  //   //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
  //   //addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
  //   //Move iterator forward to next label
  //   label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
  // }

 std::cerr << "Saving...\n", tt.tic ();
 pcl::io::savePCDFile ("output.pcd", *labeled_voxel_cloud);
 std::cerr << ">> Done: " << tt.toc () << " ms\n";

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
  }




























/*___________________________________________________*/
  // Data containers used
 

//   // Set up a Conditional Euclidean Clustering class
//   std::cerr << "Segmenting to clusters...\n", tt.tic ();
//   pcl::ConditionalEuclideanClustering<PointTypeFull> cec (true);
//   cec.setInputCloud (cloud_with_normals);
//   cec.setConditionFunction (&customRegionGrowing);
//   cec.setClusterTolerance (diam*1.0/100);
//   cec.setMinClusterSize (cloud_with_normals->points.size () / 100.0*0.5);//cloud_with_normals->points.size () / 100.0);
//   cec.setMaxClusterSize (cloud_with_normals->points.size () / 100.0*60.0);
//   cec.segment (*clusters);
//   cec.getRemovedClusters (small_clusters, large_clusters);
//   std::cerr << ">> Done: " << tt.toc () << " ms\n";

// std::cout<<"Sono stati trovati "<<clusters->size ()<<" clusters"<<std::endl;
// std::cout<<"Sono stati trovati "<<large_clusters->size ()<<" large clusters"<<std::endl;
// std::cout<<"Sono stati trovati "<<small_clusters->size ()<<" small clusters"<<std::endl;
// pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
// pcl::copyPointCloud(*cloudoutrem,*cloud_out);
//     // Using the intensity channel for lazy visualization of the output
//   for (int i = 0; i < small_clusters->size (); ++i)
//     for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
//       cloud_out->points[(*small_clusters)[i].indices[j]].intensity = -2.0;

//   for (int i = 0; i < large_clusters->size (); ++i)
//     for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
//       cloud_out->points[(*large_clusters)[i].indices[j]].intensity = +10.0;

//   for (int i = 0; i < clusters->size (); ++i)
//   {
//     int label = rand () % 20;
//     std::cout<< label<<std::endl;

//     for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
//       cloud_out->points[(*clusters)[i].indices[j]].intensity = label;
//   }

//     // Save the output point cloud
//   std::cerr << "Saving...\n", tt.tic ();
//   pcl::io::savePCDFile ("output.pcd", *cloud_out);
//   std::cerr << ">> Done: " << tt.toc () << " ms\n";

  // // Using the intensity channel for lazy visualization of the output
  // for (int i = 0; i < small_clusters->size (); ++i)
  //   for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
  //     cloud_out->points[(*small_clusters)[i].indices[j]].intensity = -2.0;
  // for (int i = 0; i < large_clusters->size (); ++i)
  //   for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
  //     cloud_out->points[(*large_clusters)[i].indices[j]].intensity = +10.0;
  // for (int i = 0; i < clusters->size (); ++i)
  // {
  //   int label = rand () % 8;
  //   for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
  //     cloud_out->points[(*clusters)[i].indices[j]].intensity = label;
  // }

  // // Save the output point cloud
  // std::cerr << "Saving...\n", tt.tic ();
  // pcl::io::savePCDFile ("output.pcd", *cloud_out);
  // std::cerr << ">> Done: " << tt.toc () << " ms\n";



  /*pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);*/

  // pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  // reg.setMinClusterSize (100);
  // reg.setMaxClusterSize (300);
  // reg.setSearchMethod (tree);
  // reg.setNumberOfNeighbours (2000);
  // reg.setInputCloud (cloudnoplane);
  // //reg.setIndices (indices);
  // reg.setInputNormals (normals);
  // reg.setSmoothnessThreshold (3/ 180.0 * 3.14);
  // reg.setCurvatureThreshold (2);

  // std::vector <pcl::PointIndices> clusters;
  // reg.extract (clusters);


  // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  // pcl::io::savePCDFileASCII ("colorata.pcd", *colored_cloud);
  // visualizza(colored_cloud);

  return (0);
}
