#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <opencv2/opencv.hpp>
#include <opencv2/flann/flann.hpp>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include<iostream>
#include<cmath> 
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <pcl/registration/super4pcs.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <gr/shared.h>
#include "demo-utils.h"


// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//const double camera_cx = 319.5;//325.5//319.5 310.95 310.95
//const double camera_cy = 239.5;//253.5//239.5 234.74 234.74
//const double camera_fx = 570.3422;//518.0//570.3422(openni2) 615.377
//const double camera_fy = 570.3422;//519.0//570.3422(openni2) 615.377

const double camera_cx = 312.83380126953125;//325.5//319.5 310.95 310.95
const double camera_cy = 241.61764526367188;//253.5//239.5 234.74 234.74
const double camera_fx = 622.0875244140625;//518.0//570.3422(openni2) 615.377
const double camera_fy = 622.0875854492188;//519.0//570.3422(openni2) 615.377
const double depth_scale = 1.0;//0.00012498664727900177;
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_input (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_inputttt (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_input_second (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz_kdtree (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr final_model (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr object_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned_sparse_xyz (new pcl::PointCloud<pcl::PointXYZ> ());

pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned_xyz_icped (new pcl::PointCloud<pcl::PointXYZ> ()); 
 
pcl::PointCloud<pcl::PointXYZ>::Ptr object_kd_search_xyz(new pcl::PointCloud<pcl::PointXYZ> ());

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_rgb (new pcl::PointCloud<pcl::PointXYZRGBA> ());
pcl::PointCloud<pcl::PointNormal>::Ptr scene_with_normals (new pcl::PointCloud<pcl::PointNormal> ()); 

using namespace gr;
using namespace cv;
// Align a rigid object to a scene with clutter and occlusions

 int outkey=0;


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2); 
    pcl::fromPCLPointCloud2(pcl_pc2,*scene_input);
 outkey=1;
}


int
main (int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "test_real");

    // 订阅base_link的位置话题
    ros::NodeHandle node;




  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = node.subscribe ("/camera/depth/color/points", 1, cloud_cb);
 

 ros::Rate loop_rate(100);
  
while (ros::ok() and outkey==0)
{ 
    ros::spinOnce();                 
    loop_rate.sleep();
}








    // 创建tf的广播器
    static tf::TransformBroadcaster br;
  Eigen::Quaternionf q;
  float transx,transy,transz;
  int ifshowresult= 0;
  int ifshowfinalresult= 1;
  // Point clouds
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_kd_search (new PointCloudT);

  PointCloudT::Ptr scene (new PointCloudT);

 PointCloudT::Ptr object_aligned (new PointCloudT);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPLY(new  pcl::PointCloud<pcl::PointXYZ>);

std::cout<<"hopeeeeee:"<<argc<<";"<<argv[0]<<";"<<argv[1]<<";"<<argv[2]<<";"<<argv[3]<<";"<<argv[4]<<";"<<argv[5]<<";"<<argv[6]<<";"<<argv[7]<<";"<<argv[8]<<";"<<argv[9]<<";"<<argv[10]<<std::endl;
 

std::cout << "1..." << std::endl;
 
 
//  cout << "1..." << endl;
//filterScene(scene_input);
//filterScene(scene_rgb);
 
  

scene_input_second=scene_input;

std::cout << "4..." << std::endl; 


pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());

pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloudA(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloudB(new pcl::PointCloud<pcl::PointXYZ>());

pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.006);//###这个是中间平面的厚度；这个参数可能到时候得一个一个试试

  seg.setInputCloud (scene_input);
  seg.segment (*inliers, *coefficients);
  std::cout << "ssss1..." << std::endl; 
  // This is also where we specify the “distance threshold”, which determines how close a point must be to the model in order to be considered an inlier. 
  //for (size_t i = 0; i < inliers->indices.size (); ++i)
 //  { 
	//filtered_cloud->points.push_back(scene_input->points[inliers->indices[i]]);

//}
 

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud (scene_input_second);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*filtered_cloud);



if(ifshowresult)
{
pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer());
 
viewer->addPointCloud<pcl::PointXYZ >( scene_input_second, "samples cloudori2");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "samples cloudori2");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0,0.0, "samples cloudori2");

viewer->addPointCloud<pcl::PointXYZ >( filtered_cloud, "samples cloudori");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "samples cloudori");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,1.0, "samples cloudori");

viewer->addCoordinateSystem (0.1);
while (!viewer->wasStopped ())
{
  viewer->spinOnce (100);
boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}


  // draw the samples

int countA=0;
int countB=0;
  for (size_t i = 0; i < filtered_cloud->points.size(); ++i)
   {  
    if(coefficients->values[0]*filtered_cloud->points[i].x+coefficients->values[1]*filtered_cloud->points[i].y+coefficients->values[2]*filtered_cloud->points[i].z+coefficients->values[3]<0 )
	remaining_cloudA->points.push_back(filtered_cloud->points[i]);
else if (coefficients->values[0]*filtered_cloud->points[i].x+coefficients->values[1]*filtered_cloud->points[i].y+coefficients->values[2]*filtered_cloud->points[i].z+coefficients->values[3]>0 ) 
remaining_cloudB->points.push_back(filtered_cloud->points[i]);


if(coefficients->values[0]*filtered_cloud->points[i].x+coefficients->values[1]*filtered_cloud->points[i].y+coefficients->values[2]*filtered_cloud->points[i].z+coefficients->values[3]<-0.01 )
countA=countA+1;
else if(coefficients->values[0]*filtered_cloud->points[i].x+coefficients->values[1]*filtered_cloud->points[i].y+coefficients->values[2]*filtered_cloud->points[i].z+coefficients->values[3]> 0.01 )
countB=countB+1;
 
}
if(countA>countB)
remaining_cloud=remaining_cloudA;
else
remaining_cloud=remaining_cloudB;
 
 pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer()); 
    //viewer1->addCoordinateSystem(0.1);
//viewer2->addCoordinateSystem(0.1);
//cout << "４..." << endl;
//viewer2->removeAllPointClouds();
// cout << "５..." << endl;
//viewer2->addPointCloud<pcl::PointNormal> (scene_with_normals, single_color2, "sample cloud1");
//viewer2->addPointCloudNormals<pcl::PointNormal> (scene_with_normals, 10, 0.01, "normals");
//viewer2->addPointCloud(remaining_cloud, "remaining_cloud");
// cout << "６..." << endl;


 //viewer2->addCoordinateSystem (0.1);
 //while (!viewer2->wasStopped ())
  //{
  // viewer2->spinOnce (100);
  // boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 // }




 remaining_cloud->width = 1;
 remaining_cloud->height = remaining_cloud->points.size();

 //pcl::io::savePCDFileASCII ("/home/yuan/doc/ppf_ws/Super4PCS/PCLWrapper/build/scene_rgb_linemod.pcd", *filtered_cloud); 


 //if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/yuan/doc/ppf_ws/src/ppf/scene_rgb_segmented.pcd",*remaining_cloud)==-1)//*cloud,指针的内容是文件内容，记得标明点云类型<pcl::PointXYZ>

//if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/yuan/doc/ppf_ws/Super4PCS/PCLWrapper/build/scene_rgb_linemod.pcd",*remaining_cloud)==-1)//*cloud,指针的内容是文件内容，记得标明点云类型<pcl::PointXYZ>
  
std::cout << "8..." << std::endl; 
//std::cout<<"9999..." << scene_input->isOrganized() << ";" << remaining_cloud->isOrganized()<<std::endl;

 std::cout << "101010..."<<scene_input->points.size() << ";" <<scene_input->width  << ";"<<scene_input->height << std::endl; // << ";"<<remaining_cloud->width  << ";"<<remaining_cloud->height  << std::endl;  

 

cout << "5...normals calculation start" << endl;

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
ne.setNumberOfThreads(12);  // 手动设置线程数，否则提示错误
  ne.setInputCloud (remaining_cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);
cout << "6...normals calculation 1" << endl;
  // Compute the features
  ne.compute (*cloud_normals);
cout << "6...normals calculation 2" << endl;

pcl::concatenateFields (*remaining_cloud, *cloud_normals, *scene);


cout << "6...normals calculation over" << endl;




// Get input object and scene
 

  // Load object and scene
char *sr[11] = {NULL};//初始化 
sr[0] = (char*)"./OpenGR-PCLWrapper";
sr[1] = (char*)"../assets/scene_rgb_segmented.obj";//这个虚架的，可不考虑
sr[2] = (char*)"/home/yuan/doc/objectpose/qr_code_icp/src/untitled.obj";  
sr[3] = (char*)"-o"; 
sr[4] = (char*)"1.0"; //#######应该是匹配程度,这个值好像挺重要的！因为新时达是固定场景，所以，这个可以慢慢试，直到到一个稳定值即可，值越在０．８左右好像精度越好，但越慢，如果场景杂点少，可以试试直接用１
sr[5]= (char*)"-d"; 
sr[6] = (char*)"0.01";//应该是匹配精度
sr[7] = (char*)"-t";
sr[8] = (char*)"1000";//应该是匹配上限时间？
sr[9] = (char*)"-n";
sr[10] = (char*)"120";//#######越小速度越快,但精度会下降！！可能会导致后面ＩＣＰ的时候点不全
cout <<"sssssssssssss"<< sr[2] << endl;

  pcl::console::print_highlight ("Loading point clouds　A...\n");
  if (pcl::io::loadOBJFile<PointNT> (sr[2], *object) < 0  )
  {
    pcl::console::print_error ("Error loading object/scene file!\n");
    return (-1);
  }

  //pcl::console::print_highlight ("Loading point clouds B...\n");
//if(pcl::io::loadOBJFile<PointNT>(argv[1],*scene)<0)
//{
 //   pcl::console::print_error ("Error loading object/scene file!\n");
 //   return (-1);
 // }


  if(int c = Demo::getArgs(11, sr) != 0)
    { 
      exit(std::max(c,0));
    }

  pcl::Super4PCS<PointNT,PointNT> align;
  Demo::setOptionsFromArgs(align.options_);

//////////////////////////////////////////////////////////// Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;

  pcl::VoxelGrid<pcl::PointXYZ> grid_search;






  const float leaf = 0.01f; //###这个决定了super4pcs匹配的分别率
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);





//////////////////////////////////////////////////////////// 
 // grid.setInputCloud (scene);
  //grid.filter (*scene);

  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  align.setInputSource (object);
  align.setInputTarget (scene);

  {
    pcl::ScopeTime t("Alignment");
 //std::cout<<"start to align..." << std::endl;
    align.align (*object_aligned);
// std::cout<<"end to align..." << std::endl;
  }
 

  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");

    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment - Super4PCS");
   // visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
////////////////////////////////////////////////////////////
//////ICP algorithms:




  int iterations = 500; 
  Eigen::Matrix4d transformation_matrix;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;


pcl::copyPointCloud( *scene, *scene_xyz);
pcl::copyPointCloud( *object, *object_xyz);
pcl::copyPointCloud( *object_aligned, *object_aligned_xyz);
pcl::copyPointCloud( *object_kd_search, *object_kd_search_xyz);


if(ifshowresult)
{


 viewer2->addCoordinateSystem(0.1); 
 viewer2->removeAllPointClouds(); 
 
 viewer2->addPointCloud(scene_xyz, "scene_xyz");
 viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "scene_xyz");
 viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,1.0, "scene_xyz");
viewer2->addPointCloud(object_xyz, "object_xyz");
viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "object_xyz");
 viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "object_xyz");


viewer2->addPointCloud(object_aligned_xyz, "object_aligned_xyz");
viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "object_aligned_xyz");
 viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "object_aligned_xyz");

 
// cout << "６..." << endl;

 
  while (!viewer2->wasStopped ())
  {
   viewer2->spinOnce (100);
   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }


}














const float leaf_kd_search = 0.01f;//###这个，就是搜场景点云中，离匹配物体临近的点云时，物体的采样率
grid_search.setLeafSize (leaf_kd_search, leaf_kd_search, leaf_kd_search);
grid_search.setInputCloud (object_aligned_xyz);
grid_search.filter (*object_aligned_sparse_xyz);


pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PointCloud_color_object(scene_input, 255.0, 255.0, 255.0);

visu.addPointCloud<pcl::PointXYZ>(scene_input, PointCloud_color_object, "object_aligned_sparse_xyz");
//###############################################################
//如果要看过滤过的点云，则取消注释下面的部分
//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PointCloud_color_object(object_aligned_xyz, 255.0, 255.0, 255.0);

//visu.addPointCloud<pcl::PointXYZ>(object_aligned_xyz, PointCloud_color_object, "object_aligned_sparse_xyz");
//###############################################################

//if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/yuan/doc/ppf_ws/src/ppf/scene_rgb_segmented.pcd",*scene_xyz)==-1)//*cloud,指针的内容是文件内容，记得标明点云类型<pcl::PointXYZ>
 
//////////////////////////////////////////////////////////////
//////KD-TREE searching
 


  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (object_aligned_xyz);

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius =  0.02;//###这个，就是搜场景点云中，离匹配物体临近的点云时，搜索半径，　　半径越大，越可能有杂点


  for (size_t i = 0; i < scene_xyz->points.size(); ++i)
   {  
     pcl::PointXYZ searchPoint;

  searchPoint.x = scene_xyz->points[i].x;
  searchPoint.y = scene_xyz->points[i].y;
  searchPoint.z = scene_xyz->points[i].z;
 

  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  {
    scene_xyz_kdtree->points.push_back(scene_xyz->points[i]);
  }

}
//////////////////////////////////////////////////////////////

if(ifshowresult)
 {
 viewer2->addCoordinateSystem(0.1);
 cout << "４..." << endl;
 viewer2->removeAllPointClouds();
 cout << "５..." << endl;
 
 viewer2->addPointCloud(scene_xyz_kdtree, "scene_xyz_kdtree");
 viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "scene_xyz_kdtree");
 viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0,1.0, "scene_xyz_kdtree");


viewer2->addPointCloud(object_aligned_xyz, "object_aligned_xyz");
viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "object_aligned_xyz");
 viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "object_aligned_xyz");



 viewer2->addPointCloud(scene_xyz, "scene_xyz");
// cout << "６..." << endl;

 
  while (!viewer2->wasStopped ())
  {
   viewer2->spinOnce (100);
   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
 
  icp.setMaximumIterations (iterations);
  icp.setInputSource (  scene_xyz_kdtree);//理论上应该是环境点云，但被虑到只剩鞋子了
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PointCloud_color_ori(scene_xyz_kdtree, 0.0, 255.0, 0.0);

//visu.addPointCloud<pcl::PointXYZ>(scene_xyz_kdtree, PointCloud_color_ori, "object_aligned_ori");
//visu.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "object_aligned_ori");


  icp.setInputTarget (object_aligned_xyz);//鞋子模型的点云
  icp.align (*scene_xyz_kdtree);
 
   Eigen::Matrix4f transformation2 = icp.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation2 (0,0), transformation2 (0,1), transformation2 (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation2 (1,0), transformation2 (1,1), transformation2 (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation2 (2,0), transformation2 (2,1), transformation2 (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation2 (0,3), transformation2 (1,3), transformation2 (2,3));
    pcl::console::print_info ("\n");



   Eigen::Matrix4f transformationfinal ;

Eigen::Matrix4f unittranform = Eigen::Matrix4f::Identity();

transformationfinal=transformation2.inverse()*transformation ;//因为之前ＩＣＰ写成了环境匹配鞋子，而不是鞋子匹配环境，所以要逆
 
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformationfinal (0,0), transformationfinal (0,1), transformationfinal (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformationfinal (1,0), transformationfinal (1,1), transformationfinal (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformationfinal (2,0), transformationfinal (2,1), transformationfinal (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformationfinal (0,3), transformationfinal (1,3), transformationfinal (2,3));
    pcl::console::print_info ("\n");
transx=transformationfinal (0,3);
transy=transformationfinal (1,3);
transz=transformationfinal (2,3);

 // icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
 // std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;
   
 pcl::transformPointCloud (*object_xyz, *final_model,transformationfinal); 

Eigen::Matrix3f rotation_final;
rotation_final << transformationfinal (0,0), transformationfinal (0,1), transformationfinal (0,2),
     transformationfinal (1,0), transformationfinal (1,1), transformationfinal (1,2),
     transformationfinal (2,0), transformationfinal (2,1), transformationfinal (2,2);
 
q= rotation_final;
if(ifshowfinalresult)
{visu.addCoordinateSystem(0.2);
////////////////////////////////////////////////////////////////////////
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PointCloud_color(final_model, 0.0, 0.0, 255.0);

visu.addPointCloud<pcl::PointXYZ>(final_model, PointCloud_color, "object_aligned");
visu.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "object_aligned");
    visu.spin ();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    return (-1);
  }}
 while(ros::ok()){
        // 初始化tf数据
        tf::Transform transform; 
        transform.setOrigin( tf::Vector3(transx, transy, transz) );
         transform.setRotation( tf::Quaternion(q.x(),q.y(),q.z(),q.w()) );

        // 广播base_link与base_laser坐标系之间的tf数据
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "object"));
    }
  return (0);
}
