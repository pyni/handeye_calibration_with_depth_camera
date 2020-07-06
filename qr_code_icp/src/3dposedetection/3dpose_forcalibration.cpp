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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include<iostream>
#include<cmath> 
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sys/time.h>
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
pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointclouds (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz_kdtree (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr final_model (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr object_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr ori_object_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned_sparse_xyz (new pcl::PointCloud<pcl::PointXYZ> ());

pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned_xyz_icped (new pcl::PointCloud<pcl::PointXYZ> ()); 
 
pcl::PointCloud<pcl::PointXYZ>::Ptr object_kd_search_xyz(new pcl::PointCloud<pcl::PointXYZ> ());

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_rgb (new pcl::PointCloud<pcl::PointXYZRGBA> ());
pcl::PointCloud<pcl::PointNormal>::Ptr scene_with_normals (new pcl::PointCloud<pcl::PointNormal> ()); 

using namespace gr;
using namespace cv;
using namespace std;

 int outkey=0;
 int outkey_time=0;
void timerCallback(const ros::TimerEvent& event)
{
outkey_time=1;

}
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2); 
    pcl::fromPCLPointCloud2(pcl_pc2,*scene_input);
    outkey=1; 
    //pcl::io::savePCDFileASCII("output_pcd.pcd", *scene_input);
}

float findmin(vector<float> vec) {
    float min =  999;
    for (auto v : vec) {
        if (min > v) min = v;
    }
    return min;
}

int getPositionOfmin(vector<float> vec, float min) {
    auto distance = find(vec.begin(), vec.end(), min);
    return distance - vec.begin();
} 

int
main (int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "test_real");

    // 订阅base_link的位置话题
    ros::NodeHandle node;


 pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer()); 
 pcl::visualization::PCLVisualizer visu("Alignment - Super4PCS");

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = node.subscribe ("/camera/depth/color/points", 1, cloud_cb);
 

 ros::Rate loop_rate(100);
  
while (ros::ok() and outkey==0)
{ 
    ros::spinOnce();                 
    loop_rate.sleep();
}

tf::TransformListener listener;
tf::StampedTransform transform_aru_tf22,transform_aru_tf44,transform_aru_tf888,transform_aru_tf963,transform_aru_tf;
float distance22,distance888,distance44,distance963,finaldistance;
int key22,key888,key44,key963;
std::vector<tf::StampedTransform> transform_aru_tf_vector;
std::vector<float> distances;
std::vector<int> multiples_vec;
std::vector<std::string> strVec;
key22=0;
key888=0;
key44=0;
key963=0; 
struct timespec tsp;
int deltatime,timestart;
clock_gettime(CLOCK_REALTIME,&tsp); 
struct tm *tmv = gmtime(&tsp.tv_sec);
timestart=tmv->tm_sec;  

  while (node.ok() and  (key22==0 or key888==0 or key44==0 or key963==0) and (deltatime<5) ){
    if (key22==0)
   { try{ 
      listener.lookupTransform("/aruco_marker_frame22", "/camera_depth_optical_frame",  
                               ros::Time(0), transform_aru_tf22);  
      key22=1;
      distance22=transform_aru_tf22.getOrigin().x()*transform_aru_tf22.getOrigin().x()+transform_aru_tf22.getOrigin().y()*transform_aru_tf22.getOrigin().y()+transform_aru_tf22.getOrigin().z()*transform_aru_tf22.getOrigin().z();
      transform_aru_tf_vector.push_back(transform_aru_tf22);
      distances.push_back(distance22);
      strVec.push_back("aruco_marker_frame22");
      multiples_vec.push_back(2);
    }
    catch (tf::TransformException ex){
      ; 
    }}



    if (key44==0)
   { try{ 
      listener.lookupTransform("/aruco_marker_frame44", "/camera_depth_optical_frame",  
                               ros::Time(0), transform_aru_tf44);  
      key44=1; 
      distance44=transform_aru_tf44.getOrigin().x()*transform_aru_tf44.getOrigin().x()+transform_aru_tf44.getOrigin().y()*transform_aru_tf44.getOrigin().y()+transform_aru_tf44.getOrigin().z()*transform_aru_tf44.getOrigin().z();
      transform_aru_tf_vector.push_back(transform_aru_tf44);
      distances.push_back(distance44);
      strVec.push_back("aruco_marker_frame44");
      multiples_vec.push_back(1);
    }
    catch (tf::TransformException ex){
      ; 
    }}

    if (key888==0)
   { try{ 
      listener.lookupTransform("/aruco_marker_frame888", "/camera_depth_optical_frame",  
                               ros::Time(0), transform_aru_tf888);  
      key888=1;
      distance888=transform_aru_tf888.getOrigin().x()*transform_aru_tf888.getOrigin().x()+transform_aru_tf888.getOrigin().y()*transform_aru_tf888.getOrigin().y()+transform_aru_tf888.getOrigin().z()*transform_aru_tf888.getOrigin().z();
      transform_aru_tf_vector.push_back(transform_aru_tf888);          
      distances.push_back(distance888);        
      strVec.push_back("aruco_marker_frame888");   
      multiples_vec.push_back(0);
    }
    catch (tf::TransformException ex){
      ; 
    }}

        if (key963==0)
   { try{ 
      listener.lookupTransform("/aruco_marker_frame963", "/camera_depth_optical_frame",  
                               ros::Time(0), transform_aru_tf963);  
                               key963=1;
      distance963=transform_aru_tf963.getOrigin().x()*transform_aru_tf963.getOrigin().x()+transform_aru_tf963.getOrigin().y()*transform_aru_tf963.getOrigin().y()+transform_aru_tf963.getOrigin().z()*transform_aru_tf963.getOrigin().z();
      transform_aru_tf_vector.push_back(transform_aru_tf963);     
      distances.push_back(distance963);      
              strVec.push_back("aruco_marker_frame963")  ;   
                    multiples_vec.push_back(3)       ;                  
    }
    catch (tf::TransformException ex){
      ; 
    }}
    
struct timespec tsp2;
clock_gettime(CLOCK_REALTIME,&tsp2); 
struct tm *tmv2 = gmtime(&tsp2.tv_sec); 
deltatime=tmv2->tm_sec-timestart;
   //cout<<"deltatime:"<<deltatime<<" :"<<tmv2->tm_sec<<" :"<<timestart<<endl; 
    }






ros::Time timeend = ros::Time::now();
pcl::console::print_info ("size2,%f \n",timeend );

    float minNumber = findmin(distances);
    int position = getPositionOfmin(distances, minNumber);


transform_aru_tf=transform_aru_tf_vector[position];
int multiples =multiples_vec[position];
pcl::console::print_info ("size,%d  ,sss %s  \n",strVec.size(),strVec[position].c_str());

 
pcl::console::print_info ("multiples,%d   \n",multiples);
////////////////////////
 

    // 创建tf的广播器
    static tf::TransformBroadcaster br;
  Eigen::Quaternionf q_r_final;
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
 std::vector<int> indices;
pcl::removeNaNFromPointCloud(*scene_input,*scene_input, indices);


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
  seg.setDistanceThreshold (0.015);//###这个是中间平面的厚度；这个参数可能到时候得一个一个试试

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

pcl::console::print_info ("filtered_cloud size, %d  \n",filtered_cloud->points.size());

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

if(filtered_cloud->points[i].x*filtered_cloud->points[i].x+filtered_cloud->points[i].y*filtered_cloud->points[i].y+filtered_cloud->points[i].z*filtered_cloud->points[i].z==0.0 )
continue;


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
  
 

if(ifshowresult)
{
pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer());
 viewer->removeAllPointClouds();
viewer->addPointCloud<pcl::PointXYZ >( scene_input_second, "samples cloudori2");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "samples cloudori2");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0,0.0, "samples cloudori2");

viewer->addPointCloud<pcl::PointXYZ >( remaining_cloud, "samples cloudori");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "samples cloudori");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,1.0, "samples cloudori");
 
while (!viewer->wasStopped ())
{
  viewer->spinOnce (100);
boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}







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
 
 


pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PointCloud_color_object(remaining_cloud, 255.0, 255.0, 255.0);

visu.addPointCloud<pcl::PointXYZ>(remaining_cloud, PointCloud_color_object, "object_aligned_sparse_xyz");

// cout << "5...normals calculation start" << endl;

//   // Create the normal estimation class, and pass the input dataset to it
//   pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
// ne.setNumberOfThreads(12);  // 手动设置线程数，否则提示错误
//   ne.setInputCloud (remaining_cloud);

//   // Create an empty kdtree representation, and pass it to the normal estimation object.
//   // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//   ne.setSearchMethod (tree);

//   // Output datasets
//   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

//   // Use all neighbors in a sphere of radius 3cm
//   ne.setRadiusSearch (0.03);
// cout << "6...normals calculation 1" << endl;
//   // Compute the features
//   ne.compute (*cloud_normals);
// cout << "6...normals calculation 2" << endl;

// pcl::concatenateFields (*remaining_cloud, *cloud_normals, *scene);


// cout << "6...normals calculation over" << endl;

// // Get input object and scene
 


  // Load object and scene
char *sr[11] = {NULL};//初始化 
sr[0] = (char*)"./OpenGR-PCLWrapper";
sr[1] = (char*)"../assets/scene_rgb_segmented.obj";//这个虚架的，可不考虑
sr[2] = (char*)"/home/yuan/doc/objectpose/qr_code_icp/src/calibblockthird.obj";  


pcl::console::print_highlight ("Loading point clouds　A...\n");
if (pcl::io::loadOBJFile<PointNT> (sr[2], *object) < 0  )
{
pcl::console::print_error ("Error loading object/scene file!\n");
return (-1);
}
 pcl::console::print_highlight ("Loading success...\n");
 

 
 

  if (1)
  {
    // Print results
    printf ("\n");
 
 
////////////////////////////////////////////////////////////
//////ICP algorithms:




  int iterations = 500 ; 
  Eigen::Matrix4d transformation_matrix;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
 
pcl::copyPointCloud( *object, *ori_object_xyz); 
 




pcl::VoxelGrid<pcl::PointXYZ> grid_search;


const float leaf_kd_search = 0.01f;//###这个，就是搜场景点云中，离匹配物体临近的点云时，物体的采样率
grid_search.setLeafSize (leaf_kd_search, leaf_kd_search, leaf_kd_search);
grid_search.setInputCloud (ori_object_xyz);
grid_search.filter (*object_aligned_sparse_xyz);

//###############################################################
//如果要看过滤过的点云，则取消注释下面的部分
//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PointCloud_color_object(object_aligned_xyz, 255.0, 255.0, 255.0);

//visu.addPointCloud<pcl::PointXYZ>(object_aligned_xyz, PointCloud_color_object, "object_aligned_sparse_xyz");
//###############################################################

 



Eigen::Matrix4f transform_rotation_humanpredefined = Eigen::Matrix4f::Identity();  //这个应该是二维码相对于物体坐标系的变换,先转动
Eigen::AngleAxisf rollAngle(0, Eigen::Vector3f::UnitX());
Eigen::AngleAxisf pitchAngle(0 , Eigen::Vector3f::UnitY());
Eigen::AngleAxisf yawAngle(0.78539816339745, Eigen::Vector3f::UnitZ()); 
Eigen::Quaternionf  q = yawAngle * pitchAngle * rollAngle; 
Eigen::Matrix3f mat3_humanpredefined = q.matrix();  
transform_rotation_humanpredefined.block(0,0,3,3) = mat3_humanpredefined;


Eigen::Matrix4f transform_trans_humanpredefined = Eigen::Matrix4f::Identity(); 
transform_trans_humanpredefined(0,3)= -0.065;//应该是负的   这个应该是二维码相对于物体坐标系的变换,再移动
transform_trans_humanpredefined(1,3)=  0.02;



Eigen::Matrix4f transform_rotation_calibration_final  = Eigen::Matrix4f::Identity();  // 就不同二维码（贴了四周，各四个），需要再进行一次绕y的旋转
Eigen::AngleAxisf rollAngle_final(0, Eigen::Vector3f::UnitX());
Eigen::AngleAxisf pitchAngle_final(1.5707963267949*multiples, Eigen::Vector3f::UnitY());//1.5707963267949*2
Eigen::AngleAxisf yawAngle_final(0 , Eigen::Vector3f::UnitZ()); 
Eigen::Quaternionf  q_final = yawAngle_final * pitchAngle_final * rollAngle_final; 
Eigen::Matrix3f mat3_humanpredefined_final = q_final.matrix();  
transform_rotation_calibration_final.block(0,0,3,3) = mat3_humanpredefined_final;


Eigen::Matrix3f mat3 = Eigen::Quaternionf(transform_aru_tf.getRotation().w(), transform_aru_tf.getRotation().x(),transform_aru_tf.getRotation().y(), transform_aru_tf.getRotation().z()).toRotationMatrix(); 
Eigen::Matrix4f transform_aru = Eigen::Matrix4f::Identity(); 
///////////////////////
transform_aru.block(0,0,3,3) = mat3;
transform_aru (0,3)=transform_aru_tf.getOrigin().x() ;
transform_aru (1,3)=transform_aru_tf.getOrigin().y();
transform_aru (2,3)=transform_aru_tf.getOrigin().z();
transform_aru=transform_rotation_calibration_final*transform_trans_humanpredefined*transform_rotation_humanpredefined*transform_aru;

Eigen::Matrix4f transform_aru_inv =transform_aru.inverse(); 
pcl::transformPointCloud (*ori_object_xyz, *object_xyz,transform_aru_inv);  
Eigen::Affine3f listeningresult(transform_aru_inv); 
visu.addCoordinateSystem(0.2,listeningresult);   





pcl::io::savePCDFileASCII ("/home/yuan/doc/objectpose/qr_code_icp/src/test_pcd.pcd", *remaining_cloud);












//////////////////////////////////////////////////////////////
 
  icp.setMaximumIterations (iterations);
  icp.setInputSource (  remaining_cloud);//理论上应该是环境点云 
 
//visu.addPointCloud<pcl::PointXYZ>(scene_xyz_kdtree, PointCloud_color_ori, "object_aligned_ori");
//visu.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "object_aligned_ori");
 
  icp.setInputTarget (object_xyz);//鞋子模型的点云
  pcl::console::print_highlight ("Start Loading ...\n");
  icp.align (*remaining_cloud);//理论上应该是环境点云 
  pcl::console::print_highlight ("Stop Loading...\n");
   
   Eigen::Matrix4f transformation2 = icp.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation2 (0,0), transformation2 (0,1), transformation2 (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation2 (1,0), transformation2 (1,1), transformation2 (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation2 (2,0), transformation2 (2,1), transformation2 (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation2 (0,3), transformation2 (1,3), transformation2 (2,3));
    pcl::console::print_info ("\n");

    pcl::console::print_info ("remaining_cloud size, %d , %d \n",remaining_cloud->points.size(),object_xyz->points.size());

   Eigen::Matrix4f transformationfinal ;

Eigen::Matrix4f unittranform = Eigen::Matrix4f::Identity();

transformationfinal=transformation2.inverse()*transform_aru_inv;//因为之前ＩＣＰ写成了环境匹配鞋子，而不是鞋子匹配环境，所以要逆
 
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


 
Eigen::Matrix3f rotation_final;
rotation_final << transformationfinal (0,0), transformationfinal (0,1), transformationfinal (0,2),
     transformationfinal (1,0), transformationfinal (1,1), transformationfinal (1,2),
     transformationfinal (2,0), transformationfinal (2,1), transformationfinal (2,2);
 
q_r_final= rotation_final;




 //originallllllllllll

pcl::transformPointCloud (*ori_object_xyz, *final_model,transformationfinal); 

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
 cout << "sending transforms" << endl;
pcl::console::print_info ("sending transformsA");
 while(ros::ok()){
 cout << "1" << endl;
//pcl::console::print_info ("2");
// 初始化tf数据
tf::Transform transform; 
transform.setOrigin( tf::Vector3(transx, transy, transz) );
transform.setRotation( tf::Quaternion(q_r_final.x(),q_r_final.y(),q_r_final.z(),q_r_final.w()) );

// 广播base_link与base_laser坐标系之间的tf数据
//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "object"));
br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "tracking_origin", "tracking_marker"));//tracking_origin其对应就是camera_depth_optical_frame
    }
  return (0);
}
