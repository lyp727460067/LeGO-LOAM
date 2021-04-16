#include "pcl/io/ply_io.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include  "ros/ros.h"
#include "mapOptmization.cpp"
#include "pcl/point_cloud.h"
#include <pcl/point_types.h>
#include "gtsam/slam/PriorFactor.h"
#include <pcl/registration/ndt.h>
#include <pclomp/ndt_omp.h>
#include <pcl/registration/registration.h>
bool new_trajecto_flag  = true;
class MapManager
{
    public:
     typedef pcl::PointXYZ PointType ;
     bool LoadMapInFile(std::string file_name) {
       pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
       std::cout << "start_view" << std::endl;
       if (pcl::io::loadPCDFile<PointType>(file_name, *cloud) == -1) {
         PCL_ERROR("Couldn't read file %c \n", file_name.c_str());
         return false;
       }
       map_data_ = cloud;
       return true;
     }
     
     Eigen::Matrix4f GetPoseInMap(
         const pcl::PointCloud<PointType>::Ptr& points) {
       icp.setMaxCorrespondenceDistance(100);
       icp.setMaximumIterations(100);
       icp.setTransformationEpsilon(1e-6);
       icp.setEuclideanFitnessEpsilon(1e-6);
       icp.setRANSACIterations(0);
       icp.setInputSource(points);
       icp.setInputTarget(map_data_);

       pcl::PointCloud<PointType>::Ptr unused_result(
           new pcl::PointCloud<PointType>());
       icp.align(*unused_result);
       if (icp.hasConverged() == false || icp.getFitnessScore() > 0.85) {
         std::cout << "not found constrait" << std::endl;
         return Eigen::Matrix4f();
       }
       return icp.getFinalTransformation();
     }
    private:
    pcl::IterativeClosestPoint<PointType,PointType> icp;
    pcl::PointCloud<PointType>::Ptr map_data_;
};






class LoctionMapOptimization :public mapOptimization{
    public:
     void performLoopClosure(){
       if(new_trajecto_flag){
      int latestFrameIDLoopCloure1 = cloudKeyPoses3D->points.size() - 1;
      pcl::IterativeClosestPoint<PointType, PointType> icp;
      pcl::PointCloud<PointType>::Ptr unused_result(
          new pcl::PointCloud<PointType>());
      //ndt->setInputSource(cornerCloudKeyFrames[latestFrameIDLoopCloure1]);
      auto pose  =  VectorToAffined(std::vector<float>(std::begin(transformAftMapped), std::end(transformAftMapped)));
      Eigen::Matrix4f expect_pose = pose.matrix().template  cast<float>();//Eigen::Matrix4f::Identity();
      // ndt->align(*unused_result);
      // if (ndt->hasConverged()) {

      //    double score = ndt->getFitnessScore();
      //   // if(score <1){
      //          expect_pose  = ndt->getFinalTransformation();
        std::cout<<"ndt found"<< expect_pose<< std::endl;    
      //  //  }
      //   // return true;
      // }
      std::cout<<"start icp"<<std::endl;
       icp.setInputTarget(map_data_);    
       icp.setInputSource(cornerCloudKeyFrames[latestFrameIDLoopCloure1]);
      //  pcl::PointCloud<PointType>::Ptr unused_result(
      //      new pcl::PointCloud<PointType>());
       icp.align(*unused_result,expect_pose);
       if (icp.hasConverged() == false ||
           icp.getFitnessScore() > historyKeyframeFitnessScore) {
         std::cout << "LoctionMapOptimization  not found constrait"
                   << std::endl;
        
       } else {
         std::cout << "LoctionMapOptimization   found constrait" << std::endl;
         float x, y, z, roll, pitch, yaw;
         Eigen::Affine3f correctionCameraFrame;
         correctionCameraFrame =
             icp.getFinalTransformation();  // get transformation in camera
                                            // frame (because points are in
                                            // camera frame)
         pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll,
                                           pitch, yaw);

         Eigen::Affine3f correctionLidarFrame =
             pcl::getTransformation(z, x, y, yaw, roll, pitch);

         // transform from world origin to corrected pose
         Eigen::Affine3f tCorrect = correctionLidarFrame;
         pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

         gtsam::Pose3 pose_prior =
             Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));

         gtsam::Vector Vector6(6);
         float noiseScore = icp.getFitnessScore();
         Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore,
             noiseScore;
         constraintNoise = noiseModel::Diagonal::Variances(Vector6);

         std::lock_guard<std::mutex> lock(mtx);
         gtSAMgraph.add(PriorFactor<Pose3>(latestFrameIDLoopCloure1, pose_prior,
                                           constraintNoise));
       }
       }

       mapOptimization::performLoopClosure();
     }



      bool LoadMapInFile(std::string file_name) {
       pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
       std::cout << "start_view" << std::endl;
       if (pcl::io::loadPCDFile<PointType>(file_name, *cloud) == -1) {
         PCL_ERROR("Couldn't read file %c \n", file_name.c_str());
         return false;
       }
         map_data_ = cloud;



      // pcl::NormalDistributionsTransform<PointType, PointType> ndt;
      // ndt.setInputSource(surfCloudKeyFrames[latestFrameIDLoopCloure1]);
      // ndt.setInputTarget(map_data_);
      // ndt.setTransformationEpsilon(1);
      //  ndt.setStepSize(0.5);
      //  ndt.setResolution(5.0);
      //  ndt.setMaximumIterations(30);

         pclomp::NormalDistributionsTransform<PointType, PointType>::Ptr ndt1(
             new pclomp::NormalDistributionsTransform<PointType, PointType>());
        
         icp.setMaxCorrespondenceDistance(100);
         icp.setMaximumIterations(100);
         icp.setTransformationEpsilon(1e-5);
         icp.setEuclideanFitnessEpsilon(1e-5);
         icp.setRANSACIterations(0);
         icp.setInputTarget(map_data_);        
        
         ndt1->setNeighborhoodSearchMethod(pclomp::DIRECT7);
         ndt1->setTransformationEpsilon(0.1);
         ndt1->setResolution(0.1);
         ndt1->setNeighborhoodSearchMethod(pclomp::KDTREE);
         ndt = ndt1;
         ndt->setInputTarget(map_data_);

         std::cout<<"read success"<<std::endl;
         return true;
     }

      pcl::PointCloud<PointType>::Ptr map_data_;
     private :

      pcl::Registration<PointType, PointType>::Ptr ndt;
      pcl::IterativeClosestPoint<PointType, PointType> icp;

};



DEFINE_string(ply_filename,"","file of ply to draw in rviz");
DEFINE_string(pose_filename,"","file pose ");

int main(int argc,char **argv)
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc,&argv,true);
    ros::init(argc, argv, "lidar_conva");
     LoctionMapOptimization MO;   
    if(FLAGS_ply_filename.empty() ){
        google::ShowUsageWithFlagsRestrict(argv[0],"ply_view");
        new_trajecto_flag  =false ;
        //return EXIT_FAILURE;
    }else {
        if(!MO.LoadMapInFile(FLAGS_ply_filename)){
          new_trajecto_flag = false;
        }
    }
    ROS_INFO("\033[1;32m---->\033[0m Map Optimization Started.");
    std::thread loopthread(&LoctionMapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&LoctionMapOptimization::visualizeGlobalMapThread, &MO);
  ros::NodeHandle nh; 
  ros::Publisher cloud_pub =nh.advertise<sensor_msgs::PointCloud2>("map_cloud",1);
    ros::Rate rate(200);
    int cout  = 0;
    while (ros::ok())
    {
        ros::spinOnce();

        MO.run();

        rate.sleep();
        sensor_msgs::PointCloud2 point_cloud2;
        if (new_trajecto_flag) {
          if(cout++>=2000){
          cout  = 0;
          pcl::toROSMsg(*MO.map_data_, point_cloud2);
          point_cloud2.header.stamp = ros::Time::now();
          point_cloud2.header.frame_id = "camera_init";
          cloud_pub.publish(point_cloud2);
        }
        }
    }
    loopthread.join();
    visualizeMapThread.join();


    ros::start();

    ros::shutdown();

}