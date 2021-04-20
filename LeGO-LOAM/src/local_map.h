#ifndef LOCAL_MAP_
#define LOCAL_MAP_
#include "Eigen/Core"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
struct Node
{
  struct NodeData{
    Eigen::MatrixXd surf_feat;
    Eigen::MatrixXd edge_feat;
    Eigen::MatrixXd groud_feat;
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
  };
  NodeData data;
  Eigen::Matrix4d local_pose;

};




class LocalMap {
 public:
 struct LocalResult
 {


 };
  void Insert(const Node::NodeData & node)
  {

  }
 Node::NodeData  GetLocalMap(){

 }
  Eigen::Matrix4f match(const Eigen::Matrix4f & init_pose,std::shared_ptr<Node> node)
  {


  }
 private:
  std::vector<std::shared_ptr<Node>> nodes_

};

#endif