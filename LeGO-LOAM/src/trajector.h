#ifndef TRAJECTOR_H
#define TRAJECTOR_H
#include  "feature_extraction.h"
#include <memory>
#include "scan_match.h"
#include "lidar_mapping.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <vector>
#include "local_map.h"





class Trajector {
 public:
  struct ScanResult
  {


  };
   ScanResult ScanMatch() {
    std::shared_ptr<Node> mutable_node =  std::make_shared<Node>();
    mutable_node->data.surf_feat = feature_extraction_->Extraction<FeatureSurf>();
    mutable_node->data.edge_feat = feature_extraction_->Extraction<FeatureEdge>();
    mutable_node->data.groud_feat = feature_extraction_->Extraction<FeatureGound>();
    Eigen::Matrix4f initial_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f predict_pose = ScanFeat(initial_pose,*mutable_node, pre_node_);
    if(IsKeyFrame){
      nodes_.push_back(mutable_node);
      predict_pose = local_match_->match(predict_pose,mutable_node);
      
    } 
    mutable_node->local_pose = predict_pose;
    pre_node_ =  *mutable_node;
  }

 private:
  Eigen::Matrix4d ScanFeat(const Eigen::Matrix4f init_pose, const Node cur_node,
                           const Node &last_node) {


  }
  bool IsKeyFrame  = false;
  Node pre_node_;
  std::vector<std::shared_ptr<Node>> nodes_;
  std::unique_ptr<LocalMap> local_match_;
  std::unique_ptr<FeatureExtraction> feature_extraction_;
};

#endif