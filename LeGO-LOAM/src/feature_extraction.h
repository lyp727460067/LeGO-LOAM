#ifndef _FEATURE_EXTRACTION_H
#define _FEATURE_EXTRACTION_H
#include "Eigen/Core"


class FeatureSurf
{


};

class FeatureEdge
{


};
class FeatureGound
{


};


template<typename Data>
class FeatureData
{
  public:
  Data data_;
};


class FeatureExtraction
{
  public:

  template<typename Data>
  Eigen::MatrixXd Extraction();


  private:



};







#endif