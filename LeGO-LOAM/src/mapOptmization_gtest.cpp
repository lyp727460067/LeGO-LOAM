
#include "gtest/gtest.h"
#include "gflags/gflags.h"
#include "mapOptmization.cpp"
#include "ros/ros.h"

namespace{
class  MapOptimizationTest :public testing::Test
{
  protected:



};


}

TEST_F(MapOptimizationTest,TestTransformAssociateToMap){

}

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"MapOptimizationTest");
  mapOptimization map_optimization_;
  float data[6] = {0, 0, 0, 0, 1, 2};

  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      for (int k = 0; k < 10; k++) {
        data[0] = i + 0.1;
        data[1] = j + 0.2;
        data[2] = k + 0.3;

        map_optimization_.SetTransformTobeMapped(data);
        map_optimization_.transformAssociateToMap();
        auto tran1 = map_optimization_.GettransformAssociateToMap();
        map_optimization_.SetTransformTobeMapped(data);
        auto tran = map_optimization_.transformAssociateToMap(0);
        EXPECT_EQ(tran1, tran);
      }
    }
  }
}