
#include "gtest/gtest.h"
#include "gflags/gflags.h"
#include "mapOptmization.cpp"
#include "ros/ros.h"

namespace{
class  MapOptimizationTest :public testing::Test
{
  protected:
  
   virtual void SetUp()  {
     
   }


};


}

TEST_F(MapOptimizationTest,TestTransformAssociateToMap){




}

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"MapOptimizationTest");
  mapOptimization map_optimization_;
  float data[6] = {0, 0, 0, 0, 1, 2};

  for (int i = 0; i < 100; i++) {
    for (int j = 0; j < 100; j++) {
      for (int k = 0; k < 100; k++) {
        data[0] = i + 0.1;
        data[1] = j + 0.2;
        data[2] = k + 0.3;  

        map_optimization_.SetTransformTobeMapped(data);
        map_optimization_.transformAssociateToMap();
        auto tran1 = map_optimization_.GettransformAssociateToMap();
        map_optimization_.SetTransformTobeMapped(data);
        auto tran = map_optimization_.transformAssociateToMap(0);
        for(int i  = 0;i<6;i++){
           EXPECT_NEAR(tran1[i], tran[i],1e-3);
        }
       
      }
    }
  }
}