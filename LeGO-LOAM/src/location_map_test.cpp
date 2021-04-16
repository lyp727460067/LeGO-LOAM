
#include "glog/logging.h" 
#include "gflags/gflags.h"
#include "gtest/gtest.h"
#include "location_map.cpp"
#include <string>
using namespace std;
namespace
{

  string file_name = "/home/lyp/surfaceMap_orig.pcd";
  
  class MapManagerTest:public testing::Test
  {
    public:
    void SetUp() {
      map_manager_.LoadMapInFile(file_name);
    }  

    protected :
    MapManager map_manager_; 

  };



}
TEST_F(MapManagerTest,CheckPose)
{





}
