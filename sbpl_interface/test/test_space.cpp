#include <gtest/gtest.h>
#include <sbpl_interface/environment_chain3d_moveit.h>

class DiscreteEnvTestWrapper : public sbpl_interface::EnvironmentChain3DMoveIt
{
public:
  bool testContinuousXYZtoDiscreteXYZ(const double X, const double Y, const double Z,
                                      int& x, int& y, int& z)
  {
    return continuousXYZtoDiscreteXYZ(X, Y, Z, x, y, z);
  }
};

TEST(testSpace, testcontinuousXYZtoDiscreteXYZ)
{
  DiscreteEnvTestWrapper env;

  int x, y, z;
  EXPECT_TRUE(env.testContinuousXYZtoDiscreteXYZ(0.0, 0.0, 0.0, x, y, z));
  EXPECT_EQ(25, x);
  EXPECT_EQ(50, y);
  EXPECT_EQ(0, z);

  EXPECT_TRUE(env.testContinuousXYZtoDiscreteXYZ(1.49, 0.99, 0.0, x, y, z));
  EXPECT_EQ(99, x);
  EXPECT_EQ(99, y);
  EXPECT_EQ(0, z);

  EXPECT_TRUE(env.testContinuousXYZtoDiscreteXYZ(-0.5, -1.0, 0.0, x, y, z));
  EXPECT_EQ(0, x);
  EXPECT_EQ(0, y);
  EXPECT_EQ(0, z);

  EXPECT_TRUE(env.testContinuousXYZtoDiscreteXYZ(-0.5, -1.0, 1.99, x, y, z));
  EXPECT_EQ(0, x);
  EXPECT_EQ(0, y);
  EXPECT_EQ(99, z);
}

// Test to make sure that math of BFS and Distance Field line up!
TEST(testSpace, testFieldBFS)
{
  sbpl_interface::SBPLPlanningParams params;

  distance_field::PropagationDistanceField field(params.field_x, params.field_y, params.field_z,
                                                 params.field_resolution,
                                                 params.field_origin_x, params.field_origin_y, params.field_origin_z,
                                                 params.field_z /* max distance, all cells initialize to this */);

  // Check size
  EXPECT_EQ(100, field.getXNumCells());
  EXPECT_EQ(100, field.getYNumCells());
  EXPECT_EQ(100, field.getZNumCells());  

  // Add an obstacle
  EigenSTL::vector_Vector3d points;
  Eigen::Vector3d point(1., 0., 1.);
  points.push_back(point);
  field.addPointsToField(points);

  sbpl_interface::BFS_3D bfs(field.getXNumCells(), field.getYNumCells(), field.getZNumCells());
  int x, y, z;
  bfs.getDimensions(&x, &y, &z);
  EXPECT_EQ(100, x);
  EXPECT_EQ(100, y);
  EXPECT_EQ(100, z);

  // Check coordinates
  field.worldToGrid(0.0, 0.0, 0.0, x, y, z);
  EXPECT_EQ(25, x);
  EXPECT_EQ(50, y);
  EXPECT_EQ(0, z);

  params.planning_link_sphere_radius = 0.05;
  int walls = sbpl_interface::fillBFSfromField(&field, &bfs, params);

  EXPECT_EQ(81, walls);
  EXPECT_EQ(0.0, field.getDistance(1., 0., 1.));
  EXPECT_EQ(0.1, field.getDistance(1.1, 0., 1.));

  EXPECT_EQ(0.0, field.getDistance(75, 50, 50));
  EXPECT_TRUE(bfs.isWall(75, 50, 50));

  EXPECT_EQ(0.1, field.getDistance(70, 50, 50));
  EXPECT_EQ(0.08, field.getDistance(71, 50, 50));
  EXPECT_EQ(0.06, field.getDistance(72, 50, 50));
  EXPECT_EQ(0.04, field.getDistance(73, 50, 50));
  EXPECT_EQ(0.02, field.getDistance(74, 50, 50));
  EXPECT_EQ(0.02, field.getDistance(76, 50, 50));
  EXPECT_EQ(0.04, field.getDistance(77, 50, 50));
  EXPECT_EQ(0.06, field.getDistance(78, 50, 50));
  EXPECT_EQ(0.08, field.getDistance(79, 50, 50));
  EXPECT_EQ(0.1, field.getDistance(80, 50, 50));
  EXPECT_FALSE(bfs.isWall(72, 50, 50));
  EXPECT_TRUE(bfs.isWall(73, 50, 50));
  EXPECT_TRUE(bfs.isWall(77, 50, 50));
  EXPECT_FALSE(bfs.isWall(78, 50, 50));

  EXPECT_EQ(0.04, field.getDistance(75, 52, 50));
  EXPECT_EQ(0.06, field.getDistance(75, 53, 50));
  EXPECT_FALSE(bfs.isWall(75, 47, 50));
  EXPECT_TRUE(bfs.isWall(75, 48, 50));
  EXPECT_TRUE(bfs.isWall(75, 52, 50));
  EXPECT_FALSE(bfs.isWall(75, 53, 50));

  EXPECT_EQ(0.04, field.getDistance(75, 50, 48));
  EXPECT_EQ(0.06, field.getDistance(75, 50, 47));
  EXPECT_FALSE(bfs.isWall(75, 50, 47));
  EXPECT_TRUE(bfs.isWall(75, 50, 48));
  EXPECT_TRUE(bfs.isWall(75, 50, 52));
  EXPECT_FALSE(bfs.isWall(75, 50, 53));

  // random spots
  EXPECT_FALSE(bfs.isWall(81, 50, 8));
  EXPECT_FALSE(bfs.isWall(45, 50, 50));
  EXPECT_FALSE(bfs.isWall(83, 10, 50));
  EXPECT_FALSE(bfs.isWall(90, 25, 50));
  EXPECT_FALSE(bfs.isWall(0, 0, 0));

  bfs.run(80, 50, 50);
  EXPECT_EQ(0, bfs.getDistance(80, 50, 50));
  EXPECT_EQ(1, bfs.getDistance(81, 51, 51));
  EXPECT_EQ(INT_MAX, bfs.getDistance(75, 50, 50));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
