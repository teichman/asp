#include <gtest/gtest.h>
#include <graphcuts/maxflow_inference.h>
#include <graphcuts/structural_svm.h>

using namespace Eigen;
using namespace std;
namespace gc = graphcuts;

TEST(Model, NameMapping)
{
  VectorXd eweights(3);
  eweights << 0, M_PI, 42.13;
  VectorXd nweights(2);
  nweights << M_PI, 42.13;
  NameMapping emap;
  emap.addName("edge_descriptor0");
  emap.addName("edge_descriptor1");
  emap.addName("edge_descriptor2");
  NameMapping nmap;
  nmap.addName("node_descriptor0");
  nmap.addName("node_descriptor1");

  gc::Model m;
  m.applyNameMapping("nmap", nmap);
  m.applyNameMapping("emap", emap);
  m.eweights_ = eweights;
  m.nweights_ = nweights;
  cout << m << endl;
}

TEST(Model, Serialization)
{
  VectorXd eweights(3);
  eweights << 0, M_PI, 42.13;
  NameMapping emap;
  emap.addName("edge_descriptor0");
  emap.addName("edge_descriptor1");
  emap.addName("edge_descriptor2");
  VectorXd nweights(2);
  nweights << M_PI, 42.13;
  NameMapping nmap;
  nmap.addName("node_descriptor0");
  nmap.addName("node_descriptor1");

  gc::Model model(nweights, eweights, nmap, emap);
  model.save("test_model");
  cout << model << endl;
  
  gc::Model model2;
  model2.load("test_model");
  cout << model2 << endl;

  EXPECT_TRUE(model.nameMappingsAreEqual(model2));
  EXPECT_TRUE(model.concatenate().rows() == model2.concatenate().rows());
  for(int i = 0; i < model2.concatenate().rows(); ++i)
    EXPECT_FLOAT_EQ(model.concatenate()(i), model2.concatenate()(i));

  VectorXd psi(5);
  psi << 0, 1, 2, 3, 4;
  EXPECT_FLOAT_EQ(model.concatenate().dot(psi), model.score(psi));
}

TEST(StructuralSVM, SanityCheck)
{
  double c = 10;
  double precision = 1e-4;
  int num_threads = 1;
  int debug_level = 0;
  gc::StructuralSVM ssvm(c, precision, num_threads, debug_level);

  // -- Generate data.
  gc::PotentialsCache::Ptr cache(new gc::PotentialsCache);
  NameMapping nmap;
  nmap.addName("GoodDescriptor");
  nmap.addName("ReversedDescriptor");
  cache->applyNameMapping("nmap", nmap);
  NameMapping emap;
  emap.addName("GoodEdge");
  emap.addName("BadEdge");
  cache->applyNameMapping("emap", emap);
  
  gc::VecXiPtr label(new gc::VecXi(10));
  *label << -1, -1, -1, -1, -1, 1, 1, 1, 1, 1;

  VectorXd good_node(10);
  good_node << 0, 0, 0, 0, -1, 1, 0, 0, 0, 0;
  cache->node_[0] = good_node;
  cache->node_[1] = -good_node;
  gc::DynamicSparseMat good_edge(10, 10);
  for(int i = 1; i < good_edge.rows(); ++i)
    good_edge.coeffRef(i-1, i) = 1.0;
  cache->edge_[0] = gc::SparseMat(good_edge);

  gc::DynamicSparseMat bad_edge(10, 10);
  bad_edge.coeffRef(4, 5) = 100;
  cache->edge_[1] = bad_edge;

  cache->symmetrizeEdges();
    
  // -- Train model.
  vector<gc::PotentialsCache::Ptr> caches;
  vector<gc::VecXiPtr> labels;
  caches.push_back(cache);
  labels.push_back(label);
  caches.push_back(cache);
  labels.push_back(label);
  gc::Model model = ssvm.train(caches, labels);
  cout << model << endl;

  // -- Run inference on data and make sure it's reasonable.
  gc::MaxflowInference mfi(model);
  VectorXi seg;
  mfi.segment(cache, &seg);
  EXPECT_TRUE(cache->numNodes() == seg.rows());
  cout << label->transpose() << endl;
  cout << seg.transpose() << endl;

  for(int i = 0; i < seg.rows(); ++i)
    EXPECT_TRUE(seg(i) == label->coeffRef(i));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
