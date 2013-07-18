#ifndef SIMPLE_SEGMENTATION_PIPELINE_H
#define SIMPLE_SEGMENTATION_PIPELINE_H

#include <asp/asp.h>

namespace asp
{
  namespace example
  {
    using namespace std;
    using namespace Eigen;
    using namespace asp;

    //! Anything that generates a node potential should inherit from
    //! NodePotentialGenerator.
    class RandomNPG : public NodePotentialGenerator
    {
    public:
      //! This is necessary for the Pipeline library to know how to work
      //! with this type of Pod.
      DECLARE_POD(RandomNPG);

      //! All Pods should have a constructor that only takes their name.
      RandomNPG(std::string name) :
        NodePotentialGenerator(name)
      {
      }

      //! This is the main computation that this pod does.
      //! In this case we'll just set node potentials randomly.
      void compute();

      //! The debug function is only called if debugging mode is on.
      //! Here, we'll write out a nice visualization of the node potentials.
      void debug() const { writeNodePotentialVisualization(); }
    };

    void RandomNPG::compute()
    {
      initializeStorage();

      for(int y = 0; y < node_.rows(); ++y)
        for(int x = 0; x < node_.cols(); ++x)
          node_(y, x) = ((double)rand() / RAND_MAX) * 2.0 - 1.0;

      push<const MatrixXd*>("Node", &node_);
    }

    class RandomEPG : public EdgePotentialGenerator
    {
    public:
      DECLARE_POD(RandomEPG);
      RandomEPG(std::string name) :
        EdgePotentialGenerator(name)
      {
      }

      void compute();
      void debug() const { writeEdgePotentialVisualization(); }
    };

    void RandomEPG::compute()
    {
      initializeStorage();
      const SparseMat& structure = *pull<const SparseMat*>("EdgeStructure");

      for(int i = 0; i < structure.rows(); ++i) {
        SparseMat::InnerIterator it(structure, i);
        for(; it; ++it)
          edge_.insert(it.row(), it.col()) = (double)rand() / RAND_MAX;
      }

      push<const SparseMat*>("Edge", &edge_);
    }
    
    void generateSimpleSegmentationPipeline(Asp* asp)
    {
      asp->addPod(new RandomNPG("RandomNPG0"));
      // When RandomNPG0 calls pull<cv::Mat3b>("Image"), it will get the img that
      // ImageEntryPoint pushed to its "Output" port with push<cv::Mat3b>("Output", img).
      asp->connect("ImageEntryPoint.Output -> RandomNPG0.Image");
      asp->connect("RandomNPG0.Node -> NodePotentialAggregator.UnweightedNode");

      // An EdgeStructureGenerator is just a way of saying what edges should be computed
      // by downstream EdgePotentialGenerators.
      asp->addPod(new EdgeStructureGenerator("GridESG"));
      // Because we want to be able to automatically twiddle with and then serialize
      // and entire pipeline structure and the params of all pods in it, the pipeline
      // library has special handling for pod params.
      // This just says that our EdgeStructureGenerator is going to use a simple grid.
      asp->setParam("GridESG", "Grid", true);
      asp->connect("ImageEntryPoint.Output -> GridESG.Image");
      asp->connect("MaskEntryPoint.Output -> GridESG.Mask");
      // We'll also add some other EdgeStructureGenerators that use different edge structures.
      asp->addPod(new EdgeStructureGenerator("DiagonalESG"));
      asp->setParam("DiagonalESG", "Diagonal", true);
      asp->connect("ImageEntryPoint.Output -> DiagonalESG.Image");
      asp->connect("MaskEntryPoint.Output -> DiagonalESG.Mask");
      asp->addPod(new EdgeStructureGenerator("WebESG"));
      asp->setParam("WebESG", "Web", true);
      asp->setParam("WebESG", "WebNumOutgoing", 5);
      asp->connect("ImageEntryPoint.Output -> WebESG.Image");
      asp->connect("MaskEntryPoint.Output -> WebESG.Mask");
  
      asp->addPod(new SmoothnessEPG("SmoothnessEPG0"));
      asp->connect("ImageEntryPoint.Output -> SmoothnessEPG0.Image");
      asp->connect("SmoothnessEPG0.Edge -> EdgePotentialAggregator.UnweightedEdge");
      asp->connect("GridESG.EdgeStructure -> SmoothnessEPG0.EdgeStructure");
      asp->addPod(new SmoothnessEPG("SmoothnessEPG1"));
      asp->connect("ImageEntryPoint.Output -> SmoothnessEPG1.Image");
      asp->connect("SmoothnessEPG1.Edge -> EdgePotentialAggregator.UnweightedEdge");
      asp->connect("DiagonalESG.EdgeStructure -> SmoothnessEPG1.EdgeStructure");

      asp->addPod(new SimpleColorDifferenceEPG("GridSimpleColorDifferenceEPG"));
      asp->connect("ImageEntryPoint.Output -> GridSimpleColorDifferenceEPG.Image");
      asp->connect("GridESG.EdgeStructure -> GridSimpleColorDifferenceEPG.EdgeStructure");
      asp->connect("GridSimpleColorDifferenceEPG.Edge -> EdgePotentialAggregator.UnweightedEdge");
      asp->addPod(new SimpleColorDifferenceEPG("DiagonalSimpleColorDifferenceEPG"));
      asp->connect("ImageEntryPoint.Output -> DiagonalSimpleColorDifferenceEPG.Image");
      asp->connect("DiagonalESG.EdgeStructure -> DiagonalSimpleColorDifferenceEPG.EdgeStructure");
      asp->connect("DiagonalSimpleColorDifferenceEPG.Edge -> EdgePotentialAggregator.UnweightedEdge");
      asp->addPod(new SimpleColorDifferenceEPG("WebSimpleColorDifferenceEPG"));
      asp->connect("ImageEntryPoint.Output -> WebSimpleColorDifferenceEPG.Image");
      asp->connect("WebESG.EdgeStructure -> WebSimpleColorDifferenceEPG.EdgeStructure");
      asp->connect("WebSimpleColorDifferenceEPG.Edge -> EdgePotentialAggregator.UnweightedEdge");

      // For now, we'll just set some weights by hand.
      // This shows how to generate a default model based on what inputs
      // the NodePotentialAggregator and EdgePotentialAggregator see.
      Model model = asp->defaultModel();
      model.nweights_.setConstant(1);
      // We can set weights of particular features without worrying about what position
      // in the vector they are at.
      // "nmap" stands for "node potential name mapping".
      model.nweights_(model.nameMapping("nmap").toId("SeedNPG")) = 2;
      model.nweights_(model.nameMapping("nmap").toId("RandomNPG0")) = 0;
      model.nweights_(model.nameMapping("nmap").toId("PriorNPG")) = 0.01;
      model.eweights_.setConstant(1);
      // "emap" stands for "edge potential name mapping".
      model.eweights_(model.nameMapping("emap").toId("SmoothnessEPG0")) = 0.1;
      model.eweights_(model.nameMapping("emap").toId("SmoothnessEPG1")) = 0.1;
      model.eweights_(model.nameMapping("emap").toId("WebSimpleColorDifferenceEPG")) = 0.3;
      // This sets the weights appropriately in the
      // NodePotentialAggregator and EdgePotentialAggregator.
      asp->setModel(model);
    }
  }
}
  

#endif // SIMPLE_SEGMENTATION_PIPELINE_H
