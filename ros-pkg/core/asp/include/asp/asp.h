#ifndef ASP_H
#define ASP_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <eigen_extensions/random.h>
#include <pipeline/pipeline.h>
#include <graphcuts/potentials_cache.h>
#include <name_mapping/name_mapping.h>

namespace asp
{
  using namespace gc;
  using namespace pl;

  typedef boost::shared_ptr<Eigen::MatrixXd> MatrixXdPtr;
  typedef boost::shared_ptr<const Eigen::MatrixXd> MatrixXdConstPtr;
  
  //! Abstract Segmentation Pipeline.
  //! See test_asp.cpp for example usage.
  class Asp : public Pipeline
  {
  public:
    //! Initialize using initializePipeline.
    Asp(int num_threads);
    //! Initialize using the Pods described in config.
    Asp(int num_threads, YAML::Node config);
    virtual ~Asp() {};

    //! Names are automatically filled in based on Pipeline pod names.
    //! Weights need not be set in NPA and EPA.
    Model defaultModel() const;
    //! Names are automatically filled in based on Pipeline pod names.
    //! Weights must be set in NPA and EPA.
    Model model() const;
    //! Installs weights into NPA and EPA.
    //! Names in nmap and emap must match node potential and edge potential pod names.
    void setModel(const Model& model);

    //! You have to set the input to the pipeline yourself first.
    //! Also, setModel() must have been called.
    //! segmentation will be resized if it is not the correct length.
    //! cache will be filled if it is not null.
    //!
    //! seg_img is 255 for fg, 0 for bg, 127 for unknown.
    //! cache and seg_vec and for training.  See graphcuts/structural_svm.h.
    void segment(cv::Mat1b* seg_img, PotentialsCache* cache = NULL);
                     
  protected:
    //! Initializes pipeline_ with NodePotentialAggregator, EdgePotentialAggregator, 
    //! and GraphCuts pods.  You then hook up your own things to this.
    void initializePipeline();
    void registerPodTypes();
  };

  // Unweighted node potentials should be in [-1, 1].
  // This value represents the affinity for label +1.
  // Affinity for label -1 is always set to zero.
  // Set the node potential to be negative to express a desire for
  // that pixel to be label -1, and positive for label +1.
  class NodePotentialGenerator : public Pod
  {
  public:
    NodePotentialGenerator(std::string name) :
      Pod(name)
    {
      // Required for visualization and for initializeStorage().
      declareInput<cv::Mat3b>("Image");  
      declareOutput<const Eigen::MatrixXd*>("Node");
    }

  protected:
    //! Values of the node potentials.
    //! Fill this and push out a pointer to it in the compute() function.
    Eigen::MatrixXd node_;

    //! Uses Image to set the size of node_ if necessary
    //! and initialize their elements to zero.
    //! This is typically called at the start of compute().
    void initializeStorage();
    
    //! Displays node potentials in a common format.
    //! Typically, you call this in debug().
    //! Saves to disk as pod::debugBasePath() + "-raw.png" and
    //! (if Image is provided) as pod::debugBasePath() + "-overlay.png".
    void writeNodePotentialVisualization() const;
  };

  //! All NodePotentialGenerators get connected to an object of this type.
  //! It adds them together in a weighted combination and outputs the result
  //! for graph cuts to use.
  class NodePotentialAggregator : public NodePotentialGenerator
  {
  public:
    DECLARE_POD(NodePotentialAggregator);
    NodePotentialAggregator(std::string name) :
      NodePotentialGenerator(name)
    {
      declareMultiInput<const Eigen::MatrixXd*>("UnweightedNode");
    }

    NameMapping generateNameMapping() const;
    //! Copy of model is translated using local name mapping.
    void setWeights(Model model);
    //! Sets nmap names based on input pod names.
    //! Sets model->nweights_ to local nweights_.
    void fillModel(Model* model) const;
    void fillPotentialsCache(PotentialsCache* pc) const;
    
  protected:
    Eigen::MatrixXd aggregated_;
    Eigen::VectorXd nweights_;

    void compute();
    void debug() const;
  };

  //! Unweighted edge potentials should be in [0, 1].
  class EdgePotentialGenerator : public Pod
  {
  public:
    EdgePotentialGenerator(std::string name) :
      Pod(name)
    {
      declareInput<cv::Mat3b>("Image");
      declareInput<const SparseMat*>("EdgeStructure");
      declareOutput<const SparseMat*>("Edge");
    }

    SparseMat edge_;

    //! Allocates new sparse matrix if necessary; just sets to zero otherwise.
    //! Size is determined by Image.
    //! For efficiency, you should set reserve_per_node to higher than the expected
    //! number of edges per node.
    void initializeStorage(double reserve_per_node = 6);

    void writeEdgePotentialVisualization(bool white_background = false) const;
  };

  class EdgePotentialAggregator : public EdgePotentialGenerator
  {
  public:
    DECLARE_POD(EdgePotentialAggregator);
    EdgePotentialAggregator(std::string name) :
      EdgePotentialGenerator(name)
    {
      declareMultiInput<const SparseMat*>("UnweightedEdge");
    }

    NameMapping generateNameMapping() const;
    //! Copy of model is translated using local name mapping.
    void setWeights(Model model);
    //! Sets nmap names based on input pod names.
    //! Sets model->nweights_ to local nweights_.
    void fillModel(Model* model) const;
    void fillPotentialsCache(PotentialsCache* pc) const;
    
  protected:
    Eigen::VectorXd eweights_;

    void compute();
    void debug() const;
  };

  class PriorNPG : public NodePotentialGenerator
  {
  public:
    DECLARE_POD(PriorNPG);
    PriorNPG(std::string name) :
      NodePotentialGenerator(name)
    {
    }

  protected:
    void compute();
    void debug() const { writeNodePotentialVisualization(); }
  };
  
  class GraphcutsPod : public Pod
  {
  public:
    DECLARE_POD(GraphcutsPod);
    GraphcutsPod(std::string name) :
      Pod(name)
    {
      declareInput<const Eigen::MatrixXd*>("AggregatedNodePotentials");
      declareInput<const SparseMat*>("AggregatedEdgePotentials");
      declareInput<cv::Mat3b>("Image");

      // 255 <-> +1; 0 <-> -1; 127 <-> unknown.
      declareOutput<cv::Mat1b>("Segmentation");

      declareParam<int>("ExpectedNumEdges", 10);
    }
    
  protected:
    cv::Mat1b seg_;
    
    void compute();
    //! Saves image of final segmentation.
    void debug() const;
  };

  class SeedNPG : public NodePotentialGenerator
  {
  public:
    DECLARE_POD(SeedNPG);
    SeedNPG(std::string name) :
      NodePotentialGenerator(name)
    {
      // 255 -> foreground
      // 0 -> background
      declareInput<cv::Mat1b>("SeedImage");
    }

  protected:
    void compute();
    void debug() const;
  };

  class EdgeStructureGenerator : public Pod
  {
  public:
    DECLARE_POD(EdgeStructureGenerator);
    EdgeStructureGenerator(std::string name):
      Pod(name)
    {
      declareParam<bool>("Grid", false);
      declareParam<bool>("Diagonal", false);
      declareParam<bool>("Web", false);
      declareParam<double>("WebMaxRadius", 10);  // In pixels.
      declareParam<double>("WebNumOutgoing", 2);
      
      declareInput<cv::Mat3b>("Image");
      declareInput<cv::Mat1b>("Mask");
            
      // Upper triangular.  EdgeStructure(i, j) != 0 means that
      // the non-directional edge between i and j should be
      // computed by downstream Pods.
      declareOutput<const SparseMat*>("EdgeStructure");
    }

  protected:
    SparseMat structure_;
    SparseMat diag_;
    DynamicSparseMat web_;
    
    void compute();
    void debug() const;
  };

  class SmoothnessEPG : public EdgePotentialGenerator
  {
  public:
    DECLARE_POD(SmoothnessEPG);
    SmoothnessEPG(std::string name) :
      EdgePotentialGenerator(name)
    {
    }

  protected:
    void compute();
    void debug() const;
  };
  
  class SimpleColorDifferenceEPG : public EdgePotentialGenerator
  {
  public:
    DECLARE_POD(SimpleColorDifferenceEPG);
    SimpleColorDifferenceEPG(std::string name) :
      EdgePotentialGenerator(name)
    {
      declareParam<double>("Sigma", 30);
    }

  protected:
    void compute();
    void debug() const;
  };

  
  /************************************************************
   * Miscellaneous functions
   ************************************************************/
  
  //! Common function so there is no confusion about the use of row-major.
  inline int index(int row, int col, int width) { return col + row * width; }
  inline void pixel(int idx, int width, int* row, int* col)
  {
    *row = idx / width;
    *col = idx - *row * width;
  }
  
  void visualizeSegmentation(cv::Mat1b seg, cv::Mat3b img, cv::Mat3b vis);
  cv::Mat3b drawEdgeVisualization(cv::Mat3b img, const SparseMat& edge);
  void imageToVectorSegmentation(cv::Mat1b seg_img, Eigen::VectorXi* seg_vec);
  
  //! This function is specifically for generating adjacency matrices for an image.
  //! The matrix is (rows * cols) x (rows * cols).
  //! rows and cols are the size of the image.
  //! reserve_per_node is the number of edges you expect per pixel.
  template<typename T>
  void initializeSparseMat(int rows, int cols, double reserve_per_node, T* mat)
  {
    int num_pixels = rows * cols;
    if(mat->rows() != num_pixels || mat->cols() != num_pixels) {
      *mat = SparseMat(num_pixels, num_pixels);
      mat->reserve((int)(reserve_per_node * num_pixels));
    }
    mat->setZero();
  }
}

#endif // ASP_H
