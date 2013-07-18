#include <graphcuts/model.h>

using namespace Eigen;
using namespace std;

namespace gc
{

  Model::Model()
  {
  }

  Model::Model(const Eigen::VectorXd& nweights,
               const Eigen::VectorXd& eweights,
               const NameMapping& nmap,
               const NameMapping& emap)
  {
    applyNameMapping("nmap", nmap);
    applyNameMapping("emap", emap);
    nweights_ = nweights;  // Initialization of name-mapped data must come after applyNameMapping() call.
    eweights_ = eweights;
  }
  
  void Model::serialize(std::ostream& out) const
  {
    ROS_ASSERT(!hasNameMapping("nmap") || nweights_.rows() == (int)nameMapping("nmap").size());
    ROS_ASSERT(!hasNameMapping("emap") || eweights_.rows() == (int)nameMapping("emap").size());
    
    out << "Graphcuts Model v0.1" << endl;
    out << "== " << nweights_.rows() << " node potential weights == " << endl;
    for(int i = 0; i < nweights_.rows(); ++i)
      out << nameMapping("nmap").toName(i) << "\t" << setprecision(16) << nweights_(i) << endl;
    out << "== " << eweights_.rows() << " edge potential weights == " << endl;
    for(int i = 0; i < eweights_.rows(); ++i)
      out << nameMapping("emap").toName(i) << "\t" << setprecision(16) << eweights_(i) << endl;
  }
  
  void Model::deserialize(std::istream& in)
  {    
    string buf;
    getline(in, buf);
    ROS_ASSERT(buf.compare("Graphcuts Model v0.1") == 0);

    in >> buf;
    int num_node;
    in >> num_node;
    getline(in, buf);
    vector<string> nnames(num_node);
    VectorXd nweights(num_node);
    for(int i = 0; i < num_node; ++i) {
      in >> nnames[i];
      in >> nweights(i);
    }
    NameMapping nmap;
    nmap.addNames(nnames);
    applyNameMapping("nmap", nmap);
    nweights_ = nweights;  // Initialization of name-mapped data must come after applyNameMapping() call.
    
    in >> buf;
    int num_edge;
    in >> num_edge;
    getline(in, buf);
    vector<string> enames(num_edge);
    VectorXd eweights(num_edge);
    for(int i = 0; i < num_edge; ++i) {
      in >> enames[i];
      in >> eweights(i);
    }
    NameMapping emap;
    emap.addNames(enames);
    applyNameMapping("emap", emap);
    eweights_ = eweights;  // Initialization of name-mapped data must come after applyNameMapping() call.
  }

  double Model::score(const VectorXd& psi) const
  {
    ROS_ASSERT(psi.rows() == eweights_.rows() + nweights_.rows());
    double val = psi.head(eweights_.rows()).dot(eweights_);
    val += psi.tail(nweights_.rows()).dot(nweights_);
    return val;
  }

  Eigen::VectorXd Model::concatenate() const
  {
    VectorXd weights(size());
    weights.head(eweights_.rows()) = eweights_;
    weights.tail(nweights_.rows()) = nweights_;
    return weights;
  }

  void Model::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
  {
    ROS_ASSERT(id == "nmap" || id == "emap");
    if(id == "nmap")
      translator.translate(&nweights_, 0.0);
    else if(id == "emap")
      translator.translate(&eweights_, 0.0);
  }

}
