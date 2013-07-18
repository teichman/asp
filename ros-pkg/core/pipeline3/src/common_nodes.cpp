#include <pipeline/common_nodes.h>

using namespace std;

namespace pl
{

  void PlaceholderNode::_compute()
  {
    int val = pull<int>("incoming");
    cout << getName() << " got value: " << val << ".  Param foo is " << param<int>("foo") << endl;
    push<int>("outgoing", val * param<int>("foo"));
  }
    
} // namespace pl
