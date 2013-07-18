#include <fstream>
#include <yaml-cpp/yaml.h>
#include <pipeline/params.h>

using namespace std;
using namespace pl;

int main(int argc, char** argv)
{
  YAML::Node doc = YAML::LoadFile("data/example.yml");

  const YAML::Node& pods = doc["Pods"];
  for(size_t i = 0; i < pods.size(); ++i) {
    const YAML::Node& pod = pods[i];

    cout << " --- " << endl;
    cout << pod["Name"].as<string>() << endl;
    cout << pod["Type"].as<string>() << endl;

    if(pod["Inputs"]) {
      const YAML::Node& inputs = pod["Inputs"];
      for(YAML::const_iterator it = inputs.begin(); it != inputs.end(); ++it) { 
        cout << it->first.as<string>() << " <- " << it->second.as<string>() << endl;
      }
    }

    if(pod["Params"]) {
      Params params = pod["Params"].as<Params>();
      cout << params << endl;
      // This should really be done automatically for me.
      //YAML::Node node = params;
      YAML::Node node = YAML::convert<Params>::encode(params);
      
      // They aren't equal, but it's only the ordering that changes.
      //assert(node == pod["Params"]);
    }
  }
  
  return 0;
}
  
