#include <fstream>
#include <yaml-cpp/yaml.h>
#include <pipeline/params.h>
#include <ros/assert.h>

using namespace std;
using namespace pl;

int main(int argc, char** argv)
{
  // Jesus fucking christ they are shared pointers...
  YAML::Node one;
  YAML::Node two;

  ROS_ASSERT(!one["aoeu"]);

  
  one["Foo"] = "Bar";
  two = one;
  two["Foo"] = "Baz";
  cout << one["Foo"] << " " << two["Foo"] << endl;

  // See yaml_clone.cpp
  // one["Foo"] = "Bar";
  // two = YAML::Clone(one);
  // two["Foo"] = "Baz";
  // cout << one["Foo"] << " " << two["Foo"] << endl;
  // ROS_ASSERT(one["Foo"].as<string>() == "Bar" && two["Foo"].as<string>() == "Baz");
  

  YAML::Node doc = YAML::LoadFile("data/example.yml");

  const YAML::Node& pods = doc["Pods"];
  for(size_t i = 0; i < pods.size(); ++i) {
    const YAML::Node& pod = pods[i];

    // cout << " -- " << endl;
    // cout << pod["Name"].IsDefined() << endl;
    // cout << pod["aoeu"].IsDefined() << endl;
    // cout << pod["Name"].IsNull() << endl;
    // cout << pod["aoeu"].IsNull() << endl;
    
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
      for(YAML::const_iterator it = pod["Params"].begin(); it != pod["Params"].end(); ++it) { 
        cout << "Tag: " << it->first.Tag() << " / " << it->second.Tag() << " / "
             << it->first.as<string>() << " :: " << it->second.as<string>() << endl;
      }

      cout << "------------------------------" << endl;
      cout << params.YAMLize() << endl;
      cout << "------------------------------" << endl;
      cout << pod["Params"] << endl;
      cout << "------------------------------" << endl;
      cout << YAML::Dump(pod["Params"]) << endl;
      cout << "------------------------------" << endl;
      assert(params.YAMLize() == pod["Params"]);
    }
  }
  
  return 0;
}
  
