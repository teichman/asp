#include <name_mapping/common_name_mappables.h>

using namespace std;
using namespace Eigen;

ColorWheel::ColorWheel()
{
  vector<VectorXi> colors;
  for(int r = 0; r < 255; r += 20) {
    for(int g = 0; g < 255; g += 20) {
      for(int b = 0; b < 255; b += 20) {
        VectorXi color(3);
        color(0) = r;
        color(1) = g;
        color(2) = b;
        colors.push_back(color);
      }
    }
  }

  default_colors_ = Matrix<int, 3, Dynamic>(3, colors.size());
  for(size_t i = 0; i < colors.size(); ++i)
    default_colors_.col(i) = colors[i];
}

std::string ColorWheel::status(const std::string& prefix) const
{
  ostringstream oss;
  oss << prefix << "ColorWheel" << endl;
  for(size_t i = 0; i < nameMapping("cmap").size(); ++i)
    oss << prefix << "  " << nameMapping("cmap").toName(i) << ": " << color(i).transpose() << endl;

  return oss.str();
}

void ColorWheel::setColor(const std::string& name, const Eigen::Vector3i& rgb)
{
  size_t idx = nameMapping("cmap").toId(name);
  colors_.col(idx) = rgb;
  special_names_[name] = rgb;
}

Eigen::Vector3i ColorWheel::color(size_t idx) const
{
  return colors_.col(idx);
}

Eigen::Vector3i ColorWheel::color(const std::string& name) const
{
  return color(nameMapping("cmap").toId(name));
}

Eigen::Vector3i ColorWheel::defaultColor(size_t idx) const
{
  return default_colors_.col(idx % default_colors_.cols());
}

void ColorWheel::_applyNameTranslator(const std::string& nmid, const NameTranslator& translator)
{
  ROS_ASSERT(nmid == "cmap");
  translator.translateCols(&colors_, -1);

  // -- Add a new color for any new names.
  for(int i = 0; i < colors_.cols(); ++i)
    if((colors_.col(i).array() == -1).all())
      colors_.col(i) = defaultColor(i);

  // -- Re-apply any special color mappings.
  map<string, VectorXi>::iterator it;
  for(it = special_names_.begin(); it != special_names_.end(); ++it)
    colors_.col(nameMapping("cmap").toId(it->first)) = it->second;
}

