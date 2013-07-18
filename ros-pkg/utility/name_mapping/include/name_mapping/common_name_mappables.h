#ifndef COMMON_NAME_MAPPABLES_H
#define COMMON_NAME_MAPPABLES_H

#include <name_mapping/name_mapping.h>

//! For the rgb colors representing object classes.
//! Uses "cmap".
class ColorWheel : public NameMappable
{
public:
  //! Call applyNameMapping("cmap", cmap) to generate default color map
  //! of the right size.
  ColorWheel();
  void setColor(const std::string& name, const Eigen::Vector3i& rgb);
  
  Eigen::Vector3i color(size_t idx) const;
  Eigen::Vector3i color(const std::string& name) const;
  size_t size() const { return (size_t)colors_.cols(); }
  std::string status(const std::string& prefix = "") const;
  Eigen::Vector3i defaultColor(size_t idx) const;
    
protected:
  //! Each col corresponds to the color for a class.
  Eigen::Matrix<int, 3, Eigen::Dynamic> colors_;
  Eigen::Matrix<int, 3, Eigen::Dynamic> default_colors_;
  std::map<std::string, Eigen::VectorXi> special_names_;
  
  void _applyNameTranslator(const std::string& nmid, const NameTranslator& translator);
};

#endif // COMMON_NAME_MAPPABLES_H
