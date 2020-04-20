
#ifndef BIKELIGHTUTIL_h
#define BIKELIGHTUTIL_h

#include <sstream>
#include <string>

namespace BikeLight {

std::stringstream get_fixed_stringstream() {
  std::stringstream ss;
  ss.precision(1);
  ss << std::fixed;
  return ss;
}

}  // namespace BikeLight

#endif