#ifndef IMAGE_INTERACTION__TYPES_HPP_
#define IMAGE_INTERACTION__TYPES_HPP_

#include <array>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace image_interaction {
using point = std::array<int, 2>;
using points = std::vector<point>;
struct annotations {
  points positive, negative;
};
using labeled_annotations = std::map<std::string, annotations>;
} // namespace image_interaction
#endif // IMAGE_INTERACTION__TYPES_HPP_
