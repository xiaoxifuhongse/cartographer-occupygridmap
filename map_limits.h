#ifndef MAP_LIMITS_H
#define MAP_LIMITS_H
#include <utility>
#include <vector>
#include "math1.h"
#include "xy_index.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
// Defines the limits of a grid map. This class must remain inlined for
// performance reasons.
class MapLimits {
 public:
  MapLimits(const double resolution, const Eigen::Vector2d& max,
            const CellLimits& cell_limits)
      : resolution_(resolution), max_(max), cell_limits_(cell_limits) {
    CHECK_GT(resolution_, 0.);
    CHECK_GT(cell_limits.num_x_cells, 0.);
    CHECK_GT(cell_limits.num_y_cells, 0.);
  }



  // Returns the cell size in meters. All cells are square and the resolution is
  // the length of one side.
  double resolution() const { return resolution_; }

  // Returns the corner of the limits, i.e., all pixels have positions with
  // smaller coordinates.
  const Eigen::Vector2d& max() const { return max_; }

  // Returns the limits of the grid in number of cells.
  const CellLimits& cell_limits() const { return cell_limits_; }

  // Returns the index of the cell containing the 'point' which may be outside
  // the map, i.e., negative or too large indices that will return false for
  // Contains().
  Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const {
    // Index values are row major and the top left has Eigen::Array2i::Zero()
    // and contains (centered_max_x, centered_max_y). We need to flip and
    // rotate.
    return Eigen::Array2i(
        RoundToInt((max_.y() - point.y()) / resolution_ - 0.5),
        RoundToInt((max_.x() - point.x()) / resolution_ - 0.5));
  }

  // Returns true if the ProbabilityGrid contains 'cell_index'.
  bool Contains(const Eigen::Array2i& cell_index) const {
    return (Eigen::Array2i(0, 0) <= cell_index).all() &&
           (cell_index <
            Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
               .all();
  }

 private:
  double resolution_;
  Eigen::Vector2d max_;
  CellLimits cell_limits_;
};
#endif // MAP_LIMITS_H
