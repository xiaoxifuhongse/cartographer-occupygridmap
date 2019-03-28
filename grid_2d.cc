#include "grid_2d.h"
#include "probability_values.h"

Grid2D::Grid2D(const MapLimits& limits,
               float min_correspondence_cost,
               float max_correspondence_cost)
                                                : limits_(limits),
                                                  correspondence_cost_cells_(
                                                      limits_.cell_limits().num_x_cells * limits_.cell_limits().num_y_cells,
                                                      kUnknownCorrespondenceValue),
                                                  min_correspondence_cost_(min_correspondence_cost),
                                                  max_correspondence_cost_(max_correspondence_cost)
{
    CHECK_LT(min_correspondence_cost_, max_correspondence_cost_);
}

// Finishes the update sequence.
void Grid2D::FinishUpdate()
{
    while (!update_indices_.empty())
    {
        DCHECK_GE(correspondence_cost_cells_[update_indices_.back()],
                  kUpdateMarker);
        correspondence_cost_cells_[update_indices_.back()] -= kUpdateMarker;
        update_indices_.pop_back();
    }
}
// Returns the correspondence cost of the cell with 'cell_index'.
float Grid2D::GetCorrespondenceCost(const Eigen::Array2i& cell_index) const
{
  if (!limits().Contains(cell_index))
      return kMaxCorrespondenceCost;
  return ValueToCorrespondenceCost(
      correspondence_cost_cells()[ToFlatIndex(cell_index)]);
}

// Returns true if the correspondence cost at the specified index is known.
bool Grid2D::IsKnown(const Eigen::Array2i& cell_index) const {
  return limits_.Contains(cell_index) &&
         correspondence_cost_cells_[ToFlatIndex(cell_index)] !=
             kUnknownCorrespondenceValue;
}

// Fills in 'offset' and 'limits' to define a subregion of that contains all
// known cells.
void Grid2D::ComputeCroppedLimits(Eigen::Array2i* const offset,
                                  CellLimits* const limits) const {
  if (known_cells_box_.isEmpty()) {
    *offset = Eigen::Array2i::Zero();
    *limits = CellLimits(1, 1);
    return;
  }
  *offset = known_cells_box_.min().array();
  *limits = CellLimits(known_cells_box_.sizes().x() + 1,
                       known_cells_box_.sizes().y() + 1);
}

// Grows the map as necessary to include 'point'. This changes the meaning of
// these coordinates going forward. This method must be called immediately
// after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
void Grid2D::GrowLimits(const Eigen::Vector2f& point) {
  CHECK(update_indices_.empty());
  while (!limits_.Contains(limits_.GetCellIndex(point))) {
    const int x_offset = limits_.cell_limits().num_x_cells / 2;
    const int y_offset = limits_.cell_limits().num_y_cells / 2;
    const MapLimits new_limits(
        limits_.resolution(),
        limits_.max() +
            limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
        CellLimits(2 * limits_.cell_limits().num_x_cells,
                   2 * limits_.cell_limits().num_y_cells));
    const int stride = new_limits.cell_limits().num_x_cells;
    const int offset = x_offset + stride * y_offset;
    const int new_size = new_limits.cell_limits().num_x_cells *
                         new_limits.cell_limits().num_y_cells;
    std::vector<uint16> new_cells(new_size, kUnknownCorrespondenceValue);
    for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) {
      for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) {
        new_cells[offset + j + i * stride] =
            correspondence_cost_cells_[j +
                                       i * limits_.cell_limits().num_x_cells];
      }
    }
    correspondence_cost_cells_ = new_cells;
    limits_ = new_limits;
    if (!known_cells_box_.isEmpty()) {
      known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
    }
  }
}

int Grid2D::ToFlatIndex(const Eigen::Array2i& cell_index) const {
  CHECK(limits_.Contains(cell_index)) << cell_index;
  return limits_.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
}
