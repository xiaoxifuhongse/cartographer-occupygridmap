#ifndef GRID_2D_H
#define GRID_2D_H
#include <vector>
#include "map_limits.h"
class Grid2D
{
public:
    explicit Grid2D(const MapLimits& limits,
                    float min_correspondence_cost,
                    float max_correspondence_cost);

    const MapLimits& limits() const {return limits_;}

    void FinishUpdate();

    float GetCorrespondenceCost(const Eigen::Array2i& cell_index) const;
    // Returns the minimum possible correspondence cost.
    float GetMinCorrespondenceCost() const { return min_correspondence_cost_; }

    // Returns the maximum possible correspondence cost.
    float GetMaxCorrespondenceCost() const { return max_correspondence_cost_; }

    // Returns true if the probability at the specified index is known.
    bool IsKnown(const Eigen::Array2i& cell_index) const;

    // Fills in 'offset' and 'limits' to define a subregion of that contains all
    // known cells.
    void ComputeCroppedLimits(Eigen::Array2i* const offset,
                              CellLimits* const limits) const;

    // Grows the map as necessary to include 'point'. This changes the meaning of
    // these coordinates going forward. This method must be called immediately
    // after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
    virtual void GrowLimits(const Eigen::Vector2f& point);

    virtual std::unique_ptr<Grid2D> ComputeCroppedGrid() const = 0;

protected:
     const std::vector<uint16>& correspondence_cost_cells() const
     {
       return correspondence_cost_cells_;
     }
     const std::vector<int>& update_indices() const { return update_indices_; }
     const Eigen::AlignedBox2i& known_cells_box() const {return known_cells_box_;}

     std::vector<uint16>* mutable_correspondence_cost_cells() {return &correspondence_cost_cells_;}
     std::vector<int>* mutable_update_indices() { return &update_indices_; }
     Eigen::AlignedBox2i* mutable_known_cells_box() { return &known_cells_box_; }

    // Converts a 'cell_index' into an index into 'cells_'.
    int ToFlatIndex(const Eigen::Array2i& cell_index) const;

private:
     MapLimits limits_;
     std::vector<uint16> correspondence_cost_cells_;
     float min_correspondence_cost_;
     float max_correspondence_cost_;
     std::vector<int> update_indices_;

     // Bounding box of known cells to efficiently compute cropping limits.
     Eigen::AlignedBox2i known_cells_box_;
};

#endif // GRID_2D_H
