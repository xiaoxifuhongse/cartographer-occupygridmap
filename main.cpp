#include <iostream>
#include "map_limits.h"
#include "probability_grid.h"
#include "probability_values.h"
using namespace std;

//just for simple
typedef std::vector<Eigen::Vector3f> PointCloud;
struct RangeData
{
    Eigen::Vector3f origin;
    PointCloud returns;
    PointCloud misses;
};

// Factor for subpixel accuracy of start and end point.
constexpr int kSubpixelScale = 1000;

// We divide each pixel in kSubpixelScale x kSubpixelScale subpixels. 'begin'
// and 'end' are coordinates at subpixel precision. We compute all pixels in
// which some part of the line segment connecting 'begin' and 'end' lies.
void CastRay(const Eigen::Array2i& begin,
             const Eigen::Array2i& end,
             const std::vector<uint16>& miss_table,
             ProbabilityGrid* const probability_grid)
{
  // For simplicity, we order 'begin' and 'end' by their x coordinate.
  if (begin.x() > end.x()) {
    CastRay(end, begin, miss_table, probability_grid);
    return;
  }

  CHECK_GE(begin.x(), 0);
  CHECK_GE(begin.y(), 0);
  CHECK_GE(end.y(), 0);

  // Special case: We have to draw a vertical line in full pixels, as 'begin'
  // and 'end' have the same full pixel x coordinate.
  if (begin.x() / kSubpixelScale == end.x() / kSubpixelScale)
  {
    Eigen::Array2i current(begin.x() / kSubpixelScale,
                           std::min(begin.y(), end.y()) / kSubpixelScale);
    const int end_y = std::max(begin.y(), end.y()) / kSubpixelScale;
    for (; current.y() <= end_y; ++current.y())
    {
        probability_grid->ApplyLookupTable(current, miss_table);
    }
    return;
  }

  const int64 dx = end.x() - begin.x();
  const int64 dy = end.y() - begin.y();
  const int64 denominator = 2 * kSubpixelScale * dx;

  // The current full pixel coordinates. We begin at 'begin'.
  Eigen::Array2i current = begin / kSubpixelScale;

  // To represent subpixel centers, we use a factor of 2 * 'kSubpixelScale' in
  // the denominator.
  // +-+-+-+ -- 1 = (2 * kSubpixelScale) / (2 * kSubpixelScale)
  // | | | |
  // +-+-+-+
  // | | | |
  // +-+-+-+ -- top edge of first subpixel = 2 / (2 * kSubpixelScale)
  // | | | | -- center of first subpixel = 1 / (2 * kSubpixelScale)
  // +-+-+-+ -- 0 = 0 / (2 * kSubpixelScale)

  // The center of the subpixel part of 'begin.y()' assuming the
  // 'denominator', i.e., sub_y / denominator is in (0, 1).
  int64 sub_y = (2 * (begin.y() % kSubpixelScale) + 1) * dx;

  // The distance from the from 'begin' to the right pixel border, to be divided
  // by 2 * 'kSubpixelScale'.
  const int first_pixel = 2 * kSubpixelScale - 2 * (begin.x() % kSubpixelScale) - 1;
  // The same from the left pixel border to 'end'.
  const int last_pixel = 2 * (end.x() % kSubpixelScale) + 1;

  // The full pixel x coordinate of 'end'.
  const int end_x = std::max(begin.x(), end.x()) / kSubpixelScale;

  // Move from 'begin' to the next pixel border to the right.
  sub_y += dy * first_pixel;
  if (dy > 0) {
    while (true) {
      probability_grid->ApplyLookupTable(current, miss_table);
      while (sub_y > denominator) {
        sub_y -= denominator;
        ++current.y();
        probability_grid->ApplyLookupTable(current, miss_table);
      }
      ++current.x();
      if (sub_y == denominator) {
        sub_y -= denominator;
        ++current.y();
      }
      if (current.x() == end_x) {
        break;
      }
      // Move from one pixel border to the next.
      sub_y += dy * 2 * kSubpixelScale;
    }
    // Move from the pixel border on the right to 'end'.
    sub_y += dy * last_pixel;
    probability_grid->ApplyLookupTable(current, miss_table);
    while (sub_y > denominator) {
      sub_y -= denominator;
      ++current.y();
      probability_grid->ApplyLookupTable(current, miss_table);
    }
    CHECK_NE(sub_y, denominator);
    CHECK_EQ(current.y(), end.y() / kSubpixelScale);
    return;
  }

  // Same for lines non-ascending in y coordinates.
  while (true) {
    probability_grid->ApplyLookupTable(current, miss_table);
    while (sub_y < 0) {
      sub_y += denominator;
      --current.y();
      probability_grid->ApplyLookupTable(current, miss_table);
    }
    ++current.x();
    if (sub_y == 0) {
      sub_y += denominator;
      --current.y();
    }
    if (current.x() == end_x) {
      break;
    }
    sub_y += dy * 2 * kSubpixelScale;
  }
  sub_y += dy * last_pixel;
  probability_grid->ApplyLookupTable(current, miss_table);
  while (sub_y < 0) {
    sub_y += denominator;
    --current.y();
    probability_grid->ApplyLookupTable(current, miss_table);
  }
  CHECK_NE(sub_y, 0);
  CHECK_EQ(current.y(), end.y() / kSubpixelScale);
}

void GrowAsNeeded(const RangeData& range_data,
                  ProbabilityGrid* const probability_grid) {
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
  constexpr float kPadding = 1e-6f;
  for (const Eigen::Vector3f& hit : range_data.returns) {
    bounding_box.extend(hit.head<2>());
  }
  for (const Eigen::Vector3f& miss : range_data.misses) {
    bounding_box.extend(miss.head<2>());
  }
  probability_grid->GrowLimits(bounding_box.min() -
                               kPadding * Eigen::Vector2f::Ones());
  probability_grid->GrowLimits(bounding_box.max() +
                               kPadding * Eigen::Vector2f::Ones());
}


void CastRays(const RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table,
              const bool insert_free_space,
              ProbabilityGrid* const probability_grid) {
  GrowAsNeeded(range_data, probability_grid);

  const MapLimits& limits = probability_grid->limits();
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  const Eigen::Array2i begin =
      superscaled_limits.GetCellIndex(range_data.origin.head<2>());
  // Compute and add the end points.
  std::vector<Eigen::Array2i> ends;
  ends.reserve(range_data.returns.size());
  for (const Eigen::Vector3f& hit : range_data.returns) {
    ends.push_back(superscaled_limits.GetCellIndex(hit.head<2>()));
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
  }

  if (!insert_free_space) {
    return;
  }

  // Now add the misses.
  for (const Eigen::Array2i& end : ends) {
    CastRay(begin, end, miss_table, probability_grid);
  }

  // Finally, compute and add empty rays based on misses in the range data.
  for (const Eigen::Vector3f& missing_echo : range_data.misses) {
    CastRay(begin, superscaled_limits.GetCellIndex(missing_echo.head<2>()),
            miss_table, probability_grid);
  }
}




void Insert(const RangeData& range_data, ProbabilityGrid* const probability_grid)
{

    const std::vector<uint16> hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
                                             Odds(0.9)));
    const std::vector<uint16> miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
                                              Odds(0.1)));

  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  CastRays(range_data, hit_table_, miss_table_, true,
           CHECK_NOTNULL(probability_grid));
  probability_grid->FinishUpdate();
}
#include <opencv2/opencv.hpp>
int main(int argc, char *argv[])
{
    RangeData range_data;
    range_data.returns.emplace_back(-3.5f, 0.5f, 0.f);
    range_data.returns.emplace_back(-2.5f, 1.5f, 0.f);
    range_data.returns.emplace_back(-1.5f, 2.5f, 0.f);
    range_data.returns.emplace_back(-0.5f, 3.5f, 0.f);
    range_data.returns.emplace_back(-13.5f, 1.5f, 0.f);
    range_data.returns.emplace_back(-20.5f, 1.5f, 0.f);
    range_data.returns.emplace_back(-1.5f, 9.5f, 0.f);
    range_data.returns.emplace_back(-5.5f, 7.5f, 0.f);

    range_data.returns.emplace_back(30.5f, 20.5f, 0.f);
    range_data.returns.emplace_back(200.5f, 100.5f, 0.f);
    range_data.returns.emplace_back(1.5f, 2.5f, 0.f);
    range_data.returns.emplace_back(0.5f, 3.5f, 0.f);
    range_data.returns.emplace_back(13.5f, 1.5f, 0.f);
    range_data.returns.emplace_back(20.5f, 1.5f, 0.f);
    range_data.returns.emplace_back(1.5f, 9.5f, 0.f);
    range_data.returns.emplace_back(5.5f, 7.5f, 0.f);

    range_data.returns.emplace_back(30.5f, 20.5f, 0.f);
    range_data.returns.emplace_back(200.5f, 100.5f, 0.f);
    range_data.returns.emplace_back(100.5f, 200.5f, 0.f);
    range_data.returns.emplace_back(60.5f, 30.5f, 0.f);
    range_data.returns.emplace_back(13.5f, 15.f, 0.f);
    range_data.returns.emplace_back(20.5f, 150.f, 0.f);
    range_data.returns.emplace_back(5.f, 49.5f, 0.f);
    range_data.returns.emplace_back(56.5f, 17.5f, 0.f);
    range_data.origin.x() = 70.5f;
    range_data.origin.y() = 72.5f;

    ProbabilityGrid grid(MapLimits(1., Eigen::Vector2d(150., 150.), CellLimits(150, 150)));
    Insert(range_data, &grid);
    constexpr int kUnKnown = 128;
    const CellLimits& celllimits = grid.limits().cell_limits();
    const int width = celllimits.num_x_cells;
    const int height = celllimits.num_y_cells;
    cv::Mat image = cv::Mat(width, height, CV_8UC3);
    for(const Eigen::Array2i& xy_index: XYIndexRangeIterator(grid.limits().cell_limits()))
    {
        CHECK(grid.limits().Contains(xy_index));
        const unsigned char value = grid.IsKnown(xy_index)? RoundToInt((1-grid.GetProbability(xy_index))*255+0):kUnKnown;
        image.at<cv::Vec3b>(xy_index.x(), xy_index.y()) = cv::Vec3b(value, value, value);
    }
        cv::imshow("image", image);
        cv::imwrite("image.png", image);
        cv::waitKey(0);

    return 1;
}
