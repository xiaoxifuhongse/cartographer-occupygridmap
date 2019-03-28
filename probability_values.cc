#include "probability_values.h"
#include"make_unique.h"

// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
float SlowValueToBoundedFloat(const uint16 value,
                              const uint16 unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound)
{
    CHECK_GE(value, 0);
    CHECK_LE(value, 32767);
    if (value == unknown_value) return unknown_result;
    const float kScale = (upper_bound - lower_bound) / 32766.f;
    return value * kScale + (lower_bound - kScale);
}
/*
 * in catographer occupygrid map the probability to value by table
 * brief : precompute value to probability
 * hit && miss so while do twice
 * miss: 0.1-0.9   hit 0.1-0.9
 */

std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(const uint16 unknown_value,
                                                                  const float unknown_result,
                                                                  const float lower_bound,
                                                                  const float upper_bound)
{
    auto result = make_unique<std::vector<float>>();
    // Repeat two times, so that both values with and without the update marker
    // can be converted to a probability.
    for (int repeat = 0; repeat != 2; ++repeat)
    {
        for (int value = 0; value != 32768; ++value)
        {
            result->push_back(SlowValueToBoundedFloat(value, unknown_value, unknown_result, lower_bound, upper_bound));
        }
    }
    return result;
}

std::unique_ptr<std::vector<float>> PrecomputeValueToProbability()
{
  return PrecomputeValueToBoundedFloat(kUnknownProbabilityValue,
                                       kMinProbability, kMinProbability,
                                       kMaxProbability);
}

std::unique_ptr<std::vector<float>> PrecomputeValueToCorrespondenceCost() {
  return PrecomputeValueToBoundedFloat(
      kUnknownCorrespondenceValue, kMaxCorrespondenceCost,
      kMinCorrespondenceCost, kMaxCorrespondenceCost);
}


const std::vector<float>* const kValueToProbability = PrecomputeValueToProbability().release();

const std::vector<float>* const kValueToCorrespondenceCost = PrecomputeValueToCorrespondenceCost().release();

std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds)
{
   std::vector<uint16> result;
   result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                    kUpdateMarker);
   for (int cell = 1; cell != 32768; ++cell)
   {
     result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds * Odds((*kValueToProbability)[cell]))) +
                      kUpdateMarker);
   }
   return result;
}

std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(float odds)
{
   std::vector<uint16> result;
   result.push_back(CorrespondenceCostToValue(
                    ProbabilityToCorrespondenceCost(
                        ProbabilityFromOdds(odds))) +
                    kUpdateMarker);
   for (int cell = 1; cell != 32768; ++cell)
   {
       result.push_back(CorrespondenceCostToValue(
                      ProbabilityToCorrespondenceCost(ProbabilityFromOdds(
                      odds * Odds(CorrespondenceCostToProbability((*kValueToCorrespondenceCost)[cell]))))) +
                      kUpdateMarker);
   }
   return result;
}
