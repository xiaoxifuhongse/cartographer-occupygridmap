#ifndef PROBABILITY_VALUES_H
#define PROBABILITY_VALUES_H
#include <cmath>
#include <vector>
#include "math1.h"
#include "port.h"
#include "glog/logging.h"

inline uint16 BoundedFloatToValue(const float float_value,
                                  const float lower_bound,
                                  const float upper_bound)
{
  const int value =
      RoundToInt(
          (Clamp(float_value, lower_bound, upper_bound) - lower_bound) *
          (32766.f / (upper_bound - lower_bound))) + 1;
  // DCHECK for performance.
  DCHECK_GE(value, 1);
  DCHECK_LE(value, 32767);
  return value;
}

// odds = p/(1-p)
inline float Odds(float probability)
{
    return probability / (1.f - probability);
}
// p = o/(o+1)
inline float ProbabilityFromOdds(const float odds) {
  return odds / (odds + 1.f);
}

// probability ---> correspondenceCost = 1-p

inline float ProbabilityToCorrespondenceCost(const float probability) {
  return 1.f - probability;
}
// correspondenctCost ----> probability = 1-c

inline float CorrespondenceCostToProbability(const float correspondence_cost) {
  return 1.f - correspondence_cost;
}
constexpr float kMinProbability = 0.1f;         // min probability
constexpr float kMaxProbability = 1.f - kMinProbability; // max probability
constexpr float kMinCorrespondenceCost = 1.f - kMaxProbability; // minc = 1-maxp
constexpr float kMaxCorrespondenceCost = 1.f - kMinProbability; // maxc = 1-minp


inline float ClampProbability(const float probability)
{
    return Clamp(probability, kMinProbability, kMaxProbability);

}

inline float ClampCorrespondenceCost(const float correspondence_cost)
{
    return Clamp(correspondence_cost, kMinCorrespondenceCost,kMaxCorrespondenceCost);
}

constexpr uint16 kUnknownProbabilityValue = 0;
constexpr uint16 kUnknownCorrespondenceValue = kUnknownProbabilityValue;
constexpr uint16 kUpdateMarker = 1u << 15; // 32768

// converts a correspondence_cost to uint16 int the [1, 32767] range
inline uint16  CorrespondenceCostToValue(const float correspondence_cost)
{
        return BoundedFloatToValue(correspondence_cost, kMinCorrespondenceCost, kMaxCorrespondenceCost);
}

//converts a probability to a uint16 in the range[1, 32767] range
inline uint16 ProbabilityToValue(const float probability)
{
    return BoundedFloatToValue(probability, kMinProbability, kMaxProbability);
}

extern const std::vector<float>* const kValueToProbability;
extern const std::vector<float>* const kValueToCorrespondenceCost;

// converts a uint16  to a probability in the range[kMinProbability, kMaxProbability]
// loop up table
inline float ValueToProbability(const uint16 value)
{
    return (*kValueToProbability)[value];
}


// Converts a uint16 (which may or may not have the update marker set) to a
// correspondence cost in the range [kMinCorrespondenceCost,
// kMaxCorrespondenceCost].
inline float ValueToCorrespondenceCost(const uint16 value)
{
  return (*kValueToCorrespondenceCost)[value];
}

// probability value -> correspondencecost value [uint16]
// method : value(uint16) -> probability(float)->correspondececost(float)->correspondencecostvalue(uint16)
//
inline uint16 ProbabilityValueToCorrespondenceCostValue(uint16 probability_value)
{
    if (probability_value == kUnknownProbabilityValue)
    {
        return kUnknownCorrespondenceValue;
    }
    bool update_carry = false;
    if (probability_value > kUpdateMarker)
    {
        probability_value -= kUpdateMarker;
        update_carry = true;
    }
    uint16 result = CorrespondenceCostToValue( //              correspondcost->uint16
                    ProbabilityToCorrespondenceCost(         // probability->correspondcost
                    ValueToProbability(probability_value))); // value->probability
    if (update_carry) result += kUpdateMarker;
    return result;
}

inline uint16 CorrespondenceCostValueToProbabilityValue(uint16 correspondence_cost_value)
{
    if (correspondence_cost_value == kUnknownCorrespondenceValue)
    {

        return kUnknownProbabilityValue;
    }
    bool update_carry = false;
    if (correspondence_cost_value > kUpdateMarker)
    {
        correspondence_cost_value -= kUpdateMarker;
        update_carry = true;
    }
    uint16 result = ProbabilityToValue(
                    CorrespondenceCostToProbability(
                    ValueToCorrespondenceCost(correspondence_cost_value)));
    if (update_carry) result += kUpdateMarker;
    return result;
}

std::vector<uint16> ComputeLookupTableToApplyOdds(float odds);
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(float odds);


#endif // PROBABILITY_VALUES_H
