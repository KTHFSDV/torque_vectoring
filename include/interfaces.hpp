#pragma once

#include <concepts>

#include "types.hpp"

namespace tv {

template <typename T>
concept ForceEstimator = requires(T estimator, const VehicleState& state) {
    { estimator.estimateForcesImpl(state) } -> std::same_as<ForceVector>;
};

template <typename T>
concept YawMomentGenerator = requires(T generator, const VehicleState& state,
                                      const ForceVector& forces, const SteeringCommand& cmd) {
    { generator.computeImpl(state, forces, cmd) } -> std::same_as<YawMomentCommand>;
};

template <typename T>
concept TorqueAllocator = requires(T allocator, const VehicleState& state,
                                   const YawMomentCommand& cmd, const ForceVector& forces) {
    { allocator.allocateImpl(state, cmd, forces) } -> std::same_as<WheelTorques>;
};

template <typename Derived>
class IYawController {
   public:
    YawMomentCommand compute(const VehicleState& s, const ForceVector& forces,
                             const SteeringCommand& cmd) {
        return static_cast<Derived*>(this)->computeImpl(s, forces, cmd);
    }
};

template <typename Derived>
class IForceEstimator {
   public:
    ForceVector estimateForces(const VehicleState& s) {
        return static_cast<Derived*>(this)->estimateForcesImpl(s);
    }
};

template <typename Derived>
class ITorqueAllocator {
   public:
    WheelTorques allocate(const VehicleState& state, const YawMomentCommand& cmd,
                          const ForceVector& forces) {
        return static_cast<Derived*>(this)->allocateImpl(state, cmd, forces);
    }
};

}  // namespace tv
