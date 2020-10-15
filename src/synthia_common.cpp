#include "synthia_to_rosbag/synthia_common.h"

namespace synthia {

Transformation interpolateTransformations(const Transformation& left,
                                          const Transformation& right,
                                          double t) {
  Transformation output;
  // Linearly interpolate the position between the two.
  output.getPosition() = left.getPosition() * (1 - t) + right.getPosition() * t;

  // slerp the rotation between the two.
  output.getRotation().toImplementation() =
      left.getRotation().toImplementation().slerp(
          t, right.getRotation().toImplementation());

  return output;
}

}  // namespace synthia
