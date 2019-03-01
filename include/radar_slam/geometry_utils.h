#ifndef GEOMETRY_UTILS_H_
#define GEOMETRY_UTILS_H_

namespace geometry_utils
{  
  /**
   * Normalizes an angle to [-PI,PI).
   *
   * @param x The input angle to be constrained to the range
   * @return The output constrained angle
   */
  double constrainAngle( double x );

  /**
   *
   * Converts an angle to [-2*PI,2*PI].
   *
   * @param angle The input angle to be converted
   * @return The output converted angle
   */
  double angleConv( double angle );

  /**
   * Computes the difference between successive angles a and b.
   *
   * @param a The first angle, in radians
   * @param b The second angle, in radians
   * @return The difference between angles, in radians
   */
  double angleDiff( double a, double b );

  /**
   * Unwraps an angle using its previous and new values.
   *
   * @param previousAngle Previous unwrapped angle, in radians
   * @param newAngle New wrapped angle, in radians
   * @return The new unwrapped angle, in radians
   */
  double unwrap( double previousAngle, double newAngle );
}

#endif
