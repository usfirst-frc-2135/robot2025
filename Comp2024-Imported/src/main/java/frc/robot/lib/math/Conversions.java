//
// Conversions - useful unit conversion utilities
//
package frc.robot.lib.math;

/****************************************************************************
 * 
 * Conversion class - gearbox calculations
 */
public class Conversions
{
  //
  // Gearbox degrees to/from rotations
  //

  /**
   * Convert input motor rotations thru gearbox to output shaft degrees
   * 
   * @param rotations
   *          Input Shaft Rotations
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism
   * @return Output Shaft Degrees of Mechanism
   */
  public static double rotationsToOutputDegrees(double rotations, double gearRatio)
  {
    return rotations * (360.0 / gearRatio);
  }

  /**
   * Convert output shaft degrees backward thru gearbox to input motor rotations
   * 
   * @param degrees
   *          Output Shaft Degrees of Mechanism
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism
   * @return Input Shaft Rotations
   */
  public static double degreesToInputRotations(double degrees, double gearRatio)
  {
    return degrees / (360.0 / gearRatio);
  }

  //
  // Gearbox rotations to/from rotations
  //

  /**
   * Convert input motor rotations thru gearbox to output shaft rotationss
   * 
   * @param rotations
   *          Input Shaft Rotations
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism
   * @return Output Shaft Rotations of Mechanism
   */
  public static double rotationsToOutputRotations(double rotations, double gearRatio)
  {
    return rotations / gearRatio;
  }

  /**
   * Convert output shaft rotations backward thru gearbox to input motor rotations
   * 
   * @param rotations
   *          Output Shaft Rotations of Mechanism
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism
   * @return Input Shaft Rotations
   */
  public static double rotationsToInputRotations(double rotations, double gearRatio)
  {
    return rotations * gearRatio;
  }

  //
  // Gearbox degrees to/from radians
  //

  /**
   * Convert input motor rotations thru gearbox to output shaft radians
   * 
   * @param rotations
   *          Input Shaft Rotations
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism
   * @return Output Shaft Radians of Mechanism
   */
  public static double rotationsToOutputRadians(double rotations, double gearRatio)
  {
    return rotations * (2 * Math.PI / gearRatio);
  }

  /**
   * Convert output shaft radians backward thru gearbox to input motor rotations
   * 
   * @param radians
   *          Output Shaft Radians of Mechanism
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism
   * @return Input Shaft Rotations
   */
  public static double radiansToInputRotations(double radians, double gearRatio)
  {
    return radians / (2 * Math.PI / gearRatio);
  }

  //
  // Velocity calculations
  //

  /**
   * Calculate velocity (m/s) through a gearbox given input motor rotations per second
   * 
   * @param rotationsPerSecond
   *          Motor rotations per second
   * @param circumference
   *          Circumference of Wheel
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism (set to 1 for Motor RPM)
   * @return Velocity meters per second
   */
  public static double RPSToMPS(double rotationsPerSecond, double circumference, double gearRatio)
  {
    double wheelRPM = rotationsPerSecond / gearRatio;
    return wheelRPM * circumference;
  }

  /**
   * Calculate input motor rotations per second through a gearbox given a velocity (m/s)
   * 
   * @param velocity
   *          Velocity meters per second
   * @param circumference
   *          Circumference of Wheel
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism (set to 1 for Motor RPM)
   * @return Motor rotations per second
   */
  public static double MPSToRPS(double velocity, double circumference, double gearRatio)
  {
    double wheelRPM = velocity / circumference;
    return wheelRPM * gearRatio;
  }

  //
  // Distance calculations
  //

  /**
   * Calculate distance (meters) through a gearbox given input motor rotations
   * 
   * @param rotations
   *          Motor rotations
   * @param circumference
   *          Circumference of Wheel
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism (set to 1 for Motor RPM)
   * @return Distance in meters
   */
  public static double rotationsToMeters(double rotations, double circumference, double gearRatio)
  {
    return rotations * (circumference / gearRatio);
  }

  /**
   * Calculate input motor rotations through a gearbox given a distance (meters)
   * 
   * @param distance
   *          Distance in meters
   * @param circumference
   *          Circumference of Wheel
   * @param gearRatio
   *          Gear Ratio between Motor and Mechanism (set to 1 for Motor RPM)
   * @return Motor rotations
   */
  public static double metersToRotations(double distance, double circumference, double gearRatio)
  {
    return distance / (circumference / gearRatio);
  }

  //
  // WINCH calculatons
  //

  /**
   * Calculate distance (inches) through a winch gearbox given input motor rotations
   * 
   * @param rotations
   *          Input motor rotations
   * @param rolloutRatio
   *          Winch rollout ratio
   * @return Linear winch distance
   */
  public static double rotationsToWinchInches(double rotations, double rolloutRatio)
  {
    return rotations * rolloutRatio;
  }

  /**
   * Calculate input motor rotations through a winch gearbox given a rope distance (inches)
   * 
   * @param inches
   *          Linear winch distance
   * @param rolloutRatio
   *          Winch rollout ratio
   * @return Input motor rotations
   */
  public static double inchesToWinchRotations(double inches, double rolloutRatio)
  {
    return inches / rolloutRatio;
  }

}
