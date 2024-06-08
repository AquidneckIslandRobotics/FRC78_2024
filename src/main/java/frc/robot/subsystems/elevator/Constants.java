package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import frc.robot.classes.Util.RevStatusRates;

/** Constants for the elevator subsystem. */
class Constants {

  // Elevator feedforward constants
  static final double kS = 0.070936;
  static final double kV = 0.79005;
  static final double kA = 0.086892;
  static final double kG = 0.088056;

  // Command loop runs at 50Hz, 20ms period
  static final double kDt = 0.02;

  static final double kP = 4.572;
  static final double kI = 0;
  static final double kD = 0;
  static final double kTolerance = Inches.of(0.1).magnitude();

  static final int leaderMotorId = 11;
  static final int followerMotorId = 12;

  static final Constraints constraints = new Constraints(15, 80);

  static final double kGearRatio = 5 * 5;

  /** Diameter is 1.29 inches */
  static final Measure<Distance> drumRadius = Inches.of(1.29 / 2.0);

  static final double kPositionConversionFactor = drumRadius.magnitude() * 2 * Math.PI / kGearRatio;

  static final boolean kLeaderMotorInverted = false;
  static final boolean kFollowerMotorInverted = true;

  static final RevStatusRates leaderMotorStatusRates = new RevStatusRates(5, 20, 20);
  static final RevStatusRates kFollowerMotorStatusRates = new RevStatusRates(500);

  static final float kSoftLimitForward = 16.4f;

  static final Measure<Mass> kCarriageMass = Pounds.of(18.427);

  static final Measure<Distance> kMinHeight = Inches.of(0);
  static final Measure<Distance> kMaxHeight = Inches.of(17);

  static final Measure<Distance> kSimStartingHeight = Inches.of(5);
}
