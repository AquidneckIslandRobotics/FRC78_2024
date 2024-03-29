package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

/** Measures wheel diameter by spinning the robot */
public class MeasureWheelDiameter extends Command {

  private Rotation2d startingRotation = new Rotation2d();

  private final Chassis chassis;
  private final Pigeon2 pigeon;

  // Slowly accelerate to limit wheel slippage
  private SlewRateLimiter rampRateLimit = new SlewRateLimiter(.5, 0, 0);

  private SwerveModulePosition[] startingPositions;

  private double wheelCircumference;
  private double robotRadius;

  /**
   * @param chassis Chassis subsystem
   * @param pigeon Pigeon used for heading reading
   * @param wheelCircumference Current value used as wheel radius in drive encoder position
   *     conversion factor
   */
  public MeasureWheelDiameter(
      Chassis chassis, Pigeon2 pigeon, double wheelCircumference, double robotRadius) {
    this.chassis = chassis;
    this.pigeon = pigeon;
    this.wheelCircumference = wheelCircumference;
    this.robotRadius = robotRadius;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    startingRotation = pigeon.getRotation2d();
    startingPositions = chassis.getPositions();
  }

  @Override
  public void execute() {
    Rotation2d currentRotation = pigeon.getRotation2d();
    Rotation2d delta = currentRotation.minus(startingRotation);

    double rotationDistance = delta.getRadians() * robotRadius;

    SwerveModulePosition[] currentPositions = chassis.getPositions();

    for (int i = 0; i < currentPositions.length; i++) {
      double rotations =
          (currentPositions[i].distanceMeters - startingPositions[i].distanceMeters)
              / wheelCircumference;

      SmartDashboard.putNumber("Wheel " + i, rotationDistance / rotations / (2 * Math.PI));
    }
    chassis.driveRobotRelative(new ChassisSpeeds(0, 0, rampRateLimit.calculate(1)));
  }
}
