// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.Structs;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.PoseEstimator;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class FieldOrientedWithCardinal extends Command {
  private final Chassis chassis;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final PoseEstimator poseEstimator;
  private final DoubleSupplier direction;

  private final ProfiledPIDController thetaPID;
  private final SimpleMotorFeedforward thetaFF;
  private ChassisSpeeds speeds;

  /** Creates a new FieldOrientedDrive. */
  public FieldOrientedWithCardinal(
      Chassis chassis,
      PoseEstimator poseEstimator,
      DoubleSupplier direction,
      Supplier<ChassisSpeeds> speedsSupplier,
      PIDConstants cardinalPidConstants,
      Constraints constraints,
      Structs.FFConstants ffConstants) {
    this.chassis = chassis;
    this.poseEstimator = poseEstimator;
    this.speedsSupplier = speedsSupplier;
    this.speeds = speedsSupplier.get();
    this.direction = direction;

    thetaPID =
        new ProfiledPIDController(
            cardinalPidConstants.kP,
            cardinalPidConstants.kI,
            cardinalPidConstants.kD,
            constraints); // TODO tune
    thetaPID.enableContinuousInput(-Math.PI, Math.PI);
    thetaFF =
        new SimpleMotorFeedforward(ffConstants.kS, ffConstants.kV, ffConstants.kA); // TODO tune

    addRequirements(chassis);

    setName("FieldOrientedDriveWithCardinal");
  }

  @Override
  public void execute() {
    speeds = speedsSupplier.get();

    thetaPID.setGoal(direction.getAsDouble());

    double cardinalRotSpeed =
        thetaPID.calculate(poseEstimator.getFusedPose().getRotation().getRadians());
    speeds.omegaRadiansPerSecond = cardinalRotSpeed;

    chassis.driveRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, poseEstimator.getFusedPose().getRotation()));
  }
}
