package frc.robot.commands;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.Structs.MotionLimits;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.PoseEstimator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AlignToPose extends Command {
  private Chassis chassis;
  private PoseEstimator poseEstimator;
  private Supplier<Transform2d> targetTransform;

  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private ProfiledPIDController thetaController;

  public AlignToPose(
      Chassis chassis,
      Supplier<Transform2d> targetTransform,
      PoseEstimator poseEstimator,
      PIDConstants translationPID,
      PIDConstants thetaPID,
      MotionLimits motionLimits) {
    this.chassis = chassis;
    this.poseEstimator = poseEstimator;
    this.targetTransform = targetTransform;

    xController =
        new ProfiledPIDController(
            translationPID.kP,
            translationPID.kI,
            translationPID.kD,
            new Constraints(motionLimits.maxSpeed, motionLimits.maxAcceleration));
    yController =
        new ProfiledPIDController(
            translationPID.kP,
            translationPID.kI,
            translationPID.kD,
            new Constraints(motionLimits.maxSpeed, motionLimits.maxAcceleration));
    thetaController =
        new ProfiledPIDController(
            thetaPID.kP,
            thetaPID.kI,
            thetaPID.kD,
            new Constraints(motionLimits.maxAngularSpeed, motionLimits.maxAngularAcceleration));

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    Transform2d pose = targetTransform.get();
    Transform2d vel = poseEstimator.getEstimatedFOVel();
    xController.reset(pose.getTranslation().getX(), vel.getX());
    yController.reset(pose.getTranslation().getY(), vel.getY());
    thetaController.reset(pose.getRotation().getRadians(), vel.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Transform2d goalTransform = targetTransform.get();
    Pose2d currentPose = poseEstimator.getFusedPose();

    double xOutput =
        xController.calculate(
            currentPose.getTranslation().getX(), goalTransform.getTranslation().getX());
    double yOutput =
        yController.calculate(
            currentPose.getTranslation().getY(), goalTransform.getTranslation().getY());
    double thetaOutput =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), goalTransform.getRotation().getRadians());

    chassis.driveRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xOutput, yOutput, thetaOutput, currentPose.getRotation()));
    Logger.recordOutput("TargetPose", goalTransform);
  }
}
