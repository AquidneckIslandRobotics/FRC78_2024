// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class PoseEstimator {
  private final Chassis chassis;

  private final SwerveDrivePoseEstimator swervePoseEstimator;
  private Transform2d vel;
  private Pose2d lastPose;
  private final List<PhotonPoseEstimator> visionPoseEstimators;
  private final Pigeon2 pigeon;

  private final Matrix<N3, N1> singleTagStdDevs;
  private final Matrix<N3, N1> multiTagStdDevs;

  private final AprilTagFieldLayout aprilTagFieldLayout;

  public PoseEstimator(
      Chassis chassis,
      SwerveDriveKinematics kinematics,
      AprilTagFieldLayout aprilTagFieldLayout,
      List<PhotonPoseEstimator> visionPoseEstimators,
      Pigeon2 pigeon,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionStdDevs,
      Matrix<N3, N1> singleTagStdDevs,
      Matrix<N3, N1> multiTagStdDevs) {
    this.visionPoseEstimators = visionPoseEstimators;
    this.aprilTagFieldLayout = aprilTagFieldLayout;

    this.chassis = chassis;

    this.singleTagStdDevs = singleTagStdDevs;
    this.multiTagStdDevs = multiTagStdDevs;

    this.pigeon = pigeon;

    swervePoseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            getGyroRot(),
            chassis.getPositions(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs);

    vel = new Transform2d();
    lastPose = new Pose2d();
  }

  public void update() {
    for (int i = 0; i < visionPoseEstimators.size(); i++) {
      Optional<EstimatedRobotPose> estimatedPoseOptional = visionPoseEstimators.get(i).update();

      if (estimatedPoseOptional.isPresent()) {
        EstimatedRobotPose estimatedRobotPose = estimatedPoseOptional.get();
        Pose2d estPose = estimatedRobotPose.estimatedPose.toPose2d();
        // Change our trust in the measurement based on the tags we can see
        Matrix<N3, N1> estStdDevs = getEstimationStdDevs(estPose, estimatedRobotPose.targetsUsed);

        swervePoseEstimator.addVisionMeasurement(
            estPose, estimatedRobotPose.timestampSeconds, estStdDevs);

        Logger.recordOutput("AT Estimate " + i, estPose);
      }
    }

    swervePoseEstimator.update(getGyroRot(), chassis.getPositions());
    Pose2d currentPose = swervePoseEstimator.getEstimatedPosition();
    vel = currentPose.minus(lastPose); // Why is this robot relative?
    vel =
        new Transform2d(
            vel.getTranslation().rotateBy(currentPose.getRotation()), vel.getRotation());

    lastPose = currentPose;

    SmartDashboard.putNumber("gyroYaw", getGyroRot().getDegrees());
    Logger.recordOutput("Estimated Pose", currentPose);
    Logger.recordOutput("Estimated Velocity", getEstimatedVel());
  }

  public Pose2d getFusedPose() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  public Transform2d getEstimatedVel() {
    return vel.div(0.02); // How consistent is this update rate?
  }

  private Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, List<PhotonTrackedTarget> targetsUsed) {
    var estStdDevs = singleTagStdDevs;
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targetsUsed) {
      var tagPose = aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = multiTagStdDevs;
    // Increase std devs based on (average) distance
    // if (numTags == 1 && avgDist > 4)
    //   estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    // else
    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public void resetPose(Pose2d pose) {
    swervePoseEstimator.resetPosition(getGyroRot(), chassis.getPositions(), pose);
  }

  public Rotation2d getGyroRot() {
    return pigeon.getRotation2d();
  }

  public void setGyroRot(double rot) {
    pigeon.setYaw(rot);
  }
}
