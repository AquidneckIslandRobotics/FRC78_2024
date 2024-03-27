package frc.robot.competition;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.classes.BaseDrive;
import frc.robot.commands.DriveToNote;
import frc.robot.commands.FieldOrientedWithCardinal;
import frc.robot.commands.PickUpNote;
import frc.robot.commands.VarShootPrime;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.PoseEstimator;
import org.littletonrobotics.junction.Logger;

public class AutoCommands {

  static void registerAutoCommands(
      Feeder feeder,
      Intake intake,
      Chassis chassis,
      Wrist wrist,
      Shooter shooter,
      Elevator elevator,
      PoseEstimator poseEstimator,
      BaseDrive baseDrive) {
    NamedCommands.registerCommand("Intake", new PickUpNote(feeder, intake, wrist));
    NamedCommands.registerCommand("StopShooter", shooter.setSpeed(0));

    NamedCommands.registerCommand(
        "StartShooter", shooter.setSpeed(RobotConstants.AUTO_SHOOT_SPEED));
    NamedCommands.registerCommand("Score", Commands.waitSeconds(0.5).andThen(feeder.shoot()));

    NamedCommands.registerCommand(
        "AmpSetUp", wrist.setToTargetCmd(19).alongWith(elevator.setToTarget(13.9)));
    NamedCommands.registerCommand("scoreInAmp", feeder.outtake().withTimeout(2));
    //    NamedCommands.registerCommand("stow", wrist.stow());
    NamedCommands.registerCommand(
        "Target",
        new FieldOrientedWithCardinal(
                chassis,
                poseEstimator,
                () -> {
                  Translation2d target =
                      DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                          ? Constants.RED_SPEAKER_POSE
                          : Constants.BLUE_SPEAKER_POSE;
                  double angle =
                      target
                              .minus(poseEstimator.getFusedPose().getTranslation())
                              .getAngle()
                              .getRadians()
                          + Math.PI;
                  Logger.recordOutput("Aiming angle", angle);
                  //   angle *=
                  //       m_poseEstimator.getEstimatedVel().getY()
                  //           * RobotConstants.SPEAKER_AIM_VEL_COEFF;
                  return angle;
                },
                baseDrive::calculateChassisSpeeds,
                RobotConstants.ROTATION_PID,
                RobotConstants.ROTATION_CONSTRAINTS,
                RobotConstants.ROTATION_FF,
                Units.degreesToRadians(5)) // was 2 changed in b80 for wk4
            .withTimeout(0.5));
    NamedCommands.registerCommand("StopShooter", shooter.setSpeed(0));
    NamedCommands.registerCommand(
        "DriveToNote", new DriveToNote(chassis).raceWith(new PickUpNote(feeder, intake, wrist)));
    NamedCommands.registerCommand("Stow", wrist.stow());
    NamedCommands.registerCommand(
        "VariableShoot",
        new VarShootPrime(
            wrist,
            elevator,
            poseEstimator,
            RobotConstants.SHOOT_POINT,
            () -> shooter.getVelocity() * 60,
            RobotConstants.DISTANCE_RANGE,
            RobotConstants.HEIGHT_LENGTH_COEFF,
            RobotConstants.SHOOTER_RPM_TO_MPS,
            RobotConstants.WRIST_HIGH_LIM));
  }
}
