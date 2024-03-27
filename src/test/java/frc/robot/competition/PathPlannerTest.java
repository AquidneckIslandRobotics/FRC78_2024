package frc.robot.competition;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.classes.BaseDrive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.PoseEstimator;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

public class PathPlannerTest {

  private final Feeder feeder = Mockito.mock(Feeder.class);
  private final Wrist wrist = Mockito.mock(Wrist.class);
  private final Shooter shooter = Mockito.mock(Shooter.class);
  private final Elevator elevator = Mockito.mock(Elevator.class);
  private final Intake intake = Mockito.mock(Intake.class);
  private final Chassis chassis = Mockito.mock(Chassis.class);
  private final PoseEstimator poseEstimator = Mockito.mock(PoseEstimator.class);
  private final BaseDrive baseDrive = Mockito.mock(BaseDrive.class);

  @BeforeEach
  public void setup() {
    Mockito.doAnswer(invocation -> Commands.none()).when(feeder).shoot();
    Mockito.doAnswer(invocation -> Commands.none()).when(feeder).outtake();
    Mockito.doAnswer(invocation -> Commands.none()).when(intake).intakeCommand();
    Mockito.doAnswer(invocation -> Commands.none()).when(intake).outtakeCommand();
    Mockito.doAnswer(invocation -> Commands.none()).when(wrist).setToTargetCmd(Mockito.anyDouble());
    Mockito.doAnswer(invocation -> Commands.none()).when(wrist).stow();
    Mockito.doAnswer(invocation -> Commands.none()).when(elevator).setToTarget(Mockito.anyDouble());
    Mockito.doAnswer(invocation -> Commands.none()).when(shooter).setSpeed(Mockito.anyDouble());

    AutoBuilder.configureHolonomic(
        Pose2d::new,
        pose2d -> {},
        ChassisSpeeds::new,
        chassisSpeeds -> {},
        new HolonomicPathFollowerConfig(0, 0, null),
        () -> false,
        chassis);
    AutoCommands.registerAutoCommands(
        feeder, intake, chassis, wrist, shooter, elevator, poseEstimator, baseDrive);
  }

  @Test
  public void testAutoBuilder() {
    AutoBuilder.buildAutoChooser();
  }
}
