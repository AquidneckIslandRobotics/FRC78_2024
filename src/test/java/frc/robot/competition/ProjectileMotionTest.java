package frc.robot.competition;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.util.Units;
import frc.robot.classes.ProjectileMotionsEquations;
import org.junit.jupiter.api.Test;

public class ProjectileMotionTest {

  @Test
  public void testLaunchAngle() {
    assertEquals(
        Units.degreesToRadians(21.02),
        ProjectileMotionsEquations.calculateLaunchAngleForTargetAndVelocity(15, 1.5, 3),
        1);
  }
}
