import frc.robot.classes.ProjectileMotionEquations;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class ProjectileMotionEquationsTest {

  @Test
  public void testLaunchVelocity() {
    // https://www.omnicalculator.com/physics/projectile-motion
    // Initial height of 1 meter is the same as a final height of -1 meter
    Assertions.assertEquals(
        9.88f,
        ProjectileMotionEquations.calculateLaunchVelocity(9.8, 10, -1, Math.toRadians(55)),
        0.1f);
  }
}
