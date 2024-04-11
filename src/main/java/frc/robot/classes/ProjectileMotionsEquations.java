package frc.robot.classes;

public class ProjectileMotionsEquations {
  // Source? It was revealed to me by a wise tree in a dream
  // JK this https://en.wikipedia.org/wiki/Projectile_motion
  public static double calculateLaunchAngleForTargetAndVelocity(
      double velocity, double height, double distance) {
    double sqrt =
        velocity * velocity * velocity * velocity
            - (9.81 * ((9.81 * distance * distance) + (2 * height * velocity * velocity)));
    double numerator = (velocity * velocity) - Math.sqrt(sqrt);
    double denominator = 9.81 * distance;

    return Math.atan(numerator / denominator);
  }
}
