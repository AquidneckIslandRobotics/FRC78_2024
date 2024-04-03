package frc.robot.classes;

public class ProjectileMotionEquations {
  public static double calculateLaunchVelocity(
      double gravity, double distance, double finalHeight, double launchAngle) {

    double nominator = Math.pow(distance, 2) * gravity;
    double denominator =
        distance * Math.sin(2 * launchAngle) - 2 * finalHeight * Math.pow(Math.cos(launchAngle), 2);
    return Math.sqrt(nominator / denominator);
  }
}
