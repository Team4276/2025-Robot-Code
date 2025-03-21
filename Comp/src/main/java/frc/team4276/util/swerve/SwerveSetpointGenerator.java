package frc.team4276.util.swerve;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.team4276.frc2025.subsystems.drive.DriveConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Swerve setpoint generator based on a version created by FRC team 254.
 *
 * <p>Modified b/c beta release is a bit bwoken Removes torque calcs
 *
 * <p>Takes a prior setpoint, a desired setpoint, and outputs a new setpoint that respects all the
 * kinematic constraints on module rotation and wheel velocity/torque, as well as preventing any
 * forces acting on a module's wheel from exceeding the force of friction.
 */
public class SwerveSetpointGenerator {
  private static final double kEpsilon = 1E-8;
  private static final int MAX_STEER_ITERATIONS = 8;
  private static final int MAX_DRIVE_ITERATIONS = 10;

  private final RobotConfig config;
  private final double maxSteerVelocityRadsPerSec;
  private final double brownoutVoltage;

  /**
   * Create a new swerve setpoint generator
   *
   * @param config The robot configuration
   * @param maxSteerVelocityRadsPerSec The maximum rotation velocity of a swerve module, in radians
   *     per second
   */
  public SwerveSetpointGenerator(RobotConfig config, double maxSteerVelocityRadsPerSec) {
    this.config = config;
    this.maxSteerVelocityRadsPerSec = maxSteerVelocityRadsPerSec;
    this.brownoutVoltage = RobotController.getBrownoutVoltage();
  }

  /**
   * Create a new swerve setpoint generator
   *
   * @param config The robot configuration
   * @param maxSteerVelocity The maximum rotation velocity of a swerve module
   */
  public SwerveSetpointGenerator(RobotConfig config, AngularVelocity maxSteerVelocity) {
    this(config, maxSteerVelocity.in(RadiansPerSecond));
  }

  /**
   * Generate a new setpoint with explicit battery voltage. Note: Do not discretize ChassisSpeeds
   * passed into or returned from this method. This method will discretize the speeds for you.
   *
   * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous
   *     iteration setpoint instead of the actual measured/estimated kinematic state.
   * @param desiredStateRobotRelative The desired state of motion, such as from the driver sticks or
   *     a path following algorithm.
   * @param dt The loop time.
   * @param inputVoltage The input voltage of the drive motor controllers, in volts. This can also
   *     be a static nominal voltage if you do not want the setpoint generator to react to changes
   *     in input voltage. If the given voltage is NaN, it will be assumed to be 12v. The input
   *     voltage will be clamped to a minimum of the robot controller's brownout voltage.
   * @return A Setpoint object that satisfies all the kinematic/friction limits while converging to
   *     desiredState quickly.
   */
  public SwerveSetpoint generateSetpoint(
      final SwerveSetpoint prevSetpoint,
      ChassisSpeeds desiredStateRobotRelative,
      double dt,
      double inputVoltage) {
    if (Double.isNaN(inputVoltage)) {
      inputVoltage = 12.0;
    } else {
      inputVoltage = Math.max(inputVoltage, brownoutVoltage);
    }

    SwerveModuleState[] desiredModuleStates =
        config.toSwerveModuleStates(desiredStateRobotRelative);
    // Make sure desiredState respects velocity limits.
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredModuleStates, config.moduleConfig.maxDriveVelocityMPS);
    desiredStateRobotRelative = config.toChassisSpeeds(desiredModuleStates);

    // Special case: desiredState is a complete stop. In this case, module angle is
    // arbitrary, so
    // just use the previous angle.
    boolean need_to_steer = true;
    if (epsilonEquals(desiredStateRobotRelative, new ChassisSpeeds())) {
      need_to_steer = false;
      for (int m = 0; m < config.numModules; m++) {
        desiredModuleStates[m].angle = prevSetpoint.moduleStates()[m].angle;
        desiredModuleStates[m].speedMetersPerSecond = 0.0;
      }
    }

    // For each module, compute local Vx and Vy vectors.
    double[] prev_vx = new double[config.numModules];
    double[] prev_vy = new double[config.numModules];
    Rotation2d[] prev_heading = new Rotation2d[config.numModules];
    double[] desired_vx = new double[config.numModules];
    double[] desired_vy = new double[config.numModules];
    Rotation2d[] desired_heading = new Rotation2d[config.numModules];
    boolean all_modules_should_flip = true;
    for (int m = 0; m < config.numModules; m++) {
      prev_vx[m] =
          prevSetpoint.moduleStates()[m].angle.getCos()
              * prevSetpoint.moduleStates()[m].speedMetersPerSecond;
      prev_vy[m] =
          prevSetpoint.moduleStates()[m].angle.getSin()
              * prevSetpoint.moduleStates()[m].speedMetersPerSecond;
      prev_heading[m] = prevSetpoint.moduleStates()[m].angle;
      if (prevSetpoint.moduleStates()[m].speedMetersPerSecond < 0.0) {
        prev_heading[m] = prev_heading[m].rotateBy(Rotation2d.k180deg);
      }
      desired_vx[m] =
          desiredModuleStates[m].angle.getCos() * desiredModuleStates[m].speedMetersPerSecond;
      desired_vy[m] =
          desiredModuleStates[m].angle.getSin() * desiredModuleStates[m].speedMetersPerSecond;
      desired_heading[m] = desiredModuleStates[m].angle;
      if (desiredModuleStates[m].speedMetersPerSecond < 0.0) {
        desired_heading[m] = desired_heading[m].rotateBy(Rotation2d.k180deg);
      }
      if (all_modules_should_flip) {
        double required_rotation_rad =
            Math.abs(prev_heading[m].unaryMinus().rotateBy(desired_heading[m]).getRadians());
        if (required_rotation_rad < Math.PI / 2.0) {
          all_modules_should_flip = false;
        }
      }
    }
    if (all_modules_should_flip
        && !epsilonEquals(prevSetpoint.robotRelativeSpeeds(), new ChassisSpeeds())
        && !epsilonEquals(desiredStateRobotRelative, new ChassisSpeeds())) {
      // It will (likely) be faster to stop the robot, rotate the modules in place to
      // the complement
      // of the desired angle, and accelerate again.
      return generateSetpoint(prevSetpoint, new ChassisSpeeds(), dt, inputVoltage);
    }

    // Compute the deltas between start and goal. We can then interpolate from the
    // start state to
    // the goal state; then find the amount we can move from start towards goal in
    // this cycle such
    // that no kinematic limit is exceeded.
    double dx =
        desiredStateRobotRelative.vxMetersPerSecond
            - prevSetpoint.robotRelativeSpeeds().vxMetersPerSecond;
    double dy =
        desiredStateRobotRelative.vyMetersPerSecond
            - prevSetpoint.robotRelativeSpeeds().vyMetersPerSecond;
    double dtheta =
        desiredStateRobotRelative.omegaRadiansPerSecond
            - prevSetpoint.robotRelativeSpeeds().omegaRadiansPerSecond;

    // 's' interpolates between start and goal. At 0, we are at prevState and at 1,
    // we are at
    // desiredState.
    double min_s = 1.0;

    // In cases where an individual module is stopped, we want to remember the right
    // steering angle
    // to command (since inverse kinematics doesn't care about angle, we can be
    // opportunistically
    // lazy).
    List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(config.numModules);
    // Enforce steering velocity limits. We do this by taking the derivative of
    // steering angle at
    // the current angle, and then backing out the maximum interpolant between start
    // and goal
    // states. We remember the minimum across all modules, since that is the active
    // constraint.
    for (int m = 0; m < config.numModules; m++) {
      if (!need_to_steer) {
        overrideSteering.add(Optional.of(prevSetpoint.moduleStates()[m].angle));
        continue;
      }
      overrideSteering.add(Optional.empty());

      double max_theta_step = dt * maxSteerVelocityRadsPerSec;

      if (epsilonEquals(prevSetpoint.moduleStates()[m].speedMetersPerSecond, 0.0)) {
        // If module is stopped, we know that we will need to move straight to the final
        // steering
        // angle, so limit based purely on rotation in place.
        if (epsilonEquals(desiredModuleStates[m].speedMetersPerSecond, 0.0)) {
          // Goal angle doesn't matter. Just leave module at its current angle.
          overrideSteering.set(m, Optional.of(prevSetpoint.moduleStates()[m].angle));
          continue;
        }

        var necessaryRotation =
            prevSetpoint
                .moduleStates()[m]
                .angle
                .unaryMinus()
                .rotateBy(desiredModuleStates[m].angle);
        if (flipHeading(necessaryRotation)) {
          necessaryRotation = necessaryRotation.rotateBy(Rotation2d.kPi);
        }

        // getRadians() bounds to +/- Pi.
        final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / max_theta_step;

        if (numStepsNeeded <= 1.0) {
          // Steer directly to goal angle.
          overrideSteering.set(m, Optional.of(desiredModuleStates[m].angle));
        } else {
          // Adjust steering by max_theta_step.
          overrideSteering.set(
              m,
              Optional.of(
                  prevSetpoint.moduleStates()[m].angle.rotateBy(
                      Rotation2d.fromRadians(
                          Math.signum(necessaryRotation.getRadians()) * max_theta_step))));
          min_s = 0.0;
        }
        continue;
      }
      if (min_s == 0.0) {
        // s can't get any lower. Save some CPU.
        continue;
      }

      double s =
          findSteeringMaxS(
              prev_vx[m],
              prev_vy[m],
              prev_heading[m].getRadians(),
              desired_vx[m],
              desired_vy[m],
              desired_heading[m].getRadians(),
              max_theta_step);
      min_s = Math.min(min_s, s);
    }

    // Enforce drive wheel acceleration limits
    double maxVelStep = Math.abs(DriveConstants.maxAccel * dt);
    for (int m = 0; m < config.numModules; m++) {
      if (min_s == 0.0) {
        // No need to carry on.
        break;
      }

      double vx_min_s =
          min_s == 1.0 ? desired_vx[m] : (desired_vx[m] - prev_vx[m]) * min_s + prev_vx[m];
      double vy_min_s =
          min_s == 1.0 ? desired_vy[m] : (desired_vy[m] - prev_vy[m]) * min_s + prev_vy[m];
      // Find the max s for this drive wheel. Search on the interval between 0 and
      // min_s, because we
      // already know we can't go faster than that.
      double s =
          findDriveMaxS(
              prev_vx[m],
              prev_vy[m],
              Math.hypot(prev_vx[m], prev_vy[m]),
              vx_min_s,
              vy_min_s,
              Math.hypot(vx_min_s, vy_min_s),
              maxVelStep);
      min_s = Math.min(min_s, s);
    }

    ChassisSpeeds retSpeeds =
        new ChassisSpeeds(
            prevSetpoint.robotRelativeSpeeds().vxMetersPerSecond + min_s * dx,
            prevSetpoint.robotRelativeSpeeds().vyMetersPerSecond + min_s * dy,
            prevSetpoint.robotRelativeSpeeds().omegaRadiansPerSecond + min_s * dtheta);
    retSpeeds = ChassisSpeeds.discretize(retSpeeds, dt);

    var retStates = config.toSwerveModuleStates(retSpeeds);
    double[] accelFF = new double[config.numModules];
    double[] dummyStuff = new double[] {0.0, 0.0, 0.0, 0.0};
    for (int m = 0; m < config.numModules; m++) {
      final var maybeOverride = overrideSteering.get(m);
      if (maybeOverride.isPresent()) {
        var override = maybeOverride.get();
        if (flipHeading(retStates[m].angle.unaryMinus().rotateBy(override))) {
          retStates[m].speedMetersPerSecond *= -1.0;
        }
        retStates[m].angle = override;
      }
      final var deltaRotation =
          prevSetpoint.moduleStates()[m].angle.unaryMinus().rotateBy(retStates[m].angle);
      if (flipHeading(deltaRotation)) {
        retStates[m].angle = retStates[m].angle.rotateBy(Rotation2d.k180deg);
        retStates[m].speedMetersPerSecond *= -1.0;
      }

      accelFF[m] =
          (retStates[m].speedMetersPerSecond - prevSetpoint.moduleStates()[m].speedMetersPerSecond)
              / dt;
    }

    return new SwerveSetpoint(
        retSpeeds,
        retStates,
        new DriveFeedforwards(accelFF, dummyStuff, dummyStuff, dummyStuff, dummyStuff));
  }

  /**
   * Generate a new setpoint with explicit battery voltage. Note: Do not discretize ChassisSpeeds
   * passed into or returned from this method. This method will discretize the speeds for you.
   *
   * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous
   *     iteration setpoint instead of the actual measured/estimated kinematic state.
   * @param desiredStateRobotRelative The desired state of motion, such as from the driver sticks or
   *     a path following algorithm.
   * @param dt The loop time.
   * @param inputVoltage The input voltage of the drive motor controllers, in volts. This can also
   *     be a static nominal voltage if you do not want the setpoint generator to react to changes
   *     in input voltage. If the given voltage is NaN, it will be assumed to be 12v. The input
   *     voltage will be clamped to a minimum of the robot controller's brownout voltage.
   * @return A Setpoint object that satisfies all the kinematic/friction limits while converging to
   *     desiredState quickly.
   */
  public SwerveSetpoint generateSetpoint(
      final SwerveSetpoint prevSetpoint,
      ChassisSpeeds desiredStateRobotRelative,
      Time dt,
      Voltage inputVoltage) {
    return generateSetpoint(
        prevSetpoint, desiredStateRobotRelative, dt.in(Seconds), inputVoltage.in(Volts));
  }

  /**
   * Generate a new setpoint. Note: Do not discretize ChassisSpeeds passed into or returned from
   * this method. This method will discretize the speeds for you.
   *
   * <p>Note: This method will automatically use the current robot controller input voltage.
   *
   * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous
   *     iteration setpoint instead of the actual measured/estimated kinematic state.
   * @param desiredStateRobotRelative The desired state of motion, such as from the driver sticks or
   *     a path following algorithm.
   * @param dt The loop time.
   * @return A Setpoint object that satisfies all the kinematic/friction limits while converging to
   *     desiredState quickly.
   */
  public SwerveSetpoint generateSetpoint(
      SwerveSetpoint prevSetpoint, ChassisSpeeds desiredStateRobotRelative, double dt) {
    return generateSetpoint(
        prevSetpoint, desiredStateRobotRelative, dt, RobotController.getInputVoltage());
  }

  /**
   * Generate a new setpoint. Note: Do not discretize ChassisSpeeds passed into or returned from
   * this method. This method will discretize the speeds for you.
   *
   * <p>Note: This method will automatically use the current robot controller input voltage.
   *
   * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous
   *     iteration setpoint instead of the actual measured/estimated kinematic state.
   * @param desiredStateRobotRelative The desired state of motion, such as from the driver sticks or
   *     a path following algorithm.
   * @param dt The loop time.
   * @return A Setpoint object that satisfies all the kinematic/friction limits while converging to
   *     desiredState quickly.
   */
  public SwerveSetpoint generateSetpoint(
      SwerveSetpoint prevSetpoint, ChassisSpeeds desiredStateRobotRelative, Time dt) {
    return generateSetpoint(
        prevSetpoint,
        desiredStateRobotRelative,
        dt.in(Seconds),
        RobotController.getBatteryVoltage());
  }

  /**
   * Check if it would be faster to go to the opposite of the goal heading (and reverse drive
   * direction).
   *
   * @param prevToGoal The rotation from the previous state to the goal state (i.e.
   *     prev.inverse().rotateBy(goal)).
   * @return True if the shortest path to achieve this rotation involves flipping the drive
   *     direction.
   */
  private static boolean flipHeading(Rotation2d prevToGoal) {
    return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
  }

  private static double unwrapAngle(double ref, double angle) {
    double diff = angle - ref;
    if (diff > Math.PI) {
      return angle - 2.0 * Math.PI;
    } else if (diff < -Math.PI) {
      return angle + 2.0 * Math.PI;
    } else {
      return angle;
    }
  }

  @FunctionalInterface
  private interface Function2d {
    double f(double x, double y);
  }

  /**
   * Find the root of the generic 2D parametric function 'func' using the regula falsi technique.
   * This is a pretty naive way to do root finding, but it's usually faster than simple bisection
   * while being robust in ways that e.g. the Newton-Raphson method isn't.
   *
   * @param func The Function2d to take the root of.
   * @param x_0 x value of the lower bracket.
   * @param y_0 y value of the lower bracket.
   * @param f_0 value of 'func' at x_0, y_0 (passed in by caller to save a call to 'func' during
   *     recursion)
   * @param x_1 x value of the upper bracket.
   * @param y_1 y value of the upper bracket.
   * @param f_1 value of 'func' at x_1, y_1 (passed in by caller to save a call to 'func' during
   *     recursion)
   * @param iterations_left Number of iterations of root finding left.
   * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the
   *     (approximate) root.
   */
  private static double findRoot(
      Function2d func,
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      int iterations_left) {
    var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));

    if (iterations_left < 0 || epsilonEquals(f_0, f_1)) {
      return s_guess;
    }

    var x_guess = (x_1 - x_0) * s_guess + x_0;
    var y_guess = (y_1 - y_0) * s_guess + y_0;
    var f_guess = func.f(x_guess, y_guess);
    if (Math.signum(f_0) == Math.signum(f_guess)) {
      // 0 and guess on same side of root, so use upper bracket.
      return s_guess
          + (1.0 - s_guess)
              * findRoot(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
    } else {
      // Use lower bracket.
      return s_guess
          * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
    }
  }

  private static double findSteeringMaxS(
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      double max_deviation) {
    f_1 = unwrapAngle(f_0, f_1);
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_deviation) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_deviation;
    Function2d func = (x, y) -> unwrapAngle(f_0, Math.atan2(y, x)) - offset;
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, MAX_STEER_ITERATIONS);
  }

  private static double findDriveMaxS(
      double x_0, double y_0, double f_0, double x_1, double y_1, double f_1, double max_vel_step) {
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_vel_step) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_vel_step;
    Function2d func = (x, y) -> Math.hypot(x, y) - offset;
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, MAX_DRIVE_ITERATIONS);
  }

  private static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  private static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  private static boolean epsilonEquals(ChassisSpeeds s1, ChassisSpeeds s2) {
    return epsilonEquals(s1.vxMetersPerSecond, s2.vxMetersPerSecond)
        && epsilonEquals(s1.vyMetersPerSecond, s2.vyMetersPerSecond)
        && epsilonEquals(s1.omegaRadiansPerSecond, s2.omegaRadiansPerSecond);
  }
}
