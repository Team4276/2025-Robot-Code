package frc.team4276.util.feedforwards;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FourbarFeedForward {
  // Constants
  private double kS; // Volts
  private final double kV; // Volts * s / rad

  private final double kGearRatio;
  private final double kStallTorque;
  private final int kMotorAmnt;
  private double kEfficiency;

  /*
   * NOTE:
   * Angles are in standard position.
   * Motor Leg is on the left and support leg is on the right.
   */

  private final double kBottomLength;
  private final double kMotorLegLength;
  private final double kTopLength;
  private final double kSupportLegLength;

  // Kg
  private final double kMotorLegMass;
  private final double kTopMass;
  private final double kSupportLegMass;

  /*
   * Lay the line between the rotating points on the X AXIS
   * (if y value = 0 then Com is on the line between the two points of rotation)
   * The STATIC POINT should be on the origin.
   */
  private final Translation2d kMotorToCom;
  private final Translation2d kMotorLegToTopCom;
  private final Translation2d kSupportToCom;

  // Dynamics

  // Angles on the INSIDE of the quadrilateral (always positive)
  private double motor_inside_angle_;
  private double motor_leg_to_top_inside_angle_;
  private double support_inside_angle_;
  private double support_leg_to_top_inside_angle_;

  // Angles relevant for us
  private double bottom_to_motor_leg_radians_;
  private double bottom_to_top_radians_;
  private double bottom_to_support_leg_radians_; // Supplement of the inside angle
  /*
   * for bottom_to_support_leg_radians_ imagine the bottom support extends
   * to the RIGHT of the support leg and measure from there
   */

  private Translation2d motor_to_leg_com_;
  private Translation2d support_to_leg_com_;
  private Translation2d motor_leg_to_top_com_;

  // private Translation2d total_motor_to_com_;

  public static class FourbarFeedForwardConstants {
    public double kS;

    public double kMotorFreeSpeedRpm;
    public double kGearRatio;
    public double kStallTorque;
    public int kMotorAmnt;
    public double kEfficiency;

    public double kBottomLength;
    public double kMotorLegLength;
    public double kTopLength;
    public double kSupportLegLength;

    public double kMotorLegMass;
    public double kTopMass;
    public double kSupportLegMass;

    public Translation2d kMotorToCom;
    public Translation2d kMotorLegToTopCom;
    public Translation2d kSupportToCom;
  }

  public FourbarFeedForward(FourbarFeedForwardConstants constants) {
    this.kS = constants.kS;
    // nominal voltage / max speed RPM converted to radians per second
    this.kV = (12 / (constants.kMotorFreeSpeedRpm / constants.kGearRatio)) * 60 / (2 * Math.PI);

    this.kGearRatio = constants.kGearRatio;
    this.kStallTorque = constants.kStallTorque;
    this.kMotorAmnt = constants.kMotorAmnt;
    this.kEfficiency = constants.kEfficiency;

    this.kBottomLength = constants.kBottomLength;
    this.kMotorLegLength = constants.kMotorLegLength;
    this.kTopLength = constants.kTopLength;
    this.kSupportLegLength = constants.kSupportLegLength;

    this.kMotorLegMass = constants.kMotorLegMass;
    this.kTopMass = constants.kTopMass;
    this.kSupportLegMass = constants.kSupportLegMass;

    this.kMotorToCom = constants.kMotorToCom;
    this.kMotorLegToTopCom = constants.kMotorLegToTopCom;
    this.kSupportToCom = constants.kSupportToCom;
  }

  public void setkS(double static_voltage) {
    kS = static_voltage;
  }

  public void setEfficiency(double efficiency) {
    kEfficiency = efficiency;
  }

  public double calculate(double posRad, double velRad, double accelRad) {
    updateInsideAngles(posRad);
    updateRelevantAngles();
    updateComs();

    double gravity_voltage = calcGravityVoltage(posRad);
    double velocity_voltage = kV * velRad;

    SmartDashboard.putNumber("Debug/Fourbar Gravity Voltage", gravity_voltage);
    SmartDashboard.putNumber("Debug/Fourbar Velocity Voltage", velocity_voltage);
    return gravity_voltage + (kS * Math.signum(velRad)) + velocity_voltage;
  }

  // private double calcAccelerationVoltage(){

  // // total COM distance * total mass * efficiency * nominal voltage
  // return (0) * (kMotorLegMass + kTopMass + kSupportLegMass) * kEfficiency * 12
  // / (kStallTorque * kMotorAmnt * kGearRatio);
  // }

  private double calcGravityVoltage(double position_setpoint) {
    // desired torque * nominal voltage / max torque
    return calcGravityTorque() * kEfficiency * 12 / (kStallTorque * kMotorAmnt * kGearRatio);
  }

  private void updateInsideAngles(double position) {
    motor_inside_angle_ = position;

    double top_left_to_bottom_right =
        LoCLength(kBottomLength, kMotorLegLength, motor_inside_angle_);

    support_leg_to_top_inside_angle_ =
        LoCAngle(kTopLength, kSupportLegLength, top_left_to_bottom_right);

    motor_leg_to_top_inside_angle_ =
        LoSAngle(top_left_to_bottom_right, motor_inside_angle_, kBottomLength)
            + LoSAngle(
                top_left_to_bottom_right, support_leg_to_top_inside_angle_, kSupportLegLength);

    support_inside_angle_ =
        LoSAngle(top_left_to_bottom_right, motor_inside_angle_, kMotorLegLength)
            + LoSAngle(top_left_to_bottom_right, support_leg_to_top_inside_angle_, kTopLength);
  }

  public double[] getInsideAngles(double position) {
    double bot_to_motor = position;
    double top_left_to_bot_right = LoCLength(kBottomLength, kMotorLegLength, bot_to_motor);
    double top_to_support = LoCAngle(kTopLength, kSupportLegLength, top_left_to_bot_right);
    double motor_to_top =
        LoSAngle(top_left_to_bot_right, bot_to_motor, kBottomLength)
            + LoSAngle(top_left_to_bot_right, top_to_support, kSupportLegLength);
    double support_to_bot =
        LoSAngle(top_left_to_bot_right, bot_to_motor, kMotorLegLength)
            + LoSAngle(top_left_to_bot_right, top_to_support, kTopLength);

    return new double[] {motor_to_top, top_to_support, support_to_bot};
  }

  private void updateRelevantAngles() {
    bottom_to_motor_leg_radians_ = motor_inside_angle_;

    bottom_to_support_leg_radians_ = Math.PI - support_inside_angle_;

    bottom_to_top_radians_ =
        motor_leg_to_top_inside_angle_ + bottom_to_motor_leg_radians_ - Math.PI;
  }

  private void updateComs() {
    motor_to_leg_com_ = kMotorToCom.rotateBy(Rotation2d.fromRadians(bottom_to_motor_leg_radians_));

    motor_leg_to_top_com_ =
        kMotorLegToTopCom.rotateBy(Rotation2d.fromRadians(bottom_to_top_radians_));

    support_to_leg_com_ =
        kSupportToCom.rotateBy(Rotation2d.fromRadians(bottom_to_support_leg_radians_));

    // total_motor_to_com_ = new Translation2d();

  }

  private double calcGravityTorque() {
    double transfered_force =
        calcSupportLegTorque()
            / (kSupportLegLength * Math.sin(support_leg_to_top_inside_angle_ % (Math.PI / 2)));

    double transfered_torque =
        transfered_force
            * Math.sin((motor_inside_angle_ - bottom_to_top_radians_) % (Math.PI / 2))
            * kMotorLegLength;

    return calcMotorLegTorque() + transfered_torque;
  }

  private double calcMotorLegTorque() {
    return (kMotorLegMass * 9.81 * motor_to_leg_com_.getX())
        + (calcShooterToMotorLeg() * Math.cos(bottom_to_motor_leg_radians_));
  }

  private double calcSupportLegTorque() {
    return (kSupportLegMass * 9.81 * support_to_leg_com_.getX())
        + (calcShooterToSupportLeg() * Math.cos(bottom_to_support_leg_radians_));
  }

  /** Returns force of top on motor leg */
  private double calcShooterToMotorLeg() {
    double gravity_torque =
        kTopMass
            * 9.81
            * ((kTopLength * Math.cos(bottom_to_top_radians_)) - motor_leg_to_top_com_.getX());

    return gravity_torque / (kTopLength * Math.cos(bottom_to_top_radians_));
  }

  /** Returns force of top on support leg */
  private double calcShooterToSupportLeg() {
    double gravity_torque = kTopMass * 9.81 * motor_leg_to_top_com_.getX();

    return gravity_torque / (kTopLength * Math.cos(bottom_to_top_radians_));
  }

  /**
   * @param side1
   * @param side2
   * @param opposite_angle radians
   * @return side length opposite of given angle
   */
  public static double LoCLength(double side1, double side2, double opposite_angle) {
    return Math.sqrt(
        Math.pow(side1, 2) + Math.pow(side2, 2) - (2 * side1 * side2 * Math.cos(opposite_angle)));
  }

  /**
   * @param side1
   * @param side2
   * @param opposite_side opposite side
   * @return radians
   */
  public static double LoCAngle(double side1, double side2, double opposite_side) {
    return Math.acos(
        (Math.pow(side1, 2) + Math.pow(side2, 2) - Math.pow(opposite_side, 2))
            / (2 * side1 * side2));
  }

  /**
   * @param side1
   * @param angle1 radians
   * @param opposite_angle radians
   * @return
   */
  public static double LoSLength(double side1, double angle1, double opposite_angle) {
    return side1 * Math.sin(opposite_angle) / Math.sin(angle1);
  }

  /**
   * @param side1
   * @param angle1 radians
   * @param opposite_side
   * @return radians
   */
  public static double LoSAngle(double side1, double angle1, double opposite_side) {
    return Math.asin(Math.sin(angle1) * opposite_side / side1);
  }
}
