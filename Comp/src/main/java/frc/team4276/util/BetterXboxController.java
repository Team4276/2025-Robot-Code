package frc.team4276.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class BetterXboxController extends CommandXboxController {
  private double JOYSTICK_DEADBAND = 0.1;
  private double TRIGGER_DEADBAND = 0.5;

  public BetterXboxController(int port) {
    super(port);
  }

  public BetterXboxController(int port, double deadband) {
    super(port);
    this.JOYSTICK_DEADBAND = deadband;
  }

  public void setDeadband(double deadband) {
    this.JOYSTICK_DEADBAND = deadband;
  }

  public class JoystickOutput {
    public final double x;
    public final double y;

    public JoystickOutput(double x, double y) {
      this.x = x;
      this.y = y;
    }

    public JoystickOutput() {
      this.x = 0.0;
      this.y = 0.0;
    }
  }

  public JoystickOutput getRightWithDeadband() {
    return Math.hypot(getRightX(), getRightY()) < JOYSTICK_DEADBAND ? new JoystickOutput() : getRight();
  }

  public JoystickOutput getRight() {
    return new JoystickOutput(getRightX(), getRightY());
  }

  public JoystickOutput getLeftWithDeadband() {
    return Math.hypot(getLeftX(), getLeftY()) < JOYSTICK_DEADBAND ? new JoystickOutput() : getLeft();
  }

  public JoystickOutput getLeft() {
    return new JoystickOutput(getLeftX(), getLeftY());
  }

  public boolean getPOVUP() {
    return getHID().getPOV() == 0;
  }

  public boolean getPOVRIGHT() {
    return getHID().getPOV() == 90;
  }

  public boolean getPOVDOWN() {
    return getHID().getPOV() == 180;
  }

  public boolean getPOVLEFT() {
    return getHID().getPOV() == 270;
  }

  public boolean getLT() {
    return getLeftTriggerAxis() > TRIGGER_DEADBAND;
  }

  public boolean getRT() {
    return getRightTriggerAxis() > TRIGGER_DEADBAND;
  }
}
