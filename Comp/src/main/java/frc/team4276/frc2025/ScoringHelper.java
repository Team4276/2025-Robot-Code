package frc.team4276.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.team4276.frc2025.field.FieldConstants.Reef;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure.Goal;
import frc.team4276.util.VikXboxController;
import frc.team4276.util.drivers.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class ScoringHelper extends VirtualSubsystem {
  private final CommandGenericHID buttonBoard;
  private final CommandGenericHID keyboard;
  private final VikXboxController xbox;
  private final boolean useKeyboard;

  private boolean isRight = false;
  private Goal level = Goal.L1;
  private int side = 0;

  public ScoringHelper(
      CommandGenericHID buttonBoard,
      CommandGenericHID keyboard,
      VikXboxController xbox,
      boolean useKeyboard) {
    this.buttonBoard = buttonBoard;
    this.keyboard = keyboard;
    this.xbox = xbox;
    this.useKeyboard = useKeyboard;
  }

  @Override
  public void periodic() {
    if (useKeyboard && Constants.getMode() == Constants.Mode.SIM) {
      updateKeyboard();
    } else {
      updateXbox(); // redundancy
      updateButtonBoard();
    }

    for (int i = 0; i < 12; i++) {
      SmartDashboard.putBoolean(
          "Comp/ReefScoring/" + Reef.values()[i].toString(), getSelectedReef() == Reef.values()[i]);
    }

    // SmartDashboard.putBoolean("Comp/ReefScoring/L4", getSuperstructureGoal() ==
    // Goal.L4);
    SmartDashboard.putBoolean("Comp/ReefScoring/L3", getSuperstructureGoal() == Goal.L3);
    SmartDashboard.putBoolean("Comp/ReefScoring/L2", getSuperstructureGoal() == Goal.L2);
    SmartDashboard.putBoolean("Comp/ReefScoring/L1", getSuperstructureGoal() == Goal.L1);

    Logger.recordOutput("ScoringHelper/SelectedAlignPose", getSelectedAlignPose());
    Logger.recordOutput("ScoringHelper/SelectedScorePose", getSelectedScorePose());
    Logger.recordOutput("ScoringHelper/IsRight", isRight);
    Logger.recordOutput("ScoringHelper/Side", side);
    Logger.recordOutput("ScoringHelper/Level", level);
    Logger.recordOutput("ScoringHelper/SelectedReef", getSelectedReef());
  }

  private void updateButtonBoard() {
    // Update Positions
    if (buttonBoard.getHID().getRawButtonPressed(9)) {
      isRight = false;
    } else if (buttonBoard.getHID().getRawButtonPressed(10)) {
      isRight = true;
    }

    if (buttonBoard.getHID().getRawButtonPressed(4)) {
      side = 0;
    } else if (buttonBoard.getHID().getRawButtonPressed(3)) {
      side = 1;
    } else if (buttonBoard.getHID().getRawButtonPressed(2)) {
      side = 2;
    } else if (buttonBoard.getHID().getRawButtonPressed(1)) {
      side = 3;
    } else if (buttonBoard.getHID().getRawButtonPressed(6)) {
      side = 4;
    } else if (buttonBoard.getHID().getRawButtonPressed(5)) {
      side = 5;
    }

    // Update Level
    if (buttonBoard.getHID().getPOV() == 90) {
      level = Goal.L1;
    } else if (buttonBoard.getHID().getPOV() == 270) {
      level = Goal.L2;
    } else if (buttonBoard.getHID().getPOV() == 180) {
      level = Goal.L3;
    } else if (buttonBoard.getHID().getPOV() == 0) {
      level = Goal.L3;
    }
  }

  private void updateXbox() {
    // Update Positions
    if (xbox.getHID().getLeftBumperButton()) {
      isRight = false;
    } else if (xbox.getHID().getRightBumperButton()) {
      isRight = true;
    }

    if (xbox.getHID().getPOV() == 180) {
      side = 0;
    } else if (xbox.getHID().getPOV() == 135) {
      side = 1;
    } else if (xbox.getHID().getPOV() == 45) {
      side = 2;
    } else if (xbox.getHID().getPOV() == 0) {
      side = 3;
    } else if (xbox.getHID().getPOV() == 315) {
      side = 4;
    } else if (xbox.getHID().getPOV() == 225) {
      side = 5;
    }

    // Update Level
    if (xbox.getHID().getAButton()) {
      level = Goal.L1;
    } else if (xbox.getHID().getBButton()) {
      level = Goal.L2;
    } else if (xbox.getHID().getYButton()) {
      level = Goal.L3;
    }
    // else if (xbox.getHID().getPOV() == 0) {
    //   level = Goal.L3;
    // }
  }

  private void updateKeyboard() {
    // Update Positions
    if (keyboard.getHID().getRawButtonPressed(1)) {
      isRight = false;
    } else if (keyboard.getHID().getRawButtonPressed(2)) {
      isRight = true;
    }

    if (keyboard.getHID().getRawButtonPressed(3)) {
      side = 0;
    } else if (keyboard.getHID().getRawButtonPressed(4)) {
      side = 1;
    } else if (keyboard.getHID().getRawButtonPressed(5)) {
      side = 2;
    } else if (keyboard.getHID().getRawButtonPressed(6)) {
      side = 3;
    } else if (keyboard.getHID().getRawButtonPressed(7)) {
      side = 4;
    } else if (keyboard.getHID().getRawButtonPressed(8)) {
      side = 5;
    }

    // Update Level
    if (keyboard.getHID().getPOV() == 90) {
      level = Goal.L1;
    } else if (keyboard.getHID().getPOV() == 180) {
      level = Goal.L2;
    } else if (keyboard.getHID().getPOV() == 270) {
      level = Goal.L3;
    } else if (keyboard.getHID().getPOV() == 0) {
      level = Goal.L3;
    }
  }

  public Goal getSuperstructureGoal() {
    return level;
  }

  public Pose2d getSelectedAlignPose() {
    return getSelectedReef().getAlign();
  }

  public Pose2d getSelectedScorePose() {
    return getSelectedReef().getScore();
  }

  public Reef getSelectedReef() {
    if (side > 1 && side < 5) {
      return Reef.values()[(side * 2) + (isRight ? 0 : 1)];

    } else {
      return Reef.values()[(side * 2) + (isRight ? 1 : 0)];
    }
  }
}
