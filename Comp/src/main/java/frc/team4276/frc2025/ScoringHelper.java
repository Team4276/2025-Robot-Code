package frc.team4276.frc2025;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure.Goal;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.VirtualSubsystem;

public class ScoringHelper extends VirtualSubsystem {
  private final int[] redScoringTable = {
      9,
      8,
      10,
      11,
      0,
      1,
      2,
      3,
      5,
      4,
      7,
      6
  };

  private final int[] blueScoringTable = {
      3,
      2,
      4,
      5,
      6,
      7,
      8,
      9,
      11,
      10,
      1,
      0
  };

  private final int[] redVizTable = {
    6,
    7,
    8,
    9,
    10,
    11,
    0,
    1,
    2,
    3,
    4,
    5,
  };

  private GenericHID keyboard1 = new GenericHID(0);
  private GenericHID buttonBoard1 = new GenericHID(1);
  private GenericHID keyboard2 = new GenericHID(2);

  private final boolean useKeyboard;

  private boolean isRight = true;
  private Goal level = Goal.L1;
  private int side = 0;

  public ScoringHelper(boolean useKeyboard){
    this.useKeyboard = useKeyboard;
  }

  @Override
  public void periodic() {
    if(useKeyboard){
      updateKeyboard();
    } else {
      updateButtonBoard();
    }

    for (int i = 0; i < 12; i++) {
      int currentIndex = AllianceFlipUtil.shouldFlip() ? redVizTable[i] : i;
      SmartDashboard.putBoolean(
          "Comp/SelectedReef" + currentIndex, getSelectedReef() == i);
    }

    for (int i = 0; i < 4; i++) {
      SmartDashboard.putBoolean("Comp/SuperstructureGoal" + i, getSuperstructureGoal().ordinal() - 2 == i);
    }

    Logger.recordOutput("ScoringHelper/SelectedPose", getSelectedPose());
    Logger.recordOutput("ScoringHelper/IsRight", isRight);
    Logger.recordOutput("ScoringHelper/Side", side);
    Logger.recordOutput("ScoringHelper/Level", level);
  }

  private void updateButtonBoard(){    
    // Update Positions
    if (buttonBoard1.getRawButtonPressed(9)) {
      isRight = true;
    } else if (buttonBoard1.getRawButtonPressed(10)) {
      isRight = false;
    }

    if (buttonBoard1.getRawButtonPressed(6)) {
      side = 0;
    } else if (buttonBoard1.getRawButtonPressed(5)) {
      side = 1;
    } else if (buttonBoard1.getRawButtonPressed(4)) {
      side = 2;
    } else if (buttonBoard1.getRawButtonPressed(3)) {
      side = 3;
    } else if (buttonBoard1.getRawButtonPressed(2)) {
      side = 4;
    } else if (buttonBoard1.getRawButtonPressed(1)) {
      side = 5;
    }

    // Update Level
    if (buttonBoard1.getPOV() == 90) {
      level = Goal.L1;
    } else if (buttonBoard1.getPOV() == 270) {
      level = Goal.L2;
    } else if (buttonBoard1.getPOV() == 180) {
      level = Goal.L3;
    } else if (buttonBoard1.getPOV() == 0) {
      level = Goal.L3;
    }

  }

  private void updateKeyboard(){
    // Update Positions
    if (keyboard1.getRawButtonPressed(1)) {
      isRight = true;
    } else if (keyboard1.getRawButtonPressed(2)) {
      isRight = false;
    }

    if (keyboard2.getRawButtonPressed(3)) {
      side = 0;
    } else if (keyboard2.getRawButtonPressed(4)) {
      side = 1;
    } else if (keyboard1.getRawButtonPressed(3)) {
      side = 2;
    } else if (keyboard1.getRawButtonPressed(4)) {
      side = 3;
    } else if (keyboard2.getRawButtonPressed(1)) {
      side = 4;
    } else if (keyboard2.getRawButtonPressed(2)) {
      side = 5;
    }

    // Update Level
    if (buttonBoard1.getRawAxis(1) >= 0.5) {
      level = Goal.L1;
    } else if (buttonBoard1.getRawAxis(0) >= 0.5) {
      level = Goal.L2;
    } else if (buttonBoard1.getRawAxis(1) <= -0.5) {
      level = Goal.L3;
    } else if (buttonBoard1.getRawAxis(0) <= -0.5) {
      level = Goal.L3;
    }
  }

  public Pose2d getSelectedPose() {
    return RobotState.getInstance().getPOIs().reefScoring[getSelectedReef()];
  }  

  private int getSelectedTableIndex(){
    return side * 2 + (isRight ? 0 : 1);
  }

  private int getSelectedReefWithIndex(int index) {
    int[] selectedTable = AllianceFlipUtil.shouldFlip() ? redScoringTable : blueScoringTable;

    return selectedTable[index];
  }

  private int getSelectedReef() {
    return getSelectedReefWithIndex(getSelectedTableIndex());
  }

  public Goal getSuperstructureGoal() {
    return level;
  }
}
