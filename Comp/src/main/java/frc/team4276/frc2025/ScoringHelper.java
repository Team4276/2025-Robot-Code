package frc.team4276.frc2025;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import frc.team4276.util.VirtualSubsystem;

public class ScoringHelper extends VirtualSubsystem {
  private final int[] redScoringTable = {
      0,
      1,
      2,
      3,
      5,
      4,
      7,
      6,
      9,
      8,
      10,
      11
  };

  private final int[] blueScoringTable = {
    6,
    7,
    8,
    9,
    11,
    10,
    1,
    0,
    3,
    2,
    4,
    5,
  };

  private GenericHID buttonBoard = new GenericHID(2);

  private boolean isRight = true;
  private int level = 0;
  private int side = 0;

  @Override
  public void periodic() {
    // Update Positions
    if (buttonBoard.getRawButtonPressed(0)) {
      isRight = true;
    } else if (buttonBoard.getRawButtonPressed(0)) {
      isRight = false;
    }

    if (buttonBoard.getRawButtonPressed(0)) {
      side = 0;
    } else if (buttonBoard.getRawButtonPressed(0)) {
      side = 1;
    } else if (buttonBoard.getRawButtonPressed(0)) {
      side = 2;
    } else if (buttonBoard.getRawButtonPressed(0)) {
      side = 3;
    } else if (buttonBoard.getRawButtonPressed(0)) {
      side = 5;
    }

    // Update Level
    if (buttonBoard.getRawButtonPressed(0)) {
      level = 0;
    } else if (buttonBoard.getRawButtonPressed(0)) {
      level = 1;
    } else if (buttonBoard.getRawButtonPressed(0)) {
      level = 2;
    } else if (buttonBoard.getRawButtonPressed(0)) {
      level = 3;
    }
  }

  public Pose2d getSelectedPose() {
    return RobotState.getInstance().getPOIs().reefScoring[getSelectedReef()];
  }

  public int getSelectedReef() {
    int[] selectedTable = ChoreoAllianceFlipUtil.shouldFlip() ? redScoringTable : blueScoringTable;

    return selectedTable[side * 2 + (isRight ? 1 : 0)];
  }

  public int getSelectedLevel() {
    return level;
  }

  public boolean isRight() {
    return isRight;
  }

}
