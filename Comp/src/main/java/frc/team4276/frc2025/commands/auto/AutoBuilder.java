package frc.team4276.frc2025.commands.auto;

import static frc.team4276.frc2025.commands.auto.AutoCommands.*;
import static frc.team4276.util.path.ChoreoUtil.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.subsystems.arm.Arm;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.feedtake.Feedtake;
import frc.team4276.frc2025.subsystems.flywheels.Flywheels;

public class AutoBuilder {
  private final Drive drive;
  private final Arm arm;
  private final Feedtake feedtake;
  private final Flywheels flywheels;

  public AutoBuilder(Drive drive, Arm arm, Feedtake feedtake, Flywheels flywheels) {
    this.drive = drive;
    this.arm = arm;
    this.feedtake = feedtake;
    this.flywheels = flywheels;
  }

  public Command CrescendoTest() {
    var traj1 = getSwerveTrajectory("Demo");

    return Commands.runOnce(
            () -> RobotState.getInstance().resetPose(traj1.getInitialPose(false).get()))
        .andThen(
            Commands.race(
                Commands.runEnd(
                    () -> arm.setGoal(Arm.Goal.SPEAKER), () -> arm.setGoal(Arm.Goal.STOW)),
                followTrajectory(drive, traj1)));
  }
}
