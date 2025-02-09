// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.team4276.frc2025;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team4276.frc2025.AutoSelector.AutoQuestion;
import frc.team4276.frc2025.AutoSelector.AutoQuestionResponse;
import frc.team4276.frc2025.Constants.Mode;
import frc.team4276.frc2025.Constants.RobotType;
import frc.team4276.frc2025.commands.DriveCommands;
import frc.team4276.frc2025.commands.FeedForwardCharacterization;
import frc.team4276.frc2025.commands.WheelRadiusCharacterization;
import frc.team4276.frc2025.commands.auto.AutoBuilder;
import frc.team4276.frc2025.subsystems.arm.Arm;
import frc.team4276.frc2025.subsystems.arm.ArmIO;
import frc.team4276.frc2025.subsystems.arm.ArmIOSparkMax;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.drive.GyroIO;
import frc.team4276.frc2025.subsystems.drive.GyroIOADIS;
import frc.team4276.frc2025.subsystems.drive.ModuleIO;
import frc.team4276.frc2025.subsystems.drive.ModuleIOSim;
import frc.team4276.frc2025.subsystems.drive.ModuleIOSpark;
import frc.team4276.frc2025.subsystems.roller.Roller;
import frc.team4276.frc2025.subsystems.roller.RollerIO;
import frc.team4276.frc2025.subsystems.roller.RollerIOSparkMax;
import frc.team4276.frc2025.subsystems.superstructure.RollerSensorsIO;
import frc.team4276.frc2025.subsystems.superstructure.RollerSensorsIOHardware;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;
import frc.team4276.frc2025.subsystems.superstructure.elevator.Elevator;
import frc.team4276.frc2025.subsystems.superstructure.elevator.ElevatorIO;
import frc.team4276.frc2025.subsystems.superstructure.elevator.ElevatorIOSparkMax;
import frc.team4276.frc2025.subsystems.superstructure.endeffector.EndEffector;
import frc.team4276.frc2025.subsystems.superstructure.endeffector.EndEffectorIO;
import frc.team4276.frc2025.subsystems.superstructure.endeffector.EndEffectorIOSparkMax;
import frc.team4276.frc2025.subsystems.vision.Vision;
import frc.team4276.frc2025.subsystems.vision.VisionConstants;
import frc.team4276.frc2025.subsystems.vision.VisionIO;
import frc.team4276.frc2025.subsystems.vision.VisionIOPhotonVision;
import frc.team4276.frc2025.subsystems.vision.VisionIOPhotonVisionSim;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.BetterXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Superstructure superstructure;
  private Arm arm;
  private Roller roller;
  private Vision vision;

  private AutoBuilder autoBuilder;

  // Controller
  private boolean useKeyboard = true;
  private boolean isDemo = true;

  private final BetterXboxController driver = new BetterXboxController(0);
  private final CommandGenericHID buttonBoard = new CommandGenericHID(1);
  private final CommandGenericHID keyboard = new CommandGenericHID(2);

  private final ScoringHelper scoringHelper = new ScoringHelper(buttonBoard, keyboard, useKeyboard);

  private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected = new Alert("Operator controller disconnected (port 1).",
      AlertType.kWarning);

  // Overrides
  private final DigitalInput elevatorCoastOverride = new DigitalInput(Ports.ELEVATOR_COAST_OVERRIDE);
  private final DigitalInput armCoastOverride = new DigitalInput(Ports.ARM_COAST_OVERRIDE);

  private boolean disableTranslationAutoAlign = true;
  private boolean disableVisionSim = true;

  // Dashboard inputs
  private final AutoSelector autoSelector = new AutoSelector();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getMode()) {
        case REAL -> {
          // Real robot, instantiate hardware IO implementations
          drive = new Drive(
              new GyroIOADIS(),
              new ModuleIOSpark(0),
              new ModuleIOSpark(1),
              new ModuleIOSpark(2),
              new ModuleIOSpark(3));
          superstructure = new Superstructure(
              new Elevator(new ElevatorIOSparkMax()),
              new EndEffector(new EndEffectorIOSparkMax(Ports.ENDEFFECTOR_LEFT,
                  Ports.ENDEFFECTOR_RIGHT,
                  40, false,
                  true)),
              new RollerSensorsIOHardware());
          arm = new Arm(new ArmIOSparkMax());
          roller = new Roller(new RollerIOSparkMax(Ports.ALGAE_INTAKE_ROLLER, 40,
              false, true));
          // vision = new Vision(
          // RobotState.getInstance()::addVisionMeasurement,
          // new VisionIOPhotonVision(
          // VisionConstants.camera0Name, VisionConstants.robotToCamera0),
          // new VisionIOPhotonVision(
          // VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        }

        case SIM -> {
          // Sim robot, instantiate physics sim IO implementations
          drive = new Drive(
              new GyroIO() {
              },
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());
          superstructure = new Superstructure(
              new Elevator(new ElevatorIO() {
              }),
              new EndEffector(new EndEffectorIO() {
              }),
              new RollerSensorsIO() {
              });
          arm = new Arm(new ArmIO() {
          });
          roller = new Roller(new RollerIO() {
          });
          if (disableVisionSim) {
            vision = new Vision(
                RobotState.getInstance()::addVisionMeasurement);
          } else {
            vision = new Vision(
                RobotState.getInstance()::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    RobotState.getInstance()::getEstimatedPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name,
                    VisionConstants.robotToCamera1,
                    RobotState.getInstance()::getEstimatedPose));

          }
        }

        default -> {
          // Replayed robot, disable IO implementations
          drive = new Drive(
              new GyroIO() {
              },
              new ModuleIO() {
              },
              new ModuleIO() {
              },
              new ModuleIO() {
              },
              new ModuleIO() {
              });
          superstructure = new Superstructure(
              new Elevator(new ElevatorIO() {
              }),
              new EndEffector(new EndEffectorIO() {
              }),
              new RollerSensorsIO() {
              });
          arm = new Arm(new ArmIO() {
          });
          roller = new Roller(new RollerIO() {
          });
          vision = new Vision(
              RobotState.getInstance()::addVisionMeasurement,
              new VisionIO() {
              },
              new VisionIO() {
              });

        }
      }
    }

    configureOverrides();
    configureAutos();
    if (Constants.isTuning) {
      configureTuningRoutines();

    }
    configureButtonBindings();

    // Peace and quiet
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureOverrides() {
    superstructure.setCoastOverride(elevatorCoastOverride::get);
    arm.setCoastOverride(armCoastOverride::get);

  }

  private void configureAutos() {
    autoBuilder = new AutoBuilder(drive, superstructure, autoSelector);

    // Set up auto routines
    autoSelector.addRoutine("Test 1 Traj", () -> autoBuilder.testTraj("z_BoxTest"));
    autoSelector.addRoutine("RP Auto",
        List.of(
            new AutoQuestion("Is Processor Side?", List.of(
                AutoQuestionResponse.YES,
                AutoQuestionResponse.NO))),
        () -> autoBuilder.rpAuto());
    autoSelector.addRoutine("Taxi Wall",
        List.of(
            new AutoQuestion("Is Processor Side?", List.of(
                AutoQuestionResponse.YES,
                AutoQuestionResponse.NO))),
        () -> autoBuilder.taxiAuto("t_WALL"));
    autoSelector.addRoutine("Max 5 Coral",
        List.of(
            new AutoQuestion("Is Processor Side?", List.of(
                AutoQuestionResponse.YES,
                AutoQuestionResponse.NO))),
        () -> autoBuilder.max5Coral());
    autoSelector.addRoutine("Safe 5 Coral", List.of(
        new AutoQuestion("Is Processor Side?", List.of(
            AutoQuestionResponse.YES,
            AutoQuestionResponse.NO))),
        () -> autoBuilder.safe5Coal());

  }

  private void configureTuningRoutines() {
    // Set up SysId routines
    autoSelector.addRoutine(
        "Drive Wheel Radius Characterization", () -> new WheelRadiusCharacterization(drive));
    autoSelector.addRoutine(
        "Drive Simple FF Characterization",
        () -> new FeedForwardCharacterization(
            drive, drive::runCharacterization, drive::getFFCharacterizationVelocity));
    autoSelector.addRoutine(
        "Drive SysId (Quasistatic Forward)",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoSelector.addRoutine(
        "Drive SysId (Quasistatic Reverse)",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoSelector.addRoutine(
        "Drive SysId (Dynamic Forward)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoSelector.addRoutine(
        "Drive SysId (Dynamic Reverse)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoSelector.addRoutine(
        "Arm Simple FF Characterization",
        () -> new FeedForwardCharacterization(
            arm, arm::runCharacterization, arm::getFFCharacterizationVelocity));
    autoSelector.addRoutine(
        "Elevator Simple FF Characterization",
        () -> new FeedForwardCharacterization(
            superstructure, superstructure::acceptCharacterizationInput,
            superstructure::getFFCharacterizationVelocity));
    autoSelector.addRoutine(
        "(Reverse) Arm Simple FF Characterization",
        () -> new FeedForwardCharacterization(
            arm, arm::runCharacterization, arm::getFFCharacterizationVelocity, true));
    autoSelector.addRoutine(
        "(Reverse) Elevator Simple FF Characterization",
        () -> new FeedForwardCharacterization(
            superstructure, superstructure::acceptCharacterizationInput,
            superstructure::getFFCharacterizationVelocity, true));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    if (useKeyboard && Constants.getMode() == Mode.SIM) {
      configureKeyBoardBindings();

    } else if (isDemo) {
      configureDemoBindings();

    } else {
      configureControllerBindings();

    }
  }

  private void configureDemoBindings() {
    // drive.setDefaultCommand(
    // drive.run(
    // () -> drive.feedTeleopInput(
    // -driver.getLeftWithDeadband().y,
    // -driver.getLeftWithDeadband().x,
    // -driver.getRightWithDeadband().x)));

    drive.setDefaultCommand(
        drive.run(
            () -> drive.feedTeleopInput(
                0.0,
                0.0,
                0.0)));

    // Reset gyro to 0° when A button is pressed
    driver
        .a()
        .onTrue(
            Commands.runOnce(
                () -> RobotState.getInstance()
                    .resetPose(
                        new Pose2d(
                            RobotState.getInstance().getEstimatedPose().getTranslation(),
                            AllianceFlipUtil.apply(Rotation2d.kZero))),
                drive)
                .ignoringDisable(false));

    // superstructure.setDefaultCommand(
    // superstructure.run(() -> superstructure.acceptCharacterizationInput(
    // 4.0 * (driver.getRightTriggerAxis() - driver.getLeftTriggerAxis()))));

    arm.setDefaultCommand(
        arm.run(() -> arm.runCharacterization(
            4.0 * (driver.getRightTriggerAxis() - driver.getLeftTriggerAxis()))));

    driver
        .leftBumper()
        .whileTrue(
            roller.setGoalCommand(Roller.Goal.INTAKE));

    driver
        .y()
        .whileTrue(
            roller.setGoalCommand(Roller.Goal.SCORE));

    driver
        .rightBumper()
        .whileTrue(
            Commands.runOnce(
                () -> superstructure.setEndEffectorGoal(EndEffector.Goal.SCORE)))
        .whileFalse(
            Commands.runOnce(
                () -> superstructure.setEndEffectorGoal(EndEffector.Goal.IDLE)));
  }

  private void configureKeyBoardBindings() {
    drive.setDefaultCommand(
        drive.run(
            () -> drive.feedTeleopInput(
                -keyboard.getRawAxis(0),
                -keyboard.getRawAxis(1),
                -keyboard.getRawAxis(2))));

    // Coral Intake Triggers
    keyboard
        .button(9)
        .whileTrue(
            superstructure.setGoalCommand(Superstructure.Goal.INTAKE)
                .alongWith(
                    DriveCommands.headingAlignCommand(drive,
                        () -> AllianceFlipUtil.apply(Rotation2d.fromDegrees(55.0)))));

    keyboard
        .button(10)
        .whileTrue(
            superstructure.setGoalCommand(Superstructure.Goal.INTAKE)
                .alongWith(
                    DriveCommands.headingAlignCommand(drive,
                        () -> AllianceFlipUtil.apply(Rotation2d.fromDegrees(305.0)))));

    // Algae Intake Trigger
    keyboard
        .button(11)
        .whileTrue(
            arm.setGoalCommand(Arm.Goal.INTAKE).alongWith(
                roller.setGoalCommand(Roller.Goal.INTAKE)))
        .whileFalse(
            Commands.either(
                arm.setGoalCommand(Arm.Goal.HOLD),
                arm.setGoalCommand(Arm.Goal.STOW),
                () -> roller.hasGamePiece()));

    // Coral Scoring Triggers
    keyboard
        .button(12)
        .whileTrue(
            Commands.sequence(
                DriveCommands.driveToPoseCommand(drive, scoringHelper::getSelectedAlignPose)
                    .until(() -> drive.isAutoAligned()
                        && (Constants.getType() == RobotType.SIMBOT ? true : superstructure.atGoal())),
                DriveCommands.driveToPoseCommand(drive, scoringHelper::getSelectedScorePose)
                    .alongWith(superstructure.setGoalCommand(() -> scoringHelper.getSuperstructureGoal())))
                .alongWith(
                    Commands
                        .waitUntil(() -> drive.disableBackVision())
                        .andThen(() -> vision.setEnableCamera(1,
                            false)))
                .finallyDo(() -> vision.setEnableCamera(1, true)));

    keyboard
        .button(13)
        .onTrue(superstructure.scoreCommand(false));

    // Algae Scoring Triggers
    keyboard
        .button(15)
        .whileTrue(roller.setGoalCommand(Roller.Goal.SCORE));

    keyboard
        .button(14)
        .whileTrue(
            Commands.either(
                superstructure.scoreCommand(true),
                arm.setGoalCommand(Arm.Goal.SCORE).alongWith(
                    Commands.startEnd(
                        () -> drive.setHeadingGoal(
                            () -> AllianceFlipUtil.apply(Rotation2d.kCCW_90deg)),
                        drive::clearHeadingGoal)),
                keyboard.button(12)));

  }

  private void configureControllerBindings() {
    drive.setDefaultCommand(
        drive.run(
            () -> drive.feedTeleopInput(
                -driver.getLeftWithDeadband().y,
                -driver.getLeftWithDeadband().x,
                -driver.getRightWithDeadband().x)));

    // Reset gyro to 0° when A button is pressed
    driver
        .a()
        .onTrue(
            Commands.runOnce(
                () -> RobotState.getInstance()
                    .resetPose(
                        new Pose2d(
                            RobotState.getInstance().getEstimatedPose().getTranslation(),
                            AllianceFlipUtil.apply(Rotation2d.kZero))),
                drive)
                .ignoringDisable(false));

    if (true) {
      return;
    }

    // Coral Intake Triggers
    driver
        .x()
        .whileTrue(
            superstructure.setGoalCommand(Superstructure.Goal.INTAKE)
                .alongWith(
                    DriveCommands.headingAlignCommand(drive,
                        () -> AllianceFlipUtil.apply(Rotation2d.fromDegrees(305.0)))));

    driver
        .b()
        .whileTrue(
            superstructure.setGoalCommand(Superstructure.Goal.INTAKE)
                .alongWith(
                    DriveCommands.headingAlignCommand(drive,
                        () -> AllianceFlipUtil.apply(Rotation2d.fromDegrees(55.0)))));

    // Algae Intake Trigger
    driver
        .leftTrigger()
        .whileTrue(
            arm.setGoalCommand(Arm.Goal.INTAKE).alongWith(
                roller.setGoalCommand(Roller.Goal.INTAKE)))
        .whileFalse(
            Commands.either(
                arm.setGoalCommand(Arm.Goal.HOLD),
                arm.setGoalCommand(Arm.Goal.STOW),
                () -> roller.hasGamePiece()));

    // Coral Scoring Triggers
    driver
        .rightTrigger()
        .whileTrue(
            Commands.sequence(
                DriveCommands.driveToPoseCommand(drive, scoringHelper::getSelectedAlignPose)
                    .until(() -> drive.isAutoAligned()
                        && (Constants.getType() == RobotType.SIMBOT ? true : superstructure.atGoal())),
                DriveCommands.driveToPoseCommand(drive, scoringHelper::getSelectedScorePose)
                    .alongWith(superstructure.setGoalCommand(() -> scoringHelper.getSuperstructureGoal())))
                .alongWith(
                    Commands
                        .waitUntil(() -> drive.disableBackVision())
                        .andThen(() -> vision.setEnableCamera(1,
                            false)))
                .finallyDo(() -> vision.setEnableCamera(1, true)));

    // driver
    // .rightStick()
    // .onTrue(
    // Commands.runOnce(() -> {
    // disableTranslationAutoAlign = !disableTranslationAutoAlign;
    // }));

    driver
        .rightBumper()
        .onTrue(superstructure.scoreCommand(false));

    // Algae Scoring Triggers
    driver
        .y()
        .whileTrue(roller.setGoalCommand(Roller.Goal.SCORE));

    driver
        .leftBumper()
        .whileTrue(
            Commands.either(
                superstructure.scoreCommand(true),
                arm.setGoalCommand(Arm.Goal.SCORE).alongWith(
                    Commands.startEnd(
                        () -> drive.setHeadingGoal(
                            () -> AllianceFlipUtil.apply(Rotation2d.kCCW_90deg)),
                        drive::clearHeadingGoal)),
                driver.rightTrigger()));
  }

  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(useKeyboard ? false
        : !DriverStation.isJoystickConnected(driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
    operatorDisconnected
        .set(useKeyboard ? false
            : !DriverStation.isJoystickConnected(buttonBoard.getHID().getPort()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.getCommand();
    // Commands.none();
  }
}
