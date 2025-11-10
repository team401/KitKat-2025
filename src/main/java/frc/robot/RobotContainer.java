// Copyright 2021-2025 FRC 6328
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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSpark;
import frc.robot.subsystems.scoring.InitBindings;
import frc.robot.subsystems.scoring.shooter.ScoringSubsystem;
import frc.robot.subsystems.scoring.shooter.ShooterIOSim;
import frc.robot.subsystems.scoring.shooter.ShooterIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final ScoringSubsystem scoring;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveIOSpark());
        scoring = new ScoringSubsystem(new ShooterIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSim());
        scoring = new ScoringSubsystem(new ShooterIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {});
        scoring = null;
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Move and score auto (left from robot's perspective)",
        new SequentialCommandGroup(
            // Turn clockwise and move forward for 3 seconds
            DriveCommands.arcadeDrive(drive, () -> 0.85, () -> -0.5).withTimeout(2),
            // Stop driving again
            DriveCommands.arcadeDrive(drive, () -> 0.0, () -> 0.0).withTimeout(0.0),
            // Spin the shooter for 0.75 seconds
            Commands.run(
                    () -> {
                      scoring.spin(6.0);
                    },
                    scoring)
                .withTimeout(0.67),
            // Stop the shooter for marginal battery savings
            Commands.run(
                    () -> {
                      scoring.stop();
                    },
                    scoring)
                .withTimeout(0),
            // Turn the robot counterclockwise and move it backwards (go back to starting position)
            DriveCommands.arcadeDrive(drive, () -> -0.85, () -> 0.5).withTimeout(1.8),
            // Stop the robot
            DriveCommands.arcadeDrive(drive, () -> 0.0, () -> 0.0).withTimeout(0),

            // Moves the robot forward for 5 seconds
            DriveCommands.arcadeDrive(drive, () -> 0.9, () -> 0.0).withTimeout(3),
            // Stop the robot
            DriveCommands.arcadeDrive(drive, () -> 0.0, () -> 0.0).withTimeout(0),
            // Turn robot clockwise
            DriveCommands.arcadeDrive(drive, () -> 0.0, () -> -0.8).withTimeout(1.5),
            // Stop the robot
            DriveCommands.arcadeDrive(drive, () -> 0.0, () -> 0.0).withTimeout(0),
            // Move the robot backward for 3 seconds
            DriveCommands.arcadeDrive(drive, () -> -0.9, () -> 0.0).withTimeout(1.0),
            // Stop the robot (to pause for coral)
            DriveCommands.arcadeDrive(drive, () -> 0.0, () -> 0.0).withTimeout(2),
            // Move the robot forward to score the second time
            DriveCommands.arcadeDrive(drive, () -> 1.0, () -> -0.3).withTimeout(2),
            // Stop the robot
            DriveCommands.arcadeDrive(drive, () -> 0.0, () -> 0.0).withTimeout(0),
            // Spin the shooter for 0.75 seconds
            Commands.run(
                    () -> {
                      scoring.spin(6.0);
                    },
                    scoring)
                .withTimeout(1.0),
            // Stop the shooter for marginal battery savings
            Commands.runOnce(
                () -> {
                  scoring.stop();
                },
                scoring)));

    autoChooser.addOption(
        "encoder left score",
        new SequentialCommandGroup(
            // Turn clockwise and drive 1.5 meters forward
            DriveCommands.arcadeDrive(drive, () -> 0.5, () -> -0.5)
                .until(
                    () ->
                        drive.getLeftPositionMeters() >= 3.02
                            && drive.getRightPositionMeters() >= 2.46),
            // Stop driving
            DriveCommands.arcadeDrive(drive, () -> 0.0, () -> 0.0),
            // Spin the shooter for 0.75 seconds
            Commands.run(
                    () -> {
                      scoring.spin(0.75);
                    },
                    scoring)
                .withTimeout(0.75),
            // Stop the shooter for marginal battery savings
            Commands.run(
                () -> {
                  scoring.stop();
                },
                scoring)));

    autoChooser.addOption(
        "Move and score auto (middle)",
        new SequentialCommandGroup(
            // Drive forward for 2.5 seconds
            DriveCommands.arcadeDrive(drive, () -> 0.7, () -> 0.0).withTimeout(2.5),
            // Stop driving
            DriveCommands.arcadeDrive(drive, () -> 0.0, () -> 0.0).withTimeout(0),
            // Spin the shooter for 2.5 seconds
            Commands.run(
                    () -> {
                      scoring.spin();
                    },
                    scoring)
                .withTimeout(0.5),
            // Stop the shooter
            Commands.runOnce(
                () -> {
                  scoring.stop();
                },
                scoring)));
    autoChooser.addOption(
        "Move and score auto (right from robot's perspective)",
        new SequentialCommandGroup(
            // Drive forward for 1 second
            DriveCommands.arcadeDrive(drive, () -> 0.7, () -> 0.0).withTimeout(1),
            // Stop driving
            DriveCommands.arcadeDrive(drive, () -> 0.0, () -> 0.0).withTimeout(0.2),
            // Turn counterclockwise and move forward for 2 seconds
            DriveCommands.arcadeDrive(drive, () -> 0.85, () -> 0.5).withTimeout(2),
            // Stop driving again
            DriveCommands.arcadeDrive(drive, () -> 0.0, () -> 0.0).withTimeout(0.2),
            // Spin the shooter for 2.5 seconds
            Commands.run(
                    () -> {
                      scoring.spin();
                    },
                    scoring)
                .withTimeout(0.5),
            // Stop the shooter for marginal battery savings and to prevent them from remaining
            // spinning at start of teleop
            Commands.runOnce(
                () -> {
                  scoring.stop();
                },
                scoring)));
    autoChooser.addOption(
        "MOVE ONLY",
        new ParallelRaceGroup(
            new WaitCommand(2.0), DriveCommands.arcadeDrive(drive, () -> 0.5, () -> 0.0)));
    autoChooser.addOption(
        "middle score encoder",
        new SequentialCommandGroup(
            // Drive 2.23 meters forward
            DriveCommands.arcadeDrive(drive, () -> 0.5, () -> 0.0)
                .until(() -> drive.getLeftPositionMeters() >= 2.23),
            // Stop driving
            DriveCommands.arcadeDrive(drive, () -> 0.0, () -> 0.0),
            // Spin the shooter for 0.75 seconds
            Commands.run(
                    () -> {
                      scoring.spin(0.75);
                    },
                    scoring)
                .withTimeout(0.75),
            // Stop the shooter for marginal battery savings
            Commands.run(
                () -> {
                  scoring.stop();
                },
                scoring)));
    autoChooser.addOption(
        "encoder test",
        new SequentialCommandGroup(
            // Drive forward 1 meter
            DriveCommands.arcadeDrive(drive, () -> 0.5, () -> 0.0)
                .until(() -> drive.getLeftPositionMeters() > 1),
            DriveCommands.arcadeDrive(drive, () -> 0.0, () -> 0.0)));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal arcade drive
    drive.setDefaultCommand(
        DriveCommands.arcadeDrive(
            drive, () -> -controller.getLeftY(), () -> -controller.getRightX()));
    InitBindings.initScoringBindings(controller, scoring);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
