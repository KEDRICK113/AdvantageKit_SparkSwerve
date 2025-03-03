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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.OIConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
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
  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  /// private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final PivotSubsystem m_PivotSubsystem = new PivotSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  Joystick m_Joystick0 = new Joystick(OIConstants.kDriverControllerPort0);
  Joystick m_Joystick1 = new Joystick(OIConstants.kDriverControllerPort1);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  SendableChooser<Command> m_chooser = new SendableChooser();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // new JoystickButton(m_Joystick1, 2)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));

    final JoystickButton ElevatorUp = new JoystickButton(m_Joystick1, 4);
    ElevatorUp.whileTrue(new ElevatorCommand(ElevatorConstants.ElevatorSpeed, m_ElevatorSubsystem))
        .onFalse(new ElevatorCommand(0, m_ElevatorSubsystem));

    final JoystickButton ElevatorDown = new JoystickButton(m_Joystick1, 3);
    ElevatorDown.whileTrue(
            new ElevatorCommand(-1 * ElevatorConstants.ElevatorSpeed / 2, m_ElevatorSubsystem))
        .onFalse(new ElevatorCommand(0, m_ElevatorSubsystem));

    final JoystickButton Climb = new JoystickButton(m_Joystick0, 11); // lowers arms
    Climb.whileTrue(m_ClimberSubsystem.climberCommand(ClimberConstants.ClimbSpeed * -1))
        .whileFalse(new ClimberSubsystem().climberCommand(0));

    final JoystickButton ClimbReverse = new JoystickButton(m_Joystick1, 11); // Raises arm
    ClimbReverse.whileTrue(m_ClimberSubsystem.climberCommand(ClimberConstants.ClimbSpeed))
        .whileFalse(m_ClimberSubsystem.climberCommand(0));

    final JoystickButton PivotUp = new JoystickButton(m_Joystick0, 4);
    PivotUp.whileTrue(m_PivotSubsystem.pivotCommand(PivotConstants.PivotSpeed))
        .whileFalse(new PivotSubsystem().pivotCommand(0));

    final JoystickButton PivotDown = new JoystickButton(m_Joystick0, 3);
    PivotDown.whileTrue(m_PivotSubsystem.pivotCommand(PivotConstants.PivotSpeed * -1))
        .whileFalse(new PivotSubsystem().pivotCommand(0));

    /*final JoystickButton ElevatorStart = new JoystickButton(m_Joystick1, 3);
    ElevatorStart.onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, 0));

    final JoystickButton ElevatorL2 = new JoystickButton(m_Joystick1, 5);
    ElevatorL2.onTrue(new ElevatorPIDCommand(m_ElevatorSubsystem, 12));

    final JoystickButton ResetElevatorEncoder = new JoystickButton(m_Joystick0, 7);
    ResetElevatorEncoder.onTrue(m_ElevatorSubsystem.ResetEncoder());*/

    final JoystickButton PivotLoad = new JoystickButton(m_Joystick0, 1);
    PivotLoad.onTrue(new PivotPIDCommand(m_PivotSubsystem, 180));

    final JoystickButton PivotL1 = new JoystickButton(m_Joystick1, 1);
    PivotL1.onTrue(new PivotPIDCommand(m_PivotSubsystem, 80));
  }

  private Command NoAuto;

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
