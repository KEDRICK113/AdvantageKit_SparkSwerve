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

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.8;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(24.);
  public static final double wheelBase = Units.inchesToMeters(28);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

  // Device CAN IDs
  // NOTE: These are the CAN IDs for the 2025 robot
  public static final int pigeonCanId = 9;

  public static final int frontLeftDriveCanId = 19; // Green
  public static final int backLeftDriveCanId = 18; // Black
  public static final int frontRightDriveCanId = 15; // Red
  public static final int backRightDriveCanId = 14; // Blue

  public static final int frontLeftTurnCanId = 17;
  public static final int backLeftTurnCanId = 16;
  public static final int frontRightTurnCanId = 20; // PDH port 0
  public static final int backRightTurnCanId = 1;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
  public static final double driveMotorReduction =
      (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassLbs = 115;
  public static final double robotMassKg = (robotMassLbs * 0.45359237);
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  // Added from 2025 Alpha code KLJ
  public static final class OIConstants {
    public static final int kDriverControllerPort0 = 0;
    public static final int kDriverControllerPort1 = 1;
    public static final int kDriverControllerPort2 = 2;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class VortexMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class MotorConstants {
    public static final int AmpLimit550 = 20;
    public static final int AmpLimitNeo = 50;
    public static final int AmplLimitVortex = 80;
  }

  public static final class ElevatorConstants {
    public static final int ElevatorCanID = 4;
    public static final double ElevatorSpeed = .3;
  }

  public static final class ClimberConstants {
    public static final int FrontClimbCanID = 2;
    public static final int BackClimbCanID = 3;
    public static final double ClimbSpeed = 0.4;
  }

  public static final class PivotConstants {
    public static final int PivotCanID = 13;
    public static final double PivotSpeed = .15;
    public static final double PivotMaxSpeed = .6;
  }
}
