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

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;

// /**
//  * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
//  * constants. This class should not be used for any other purpose. All constants should be declared
//  * globally (i.e. public static). Do not put anything functional in this class.
//  *
//  * <p>It is advised to statically import this class (or one of its inner classes) wherever the
//  * constants are needed, to reduce verbosity.
//  */
// public final class Constants {
//   public static final class DriveConstants {
//     // Driving Parameters - Note that these are not the maximum capable speeds of
//     // the robot, rather the allowed maximum speeds
//     public static final double kMaxSpeedMetersPerSecond = 4.8;
//     public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

//     // Chassis configuration
//     public static final double kTrackWidth = Units.inchesToMeters(24.5);
//     // Distance between centers of right and left wheels on robot
//     public static final double kWheelBase = Units.inchesToMeters(28.0);
//     // Distance between front and back wheels on robot
//     public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
//         new Translation2d(.552, .564),
//         new Translation2d(.552, -.061),
//         new Translation2d(-.164, .564),
//         new Translation2d(-.164, -.061));

//     // Angular offsets of the modules relative to the chassis in radians
//     public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
//     public static final double kFrontRightChassisAngularOffset = 0;
//     public static final double kBackLeftChassisAngularOffset = Math.PI;
//     public static final double kBackRightChassisAngularOffset = Math.PI / 2;

//     // SPARK MAX CAN IDs
//     public static final int kFrontLeftDrivingCanId = 19; //Green
//     public static final int kRearLeftDrivingCanId = 18; //Black
//     public static final int kFrontRightDrivingCanId = 15; //Red
//     public static final int kRearRightDrivingCanId = 14; //Blue

//     public static final int kFrontLeftTurningCanId = 17;
//     public static final int kRearLeftTurningCanId = 16;
//     public static final int kFrontRightTurningCanId = 20; //PDH port 0
//     public static final int kRearRightTurningCanId = 1;

//     public static final boolean kGyroReversed = false;
//   }

//   public static final class ModuleConstants {
//     // The MAXSwerve module can be configured with one of three pinion gears: 12T,
//     // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
//     // more teeth will result in a robot that drives faster).
//     public static final int kDrivingMotorPinionTeeth = 13;

//     // Calculations required for driving motor conversion factors and feed forward
//     public static final double kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60;
//     public static final double kWheelDiameterMeters = 0.0762;
//     public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
//     // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
//     // teeth on the bevel pinion
//     public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
//     public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
//         / kDrivingMotorReduction;
//   }

  // public static final class OIConstants {
  //   public static final int kDriverControllerPort0 = 0;
  //   public static final int kDriverControllerPort1 = 1;
  //   public static final int kDriverControllerPort2 = 2;
  //   public static final double kDriveDeadband = 0.1;
  

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
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

