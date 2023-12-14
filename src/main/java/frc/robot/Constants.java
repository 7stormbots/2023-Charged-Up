// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DriveConstants {
    public static double spdLimitFast = .85;
    public static double turnLimitFast = .8;

    public static double spdLimitSlow = .25;
    public static double turnLimitSlow = .25;
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.6;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.2; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)
    // private final Swerve swerveSubsystem = new Swerve();

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11; //
    public static final int kRearLeftDrivingCanId = 13; //
    public static final int kFrontRightDrivingCanId = 15; // 
    public static final int kRearRightDrivingCanId = 17; //

    public static final int kFrontLeftTurningCanId = 10; //
    public static final int kRearLeftTurningCanId = 12; //
    public static final int kFrontRightTurningCanId = 14; //
    public static final int kRearRightTurningCanId = 16; //

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kArmControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.75;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI / 2;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI / 2;

    public static final double kPXController = 1;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 10;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);


    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    public static final double kWheelBase = Units.inchesToMeters(26.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static Object Swerve;


  public static final class ArmConstants {

    // CAN IDs
    public static final int liftLeftMotorCanId = 18;
    public static final int liftRightMotorCanId = 19;
    public static final int extendMotorCanId = 20;
    public static final int wristMotorCanId = 21;

    public static double liftTarget;


    public static double liftState;
    public static double extendState;

    public static final int liftHigh = 5900;
    public static final int liftMid = 5100;
    public static final int liftLow = 2000;
    public static final int liftIn = 250;

    public static final int extendToggle = 2000;

    public static final int liftDropIntake = 2970;

    public static final int liftMax = 6200;
    public static final int liftMin = 0;

    public static final int maxLiftInterval = 500;

    public static double extendMax = 220;
    public static double extendMin = -15;

    public static final int maxExtendInterval = 33;

    public static final int wristMax = 50;
    public static final int wristMin = -25;

    public static double liftMaxVel = 6500;
    public static double liftMaxAcc = 4000;

    public static double extendMaxVel = 200;
    public static double extendMaxAcc = 200;

    public static double wristMaxVel = 175;
    public static double wristMaxAcc = 85;

    public static double liftKp = 1.5;
    public static double liftKi = 1;
    public static double liftKd = 0;

    public static double extendKp = .075;
    public static double extendKi = 0;
    public static double extendKd = 0;

    public static double wristKp = .075;
    public static double wristKi = .01;
    public static double wristKd = 0;

    public static final IdleMode kExtendMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kWristMotorIdleMode = IdleMode.kCoast;

    public static final int kLiftMotorCurrentLimit = 1;
    public static final int kExtendMotorCurrentLimit = 20;
    public static final int kWristMotorCurrentLimit = 7;

    public static double liftTicksPerRev = 4096;
    public static double liftSprocketMultiplier = 7.5625;
    public static double liftTicksPerSprocketRev = liftTicksPerRev * liftSprocketMultiplier;
    public static double liftTicksPerDEG = liftTicksPerSprocketRev / 360;

    public static double extendTicksPerRev = 0;
    public static double extendTicksPerDeg = extendTicksPerRev / 360;
  }

  public static final class ClawConstants {
    public static final int intakeMotorCanId = 22;
    public static final IdleMode kIntakeMotorIdleMode = IdleMode.kBrake;

    public static final int leftServoChannel = 8;
    public static final int rightServoChannel = 6;
    

    public static boolean open;

    public static final double openLPosition = .4;
    public static final double openRPosition = calculateRightServo(openLPosition);

    public static final double coneCloseLPosition = 1;
    public static final double coneCloseRPosition = calculateRightServo(coneCloseLPosition);

    public static final double cubeCloseLPosition = .6;
    public static final double cubeCloseRPosition = calculateRightServo(cubeCloseLPosition);

    public static double calculateRightServo(double position) {
      int full = 1;

      double calculated = full - position;

      return calculated;
    }
  }

  

  public static final class LimelightConstants {
    public static final double LimelightMountingAngle = 0; //mounting angle of the camera DEGREES
    public static final double LimelightMountingHeight = 19.5; //height of camera off the ground INCHES (NOT FINAL VALUE WILL KNOW BEFORE LACROSSE)
    public static final double TargetHeight = 41.875; //height of high pole off the ground (Mahi still not figured out how to do multiple targets)

    public static double x;  //wrong
    public static double y;  //wrong
    public static double area;
        
    public static boolean limelightToggle = false;
  }
}
