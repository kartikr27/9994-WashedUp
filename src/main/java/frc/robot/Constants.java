// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.util.ArmPreset;
import frc.lib.util.controller.AsymmetricTrapezoidProfile.Constraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class SwerveConstants {
    public static final double stickDeadband = 0.05;

    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(0);
    public static final double wheelBase = Units.inchesToMeters(0);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (50.0/14.0) * (17.0/27.0) * (45.0/15.0); 
    public static final double angleGearRatio = (150.0 / 7.0); 

    public static final SwerveDriveKinematics m_driveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second  
    public static final double maxAngularVelocity = 6;            

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class FrontLeftModule {
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 0;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0); 
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class FrontRightModule {
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 0;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0); 
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class BackLeftModule {
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 0;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class BackRightModule {
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 0;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class ElbowConstants { //TODO
    public static final int kRightElbowMotorCanId = 10;
    public static final int kLeftElbowMotorCanId = 9;

    public static final double kElbowGearRatio = 240.0;
    public static final double kElbowLength = Units.inchesToMeters(28);
    public static final double kElbowPositionConversionFactor = (2*Math.PI) * kElbowGearRatio; //Radians * Gear ratio
    public static final double kElbowEncoderZeroOffset = 623.8;
    public static final double kElbowKinematicOffset = 105.0;
    public static final boolean kElbowMotorInverted = true;
    public static final boolean kElbowEncoderInverted = true;
    public static final double kElbowP = 8.0;
    public static final int kElbowMotorCurrentLimit = 40; //amps

    public static final Constraints kFarConstraints = new Constraints(16, 28, 24);
    public static final Constraints kCloseConstraints = new Constraints(30, 40, 30);

    public static final ArmFeedforward kElbowFeedForward = new ArmFeedforward(0, 0.47, 4.68, 0.04);
  }

  public static final class WristConstants { //TODO
    public static final int kRightWristMotorCanId = 11;

    public static final double kWristGearRatio = 225.0;
    public static final double kWristLength = Units.inchesToMeters(25);
    public static final double kWristPositionConversionFactor = (2*Math.PI) * kWristGearRatio;
    public static final double kWristEncoderZeroOffset = 1422.0;
    public static final double kWristKinematicOffset = 762.0;
    public static final boolean kWristMotorInverted = false;
    public static final boolean kWristEncoderInverted = true;
    public static final double kWristP = 8.0;
    public static final int kWristMotorCurrentLimit = 30; //amps

    public static final Constraints kFarConstraints = new Constraints(16, 28, 24);
    public static final Constraints kCloseConstraints = new Constraints(30, 40, 30);

    public static final ArmFeedforward kWristFeedForward = new ArmFeedforward(0, 0.35, 4.38, 0.03);
  }

  public static final class ManipulatorConstants {
    public static final int kManipulatorMotorCanId = 14;
    public static final int kManipulatorMotorCurrentLimit = 12; //amps 

    public static final double kIntakeMotorSpeed = 1;
    public static final double kHoldMotorSpeed = 0.5;
    public static final double kOuttakeMotorSpeed = -0.5;
  }

  public static final class ArmConstants {
    //Intake
    public static final ArmPreset kForwardShelfCone = new ArmPreset(90, 90);
    public static final ArmPreset kForwardShelfCube = new ArmPreset(90, 90);

    public static final ArmPreset kForwardPortalCone = new ArmPreset(90, 90);
    public static final ArmPreset kForwardPortalCube = new ArmPreset(90, 45);

    public static final ArmPreset kForwardFloorCone = new ArmPreset(90, 0);
    public static final ArmPreset kForwardFloorCube = new ArmPreset(90, 0);

    //Score
    public static final ArmPreset kForwardHighCone = new ArmPreset(30, 30);
    public static final ArmPreset kForwardHighCube = new ArmPreset(30, 30);

    public static final ArmPreset kForwardMiddleCone = new ArmPreset(30, 90);
    public static final ArmPreset kForwardMiddleCube = new ArmPreset(30, 45);

    public static final ArmPreset kForwardLowCone = new ArmPreset(30, 0);
    public static final ArmPreset kForwardLowCube = new ArmPreset(30, 0);

    //Hold
    public static final ArmPreset kHoldPreset = new ArmPreset(0, 0);
  }
}
