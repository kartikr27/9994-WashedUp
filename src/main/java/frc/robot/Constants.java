// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 5.14;
    public static final double kTurningMotorGearRatio = 1 / 12.8;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kTurnP = 0.3;
    public static final double kDriveP = 0.7;
}

public static final class DriveConstants {

    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(20.5);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(20.5);


    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
    //Drivetrain Motor Ports
    public static final int kFrontLeftDriveMotorPort = 5;
    public static final int kBackLeftDriveMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 8;
    public static final int kBackRightDriveMotorPort = 3;

    public static final int kFrontLeftTurningMotorPort = 6;
    public static final int kBackLeftTurningMotorPort = 1;
    public static final int kFrontRightTurningMotorPort = 7;
    public static final int kBackRightTurningMotorPort = 4;

    //Relative Encoders
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    //Absolute Encoders
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 14;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 13;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 15;
    public static final int kBackRightDriveAbsoluteEncoderPort = 12;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(332.578-180); 
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(269.38-180); 
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(133.59);
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(339.25); 

    //Physical
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5.8;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    //Speed and Accel
    public static final double kTeleDriveMaxSpeedMultiplier = 1.0;
    public static final double kCreepModeMultiplier = 0.3;
    public static final double kTeleDriveMaxAnglularSpeedMultiplier = 0.5;
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * kTeleDriveMaxSpeedMultiplier;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * kTeleDriveMaxAnglularSpeedMultiplier;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3.0;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
}

public static final class OIConstants {
  public static final int kDriverControllerPort = 0;

  public static final int kDriverYAxis = 1;
  public static final int kDriverXAxis = 0;
  public static final int kDriverRotAxis = 4;
  public static final int kDriverFieldOrientedButtonIdx = 1;

  public static final double kDeadband = 0.1;
}
  
  public static final class Arm{
    public static final int armMotorRight = 10;

    public static final int armMotorLeft = 9;

    public static final int BORE_ENCODER_PORT = 0;

    // Absolute encoder offset
    public static final double BORE_ENCODER_OFFSET = 0.075;


    // Profiled PID controller gains
    public static final double kP = 12.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kMaxVelocityRadiansPerSecond = 5.8;
    public static final double kMaxAccelerationRadiansPerSecondSquared = 8.0;

    public static final double kSlowMaxVelocityRadiansPerSecond = 4.2;
    public static final double kSlowMaxAccelerationRadiansPerSecondSquared = 4.0;

    // Feedforward constants
    public static final double kS = 0.0;
    public static final double kG = 0.32;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    // Setpoints

    public static final Rotation2d ARM_SETPOINT_DOUBLE_SUBSTATION = Rotation2d.fromDegrees(188);

    public static final Rotation2d ARM_SETPOINT_SINGLE_SUBSTATION = Rotation2d.fromDegrees(188);
    public static final Rotation2d ARM_SETPOINT_BOT = Rotation2d.fromDegrees(180);

    public static final Rotation2d ARM_SETPOINT_UPRIGHT_CONE_INTAKE = Rotation2d.fromDegrees(230
    );
    public static final Rotation2d ARM_SETPOINT_GROUND_INTAKE_CONE =
        Rotation2d.fromDegrees(191); 
        public static final Rotation2d ARM_SETPOINT_GROUND_INTAKE_CUBE =
        Rotation2d.fromDegrees(191); 
    public static final Rotation2d ARM_SETPOINT_MID_CONE = Rotation2d.fromDegrees(60);
    public static final Rotation2d ARM_SETPOINT_MID_CUBE = Rotation2d.fromDegrees(154);
    public static final Rotation2d ARM_SETPOINT_HIGH = Rotation2d.fromDegrees(63);
  } 
  public static final class Elbow {
    public static final double P = 11.2;
    public static final double I = 2.7;
    public static final double D = 0.0;
    public static final int ElbowMotor = 11;

    public static final double ElbowMaxVelocity = 14.8;
    public static final double ElbowMaxAccel = 18.0;

    public static final double boreEncoderOffset = 0.0;
    public static final int boreEncoderPort = 1;

    public static final double kS = 0.0;
    public static final double kG = 0.2;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    

    public static final Rotation2d ELBOW_SETPOINT_DOUBLE_SUBSTATION = Rotation2d.fromDegrees(43);

    public static final Rotation2d ELBOW_SETPOINT_SINGLE_SUBSTATION = Rotation2d.fromDegrees(43);
    public static final Rotation2d ELBOW_SETPOINT_BOT = Rotation2d.fromDegrees(1.0);

    public static final Rotation2d ELBOW_SETPOINT_UPRIGHT_CONE_INTAKE = Rotation2d.fromDegrees(218);
    public static final Rotation2d ELBOW_SETPOINT_GROUND_INTAKE_CONE =
        Rotation2d.fromDegrees(167); 

        public static final Rotation2d ELBOW_SETPOINT_GROUND_INTAKE_CUBE =
        Rotation2d.fromDegrees(140); 
    public static final Rotation2d ELBOW_SETPOINT_MID_CONE = Rotation2d.fromDegrees(110);
    public static final Rotation2d ELBOW_SETPOINT_MID_CUBE = Rotation2d.fromDegrees(-77.6);
    public static final Rotation2d ELBOW_SETPOINT_HIGH = Rotation2d.fromDegrees(195);
  }

  public static final class Intake {
    public static final double P = 0.01;
    public static final double I = 0;
    public static final double D = 0;
    public static final int intakeMotor = 25;

  }
  public static final class Autonomous {
    // Profiled PID Controller for rotation
    public static final double DRIVE_CONTROLLER_ROTATION_KP = 2.1; // 0.4
    public static final double DRIVE_CONTROLLER_ROTATION_KI = 0.0;
    public static final double DRIVE_CONTROLLER_ROTATION_KD = 0.0;
    
    // TODO: Tune this
    public static final double AUTO_BALANCE_SPEED = 0.35;
    public static final double AUTO_BALANCE_GROUND_SPEED = 1.1;
    public static final double AUTO_BALANCE_GROUND_ANGLE_THRESHOLD = 14;
    public static final double AUTO_BALANCE_VELOCITY_THRESHOLD = 4.5;
    public static final double AUTO_BALANCE_GROUND_VELOCITY_THRESHOLD = 4.0;
    public static final double AUTO_BALANCE_POSITION_THRESHOLD = 3.0;

    public static final double AUTO_BALANCE_P_START = 0.051;
    public static final double AUTO_BALANCE_P_MULTIPLIER = 0.8;

    public static final double balancedAngle = 0; // The angle the robot should be at when balanced
    public static final double kP = 0.073; // The proportional constant for the PID controller
    public static final double angleSetPoint =
        0; // The angle the PID controller should try to reach
    public static final double kTurn = 0.007; // The constant for the turn PID controller
  }
}
