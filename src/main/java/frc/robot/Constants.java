// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class Swerve {
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

    public static final SwerveDriveKinematics swerveKinematics =
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
    public static final class Mod0 {
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 0;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0); 
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 0;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0); 
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 0;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 0;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }
  public static final class Arm{
    public static final int armMotorRight = 6;

    public static final int armMotorLeft = 17;

    public static final int BORE_ENCODER_PORT = 0;

    // Absolute encoder offset
    public static final double BORE_ENCODER_OFFSET = 0.562 - .047;

    public static final Rotation2d ESTOP_TOLERANCE = Rotation2d.fromDegrees(10);

    // Profiled PID controller gains
    public static final double kP = 34.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kMaxVelocityRadiansPerSecond = 5.45;
    public static final double kMaxAccelerationRadiansPerSecondSquared = 6.4;

    public static final double kSlowMaxVelocityRadiansPerSecond = 4.2;
    public static final double kSlowMaxAccelerationRadiansPerSecondSquared = 4.0;

    // Feedforward constants
    public static final double kS = 0.0;
    public static final double kG = 0.35;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    // Setpoints

    public static final Rotation2d ARM_SETPOINT_DOUBLE_SUBSTATION = Rotation2d.fromDegrees(102);
    public static final Rotation2d ARM_SETPOINT_BOT = Rotation2d.fromDegrees(30.5);
    public static final Rotation2d ARM_SETPOINT_PREINTAKE = Rotation2d.fromDegrees(51);
    public static final Rotation2d ARM_SETPOINT_PREHIGH_SCORE_AUTON =
        Rotation2d.fromDegrees(170); // 170 for parallel
    public static final Rotation2d ARM_SETPOINT_PREHIGH_SCORE =
        Rotation2d.fromDegrees(244); // 170 for parallel
    public static final Rotation2d ARM_SETPOINT_UPRIGHT_CONE_INTAKE = Rotation2d.fromDegrees(49.3);
    public static final Rotation2d ARM_SETPOINT_SIDE_CONE_INTAKE =
        Rotation2d.fromDegrees(41.26); // 42.8
    public static final Rotation2d ARM_SETPOINT_MID = Rotation2d.fromDegrees(282);
    public static final Rotation2d ARM_SETPOINT_HIGH = Rotation2d.fromDegrees(264);
  } 
  public static final class Wrist {
    public static final double P = 17.2;
    public static final double I = 0.1;
    public static final double D = 0.0;
    public static final int WristMotor = 13;

    public static final double WristMaxVelocity = 9.8;
    public static final double WristMaxAccel = 17;

    public static final Rotation2d WRIST_STOP_MAX = new Rotation2d();
    public static final Rotation2d WRIST_STOP_MIN = new Rotation2d();

    public static final double boreEncoderOffset = -0.35;
    // -0.507

    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final Rotation2d wrist_upright_cone_intake = Rotation2d.fromDegrees(180);
    public static final Rotation2d wrist_zero = Rotation2d.fromDegrees(4);
    public static final Rotation2d wrist_cone_intake = Rotation2d.fromDegrees(90);
    public static final Rotation2d wrist_cone_leftScore = Rotation2d.fromDegrees(90);
  }
}
