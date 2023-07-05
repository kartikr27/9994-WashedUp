// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;


public class Arm extends ProfiledPIDSubsystem {
  /** Creates a new Arm. */
  private CANSparkMax armMotorRight;

  private CANSparkMax armMotorLeft;  

  private DutyCycleEncoder boreEncoder;

  private ArmFeedforward armFeedForward;

  private ShuffleboardTab armTab;

  private double voltage = 0;

  private static final TrapezoidProfile.Constraints normalConstraints =
      new TrapezoidProfile.Constraints(
          Constants.Arm.kMaxVelocityRadiansPerSecond,
          Constants.Arm.kMaxAccelerationRadiansPerSecondSquared);

  private static final TrapezoidProfile.Constraints slowConstraints =
      new TrapezoidProfile.Constraints(
          Constants.Arm.kSlowMaxVelocityRadiansPerSecond,
          Constants.Arm.kSlowMaxAccelerationRadiansPerSecondSquared);

  private GenericEntry enabledEntry;
  private GenericEntry voltageEntry;
  private GenericEntry coneEntry;
  private GenericEntry positionEntry;
  private GenericEntry setpointEntry;
  private GenericEntry goalEntry;
  private GenericEntry velocitySetpointEntry;
  private GenericEntry currentLeftEntry;
  private GenericEntry currentRightEntry;

  private GenericEntry rotationsEntry;

  private GenericEntry velocityEntry;

  public double prevBoreValue;
  private Rotation2d prevPosition;
  private double prevTime;
  public int rotations;

  public Arm() {
    super(
        new ProfiledPIDController(
            Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD, normalConstraints));

    getController().setTolerance(Units.degreesToRadians(3));


    armMotorRight = new CANSparkMax(Constants.Arm.armMotorRight, MotorType.kBrushless);
    armMotorRight.setIdleMode(IdleMode.kBrake);
    armMotorRight.setSmartCurrentLimit(40);
    armMotorRight.burnFlash();

    armMotorLeft = new CANSparkMax(Constants.Arm.armMotorLeft, MotorType.kBrushless);
    armMotorLeft.setIdleMode(IdleMode.kBrake);
    armMotorLeft.setSmartCurrentLimit(40);
    
    armMotorLeft.setInverted(true);

    armMotorLeft.burnFlash();

    boreEncoder = new DutyCycleEncoder(Constants.Arm.BORE_ENCODER_PORT);

    armFeedForward =
        new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV, Constants.Arm.kA);

    rotations = 0;
    prevBoreValue = getBore();
    prevPosition = getAbsoluteRotation();
    prevTime = Timer.getFPGATimestamp();

    armTab = Shuffleboard.getTab("Arm");

  
    enabledEntry =
        armTab.add("Enabled", m_enabled).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    // setpointEntry = armTab.add("Setpoint", 0).withWidget(BuiltInWidgets.kGyro).getEntry();

    positionEntry = armTab.add("Position", getAbsoluteRotation().getDegrees()).getEntry();

    voltageEntry = armTab.add("Voltage", 0).withWidget(BuiltInWidgets.kVoltageView).getEntry();
    coneEntry = armTab.add("Cone", false).withPosition(9, 0).getEntry();
    setpointEntry = armTab.add("Setpoint", getController().getSetpoint().position).getEntry();
    goalEntry = armTab.add("Goal", getController().getGoal().position).getEntry();
    currentLeftEntry =
        armTab.add("Current Draw (Left)", armMotorLeft.getOutputCurrent()).getEntry();
    currentRightEntry =
        armTab.add("Current Draw (Right)", armMotorRight.getOutputCurrent()).getEntry();

    rotationsEntry = armTab.add("Rotations", rotations).withPosition(10, 0).getEntry();
    velocityEntry = armTab.add("Velocity", 0).getEntry();
    velocitySetpointEntry = armTab.add("Velocity Setpoint", 0.0).getEntry();



  }


  @Override
  public void periodic() {
    super.periodic();

    enabledEntry.setBoolean(m_enabled);
    positionEntry.setDouble(getAbsoluteRotation().getDegrees());
    setpointEntry.setDouble(getBore());
    goalEntry.setDouble(Units.radiansToDegrees(getController().getGoal().position));
    currentLeftEntry.setDouble(armMotorLeft.getOutputCurrent());
    currentRightEntry.setDouble(armMotorRight.getOutputCurrent());
    rotationsEntry.setDouble(rotations);
    velocitySetpointEntry.setDouble(getController().getSetpoint().velocity);

    Rotation2d deltaPosition = getAbsoluteRotation().minus(prevPosition);
    double deltaTime = Timer.getFPGATimestamp() - prevTime;

    double degreesPerSecond = deltaPosition.getRadians() / deltaTime;
    velocityEntry.setDouble(degreesPerSecond);

    prevPosition = getAbsoluteRotation();
    prevTime = Timer.getFPGATimestamp();
  }

  public double getBore() {
    return MathUtil.inputModulus(
        boreEncoder.getAbsolutePosition() + Constants.Arm.BORE_ENCODER_OFFSET, 0.0, 1.0);
  }



  public Rotation2d getAbsoluteRotation() {
    double radians = 2 * Math.PI * getBore();

    return new Rotation2d(radians);


  }

  @Override
  protected void useOutput(double output, State setpoint) {
    if (this.m_enabled) {

      voltage =
          output + armFeedForward.calculate(0.5 * Math.PI + setpoint.position, setpoint.velocity);

      // voltage = MathUtil.clamp(voltage, -12.0, 12.0);

      voltageEntry.setDouble(voltage);

      armMotorLeft.setVoltage(voltage);
      armMotorRight.setVoltage(voltage);
    }
  }

  @Override
  protected double getMeasurement() {
    return getAbsoluteRotation().getRadians();
  }

  public void setGoal(Rotation2d rotation) {
    this.setGoal(rotation.getRadians());
  }

  public void stopArm() {
    armMotorRight.stopMotor();
    armMotorLeft.stopMotor();
  }

  public void setArmCommand(double setpoint) {
    Rotation2d angle = new Rotation2d(setpoint);

    armMotorLeft.setVoltage(setpoint);
    armMotorRight.setVoltage(setpoint);
  }

  public void setSpeed(double speed) {
    disable();
    armMotorLeft.set(speed);
    armMotorRight.set(speed);
  }


}