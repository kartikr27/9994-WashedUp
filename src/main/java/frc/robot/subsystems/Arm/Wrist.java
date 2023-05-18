// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.lib.util.controller.AsymmetricProfiledPIDController;
import frc.lib.util.controller.AsymmetricTrapezoidProfile.Constraints;
import frc.lib.util.controller.AsymmetricTrapezoidProfile.State;

public class Wrist extends SubsystemBase {
  private CANSparkMax WristMotor;
  private AbsoluteEncoder WristEncoder;

  private AsymmetricProfiledPIDController WristController = 
    new AsymmetricProfiledPIDController(0,0,0, WristConstants.kFarConstraints); //MUST START AT 0 P
  
  /** Creates a new Wrist. */
  public Wrist() {
    WristMotor = new CANSparkMax(WristConstants.kRightWristMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    WristMotor.setInverted(WristConstants.kWristMotorInverted);
    WristMotor.setIdleMode(IdleMode.kBrake);
    WristMotor.setSmartCurrentLimit(WristConstants.kWristMotorCurrentLimit);
    
    WristEncoder = WristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    WristEncoder.setPositionConversionFactor(WristConstants.kWristPositionConversionFactor);
    WristEncoder.setInverted(WristConstants.kWristEncoderInverted);
    WristEncoder.setZeroOffset(WristConstants.kWristEncoderZeroOffset);
    //todo set velocity conversion factor

    WristMotor.burnFlash();

    WristController.disableContinuousInput();
  }

  public double getAngleRadians() {
    return (WristEncoder.getPosition() - WristConstants.kWristKinematicOffset) / WristConstants.kWristGearRatio;
  }

  public void setTargetRadians(double targetAngleRadians) {
    SmartDashboard.putNumber("Wrist Target Angle", Units.radiansToDegrees(targetAngleRadians));
    WristController.setP(WristConstants.kWristP);
    Constraints selectedConstraint = 
      (Math.abs(targetAngleRadians - getAngleRadians()) < Units.degreesToRadians(45)) ? 
      WristConstants.kCloseConstraints : WristConstants.kFarConstraints;
    WristController.setConstraints(selectedConstraint);
    SmartDashboard.putString("Wrist Selected Constraint", selectedConstraint.equals(WristConstants.kFarConstraints) ? "FAR" : "CLOSE");

    WristController.setGoal(new State(targetAngleRadians, 0));
  }

  public boolean atGoal() {
    return WristController.atGoal();
  }

  private void setCalculatedVoltage() {
    double voltage = 
      WristController.calculate(getAngleRadians())
      + WristConstants.kWristFeedForward.calculate(WristController.getSetpoint().position, 0);
    SmartDashboard.putNumber("Wrist Voltage", voltage);

    WristMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    setCalculatedVoltage();

    // SmartDashboard.putNumber("Wrist Encoder Position", WristEncoder.getPosition());
    // SmartDashboard.putNumber("Wrist Encoder Velocity", WristEncoder.getVelocity());
    
    SmartDashboard.putNumber("Wrist Kinematic Angle", Units.radiansToDegrees(getAngleRadians()));
  }
}