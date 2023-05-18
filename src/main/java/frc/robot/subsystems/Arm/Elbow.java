// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.controller.AsymmetricProfiledPIDController;
import frc.lib.util.controller.AsymmetricTrapezoidProfile.Constraints;
import frc.lib.util.controller.AsymmetricTrapezoidProfile.State;
import frc.robot.Constants.ElbowConstants;


public class Elbow extends SubsystemBase {
  private CANSparkMax RightElbowMotor;
  private CANSparkMax LeftElbowMotor;
  private AbsoluteEncoder ElbowEncoder;

  private AsymmetricProfiledPIDController ElbowController = 
    new AsymmetricProfiledPIDController(0,0,0, ElbowConstants.kFarConstraints); //MUST START AT 0 P
  
  /** Creates a new Elbow. */
  public Elbow() {
    LeftElbowMotor = new CANSparkMax(ElbowConstants.kLeftElbowMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    RightElbowMotor = new CANSparkMax(ElbowConstants.kRightElbowMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    LeftElbowMotor.follow(RightElbowMotor, true);
    RightElbowMotor.setInverted(ElbowConstants.kElbowEncoderInverted); //must be inverted
    RightElbowMotor.setIdleMode(IdleMode.kBrake);
    LeftElbowMotor.setIdleMode(IdleMode.kBrake);
    RightElbowMotor.setSmartCurrentLimit(ElbowConstants.kElbowMotorCurrentLimit);
    LeftElbowMotor.setSmartCurrentLimit(ElbowConstants.kElbowMotorCurrentLimit);

    ElbowEncoder = RightElbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
    ElbowEncoder.setPositionConversionFactor(ElbowConstants.kElbowPositionConversionFactor);
    ElbowEncoder.setInverted(ElbowConstants.kElbowEncoderInverted); //must be inverted
    ElbowEncoder.setZeroOffset(ElbowConstants.kElbowEncoderZeroOffset);
    //todo set velocity conversion factor

    RightElbowMotor.burnFlash();
    LeftElbowMotor.burnFlash();

    ElbowController.disableContinuousInput();
  }

  public double getAngleRadians() {
    return (ElbowEncoder.getPosition() - ElbowConstants.kElbowKinematicOffset) / ElbowConstants.kElbowGearRatio;
  }

  public void setTargetRadians(double targetAngleRadians) {
    SmartDashboard.putNumber("Elbow Target Angle", Units.radiansToDegrees(targetAngleRadians));
    ElbowController.setP(ElbowConstants.kElbowP);
    Constraints selectedConstraint = 
      (Math.abs(targetAngleRadians - getAngleRadians()) < Units.degreesToRadians(20)) ? 
      ElbowConstants.kCloseConstraints : ElbowConstants.kFarConstraints;
    ElbowController.setConstraints(selectedConstraint);
    SmartDashboard.putString("Elbow Selected Constraint", selectedConstraint.equals(ElbowConstants.kFarConstraints) ? "FAR" : "CLOSE");

    ElbowController.setGoal(new State(targetAngleRadians, 0));
  }

  public boolean atGoal() {
    return ElbowController.atGoal();
  }

  private void setCalculatedVoltage() {
    double voltage =
      ElbowController.calculate(getAngleRadians())
      + ElbowConstants.kElbowFeedForward.calculate(ElbowController.getSetpoint().position, 0);
    SmartDashboard.putNumber("Elbow Voltage", voltage);

    RightElbowMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    setCalculatedVoltage();

    // SmartDashboard.putNumber("Elbow Encoder Position", ElbowEncoder.getPosition());
    // SmartDashboard.putNumber("Elbow Encoder Velocity", ElbowEncoder.getVelocity());
    
    SmartDashboard.putNumber("Elbow Kinematic Angle", Units.radiansToDegrees(getAngleRadians()));
  }
}