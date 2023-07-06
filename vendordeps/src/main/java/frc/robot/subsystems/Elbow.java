// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Elbow extends ProfiledPIDSubsystem {
  /** Creates a new Elbow2. */


  private CANSparkMax ElbowMotor;

  private ArmFeedforward elbowFeedForward;



  private RelativeEncoder relativeEncoder;

  private ShuffleboardTab ElbowTab;

  private GenericEntry voltageEntry;
  private GenericEntry positionEntry;
  private GenericEntry setpointEntry;
  private GenericEntry goalEntry;
  private GenericEntry velocityEntry;
  private GenericEntry velocitySetpointEntry;

  private Rotation2d prevPosition;
  private double prevTime;

  private int rotations;
  private double prevRadians;

  private double voltage = 0;

  public Elbow() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            Constants.Elbow.P,
            Constants.Elbow.I,
            Constants.Elbow.D,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                Constants.Elbow.ElbowMaxVelocity, Constants.Elbow.ElbowMaxAccel)));
    getController().setTolerance(Units.degreesToRadians(3));



    ElbowMotor = new CANSparkMax(Constants.Elbow.ElbowMotor, MotorType.kBrushless);


    ElbowMotor.setIdleMode(IdleMode.kBrake);
    ElbowMotor.setInverted(true);
    // relativeEncoder.setInverted(true);

    




elbowFeedForward =
new ArmFeedforward(Constants.Elbow.kS, Constants.Elbow.kG, Constants.Elbow.kV, Constants.Elbow.kA);

    relativeEncoder = ElbowMotor.getEncoder();

    prevPosition = getAbsoluteRotation();
    prevTime = Timer.getFPGATimestamp();

    ElbowTab = Shuffleboard.getTab("Elbow");
    positionEntry = ElbowTab.add("Positioning degrees", getBoreEncoder()).getEntry();

    voltageEntry = ElbowTab.add("Voltage", 0).withWidget(BuiltInWidgets.kVoltageView).getEntry();
    setpointEntry = ElbowTab.add("Setpoint", getController().getSetpoint().position).getEntry();
    goalEntry = ElbowTab.add("Goal", getController().getGoal().position).getEntry();
    velocityEntry = ElbowTab.add("Velocity", 0).getEntry();
    velocitySetpointEntry = ElbowTab.add("Velocity Setpoint", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    super.periodic();

    setpointEntry.setDouble(getAbsoluteRotation().getDegrees());
    goalEntry.setDouble(getController().getGoal().position);
    velocitySetpointEntry.setDouble(getController().getSetpoint().velocity);

    Rotation2d deltaPosition = getAbsoluteRotation().minus(prevPosition);
    double deltaTime = Timer.getFPGATimestamp() - prevTime;

    double degreesPerSecond = deltaPosition.getRadians() / deltaTime;
    velocityEntry.setDouble(degreesPerSecond);

    prevPosition = getAbsoluteRotation();
    prevTime = Timer.getFPGATimestamp();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here

    if (this.m_enabled) {
      voltage = output +elbowFeedForward.calculate(0.5 * Math.PI + setpoint.position, setpoint.velocity);

      voltage = MathUtil.clamp(voltage, -9.0, 9.0);

    
        ElbowMotor.setVoltage(voltage);
      

      voltageEntry.setDouble(voltage);
    }
  }

  public double getBoreEncoder() {
   

    return relativeEncoder.getPosition();
  }
  

  public Rotation2d getAbsoluteRotation() {
    // 1.25
    double radians = 2 * Math.PI * getBoreEncoder();


    return new Rotation2d(radians);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAbsoluteRotation().getRadians();
  }

  public void setGoal(Rotation2d rotation) {
    this.setGoal(rotation.getRadians());
  }

  public void setSpeed(double speed) {
    //disable();
    ElbowMotor.set(speed);
    
  }

  public double AbsoluteRadians() {
    double radians = 2 * Math.PI * getBoreEncoder();
    return radians;
  }

  public boolean softLimit(double value) {
    if (value > 0 && (getAbsoluteRotation().getDegrees() > 190)) {
      ElbowMotor.stopMotor();
      return true;
    }

    if (value < 0 && (getAbsoluteRotation().getDegrees() < -10)) {
      ElbowMotor.stopMotor();
      return true;
    }

    return false;
  }

  public void stopElbow() {
    ElbowMotor.stopMotor();
  }

  public void resetEncoder(){
    relativeEncoder.setPosition(0.0);
  }
}