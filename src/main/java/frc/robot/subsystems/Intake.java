// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

private TalonFX intakeMotor2;





  // private CANSparkMax intakeMotor;
  // private PWMSparkMax intakeMotor;

  private GenericEntry currentIntakeEntry;
  private ShuffleboardTab intakeTab;
  // private CANSparkMax intakeMotorCAN;

  public Intake() {

    intakeMotor2 = new TalonFX(Constants.Intake.intakeMotor);

    intakeMotor2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 0, 0));
    intakeTab = Shuffleboard.getTab("Intake");

    currentIntakeEntry =
        intakeTab.add("Current Output Intake", intakeMotor2.getStatorCurrent()).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void runIntake(double speed) {

    intakeMotor2.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void stopIntake() {
    intakeMotor2.set(TalonFXControlMode.PercentOutput, 0);
  }

}
