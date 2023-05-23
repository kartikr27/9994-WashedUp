// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
  private CANSparkMax manipulatorMotor = new CANSparkMax(ManipulatorConstants.kManipulatorMotorCanId, MotorType.kBrushless);

  private enum ManipulatorState {
    INTAKING, OUTTAKING, HOLDING, STOPPED
  }

  private ManipulatorState manipulatorState = ManipulatorState.STOPPED;
  
  private Timer manipulatorRunningTimer = new Timer();

  /** Creates a new Manipulator. */
  public Manipulator() {
    manipulatorMotor.setSmartCurrentLimit(ManipulatorConstants.kManipulatorMotorCurrentLimit);
    manipulatorMotor.setInverted(true);
    manipulatorMotor.burnFlash();
  }
 
  public Command setIntake() {
    return new InstantCommand(() -> {
      manipulatorMotor.set(ManipulatorConstants.kIntakeMotorSpeed);
      if (manipulatorState != ManipulatorState.INTAKING) {
        manipulatorRunningTimer.reset();
        manipulatorRunningTimer.start();
      }
      manipulatorState = ManipulatorState.INTAKING;
    });
  }

  public Command setOuttake() {
    return new InstantCommand(() -> {
      manipulatorMotor.set(ManipulatorConstants.kOuttakeMotorSpeed);
      manipulatorState = ManipulatorState.OUTTAKING;
    });
  }

  public Command setHold() {
    return new InstantCommand(() -> {
      manipulatorMotor.set(ManipulatorConstants.kHoldMotorSpeed);
      manipulatorState = ManipulatorState.HOLDING;
    });
  }

  public Command setStop() {
    return new InstantCommand(() -> {
      manipulatorMotor.set(0);
      manipulatorState = ManipulatorState.STOPPED;
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Manipulator State", manipulatorState.toString());
  }
}