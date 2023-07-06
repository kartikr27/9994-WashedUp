// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elbow;

public class ElbowWait extends CommandBase {
  /** Creates a new ElbowWait. */
  private final Arm m_Arm;
  private final Elbow m_Elbow;
  private final Rotation2d m_Setpoint;
  public ElbowWait(Arm arm, Elbow elbow, Rotation2d setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Arm=arm;
    this.m_Elbow=elbow;
    this.m_Setpoint=setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Elbow.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Arm.getAbsoluteRotation().getDegrees()<100){
      m_Elbow.setGoal(m_Setpoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elbow.stopElbow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Elbow.getController().atGoal();
  }
}
