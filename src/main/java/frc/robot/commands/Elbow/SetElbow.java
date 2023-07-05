// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elbow;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elbow;

public class SetElbow extends CommandBase {
  private final Elbow m_Elbow;

  private final Rotation2d m_setPoint;

  public SetElbow(Elbow Elbow, Rotation2d setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Elbow = Elbow;
    this.m_setPoint = setpoint;
    addRequirements(m_Elbow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Elbow.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Elbow.setGoal(m_setPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_Elbow.disable();
    m_Elbow.stopElbow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Elbow.getController().atGoal();
  }
}
