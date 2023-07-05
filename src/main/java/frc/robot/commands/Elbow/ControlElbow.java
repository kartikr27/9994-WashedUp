// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elbow;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elbow;

public class ControlElbow extends CommandBase {
  private final Elbow m_Elbow;

  private final XboxController m_Controller;

  public ControlElbow(Elbow elbow, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Elbow = elbow;
    this.m_Controller = controller;
    addRequirements(m_Elbow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Elbow.disable();
    m_Elbow.setSpeed(m_Controller.getRightY()*0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elbow.stopElbow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
