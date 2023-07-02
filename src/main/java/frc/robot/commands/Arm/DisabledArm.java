package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class DisabledArm extends CommandBase {
  private Arm m_arm;

  public DisabledArm(Arm arm) {
    this.m_arm = arm;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_arm.disable();
    m_arm.stopArm();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
