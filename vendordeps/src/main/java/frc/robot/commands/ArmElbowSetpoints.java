// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Elbow.SetElbow;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elbow;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmElbowSetpoints extends ParallelCommandGroup {
  /** Creates a new ScoreHigh. */
  private Arm m_Arm;
  private Elbow m_Elbow;
  private Rotation2d m_armAngle;
  private Rotation2d m_elbowAngle;

  public ArmElbowSetpoints(Arm arm, Rotation2d armAngle, Elbow elbow, Rotation2d elbowAngle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_Arm=arm;
    this.m_Elbow=elbow;
    this.m_armAngle=armAngle;
    this.m_elbowAngle=elbowAngle;
    addCommands(new SetArm(arm, armAngle),new SetElbow(elbow, elbowAngle));
  }
}
