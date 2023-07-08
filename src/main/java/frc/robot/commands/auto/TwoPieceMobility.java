// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ReverseSequence;
import frc.robot.commands.SequentialSetpoint;
import frc.robot.commands.Drivetrain.SwerveDrive;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceMobility extends SequentialCommandGroup {
  /** Creates a new OnePieceMobility. */

  public TwoPieceMobility(SwerveSubsystem swerve,Arm arm,Elbow elbow,Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Timer timer = new Timer();
    
    addCommands(
    new InstantCommand(() -> {swerve.zeroGyro();}), new InstantCommand(() -> {
        swerve.setPose(new Pose2d(0,0,Rotation2d.fromDegrees(180)));}),
    new RunIntake(intake, 0.2).withTimeout(0.5), 
    new SequentialSetpoint(arm, Constants.Arm.ARM_SETPOINT_HIGH, elbow, Constants.Elbow.ELBOW_SETPOINT_HIGH),
    new RunIntake(intake,Constants.Intake.reverseIntakeSpeed).withTimeout(0.2),
    new ReverseSequence(arm, Constants.Arm.ARM_SETPOINT_BOT, elbow, Constants.Elbow.ELBOW_SETPOINT_BOT),
    new InstantCommand(()-> {timer.start();}), swerve.driveCommandBase(2.0, 0, 0, true).until(() -> {return timer.get()>2.5;})
        .alongWith(new SequentialSetpoint(arm, Constants.Arm.ARM_SETPOINT_GROUND_INTAKE_CONE, elbow, Constants.Elbow.ELBOW_SETPOINT_GROUND_INTAKE_CONE))
        .alongWith(new RunIntake(intake, Constants.Intake.coneIntakeSpeed).withTimeout(2.75)),
        new InstantCommand(()-> {timer.start();}), swerve.driveCommandBase(-3.0, 0, 0, true).until(() -> {return timer.get()>1.75;})
        .alongWith(new SequentialSetpoint(arm, Constants.Arm.ARM_SETPOINT_MID_CUBE, elbow, Constants.Elbow.ELBOW_SETPOINT_MID_CUBE)),
    new RunIntake(intake,Constants.Intake.reverseIntakeSpeed).withTimeout(0.5)
    );
  }
}
