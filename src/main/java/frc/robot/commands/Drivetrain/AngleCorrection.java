// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AngleCorrection extends CommandBase {
  /** Creates a new AngleCorrection. */

  private final SwerveSubsystem m_drivetrain;

  public static final double MaxVel = Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
  public static final double AngVel = 2 * Math.PI;


  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(AngVel, Math.pow(AngVel, 2));
      private final ProfiledPIDController omegaController =
      new ProfiledPIDController(0.05, 0, 0, OMEGA_CONSTRAINTS);
  public AngleCorrection(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = swerve;
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thVel = 0.0;
    omegaController.setGoal(0.001);
    thVel = omegaController.calculate(m_drivetrain.getYaw().getRadians());
    m_drivetrain.drive(0.0, 0.0, thVel, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return omegaController.atGoal();
  }
}
