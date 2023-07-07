// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalance extends CommandBase {
  /** Creates a new Balance2. */
  private final SwerveSubsystem m_Drivetrain;

  private double balanaceEffort;
  private double turningEffort;
  private boolean onGround;
  private boolean reversed;
  private Timer timer;

  private double prevAngle;

  private double kP;

  private PIDController thetaController;

  public AutoBalance(SwerveSubsystem drivetrain, boolean reversed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Drivetrain = drivetrain;
    this.reversed = reversed;
    addRequirements(drivetrain);

    thetaController =
        new PIDController(
            Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_KP,
            Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_KI,
            Constants.Autonomous.DRIVE_CONTROLLER_ROTATION_KD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onGround = true;
    timer = new Timer();
    timer.start();

    kP = Constants.Autonomous.AUTO_BALANCE_P_START;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_Drivetrain.getPitch());
    double angleDegrees = m_Drivetrain.getPitch().getDegrees();
    if (Math.abs(angleDegrees) >= Constants.Autonomous.AUTO_BALANCE_GROUND_ANGLE_THRESHOLD) {
      onGround = false;
    }

    if (onGround) {
      m_Drivetrain.drive(
          
              Constants.Autonomous.AUTO_BALANCE_GROUND_SPEED * (reversed ? -1 : 1), 0, 0,
          true);

    } else {
      System.out.println("ON CHARGER");
      if ((prevAngle < 0 && m_Drivetrain.getPitch().getDegrees() > 0)
          || (prevAngle > 0 && m_Drivetrain.getPitch().getDegrees() < 0)) {
        kP *= Constants.Autonomous.AUTO_BALANCE_P_MULTIPLIER;
        System.out.println("Reducing p " + kP);
      }

      turningEffort =
          calculteTheta(0.0);
      balanaceEffort = (Constants.Autonomous.balancedAngle - m_Drivetrain.getPitch().getDegrees()) * kP;
      m_Drivetrain.drive(balanaceEffort, 0, 0, false);
    }

    prevAngle = m_Drivetrain.getPitch().getDegrees();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.drive(0.0, 0.0, 0.5, false);
    setNTState(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!onGround && Math.abs(m_Drivetrain.getPitch().getDegrees()) < 2.5) {
      return timer.get() > 0.1;
    } else {
      timer.reset();
      return false;
    }
  }

  private void setNTState(boolean state) {
    SmartDashboard.getEntry("/auto/balance/state").setBoolean(state);
  }

  private double calculteTheta(double goalAngleRadians) {
    return thetaController.calculate(m_Drivetrain.getPose().getRotation().getRadians(), goalAngleRadians);
  }
}