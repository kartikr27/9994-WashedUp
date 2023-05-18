// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Arm.CargoType;
import frc.robot.subsystems.Arm.Arm.IntakeMode;
import frc.robot.subsystems.Arm.Arm.RobotDirection;
import frc.robot.subsystems.Arm.Arm.ScoreLevel;

public class RobotContainer {
  CommandXboxController m_driverController = new CommandXboxController(0);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Arm m_arm = new Arm();

  public RobotContainer() {
    // Configure the trigger bindings
    m_drivetrain.setDefaultCommand(
        new SwerveDrive(
            m_drivetrain,
            () -> -m_driverController.getRawAxis(translationAxis),
            () -> -m_driverController.getRawAxis(strafeAxis),
            () -> -m_driverController.getRawAxis(rotationAxis),
            () -> false));

            configureButtonBindings();
  }

  private void configureButtonBindings() {
		DriverStation.silenceJoystickConnectionWarning(true);

		m_driverController.rightBumper()
			.onTrue(m_arm.setCargoType(CargoType.CONE));
		m_driverController.leftBumper()
			.onTrue(m_arm.setCargoType(CargoType.CUBE));

		m_driverController.povUp()
			.onTrue(m_arm.setIntakeMode(IntakeMode.SHELF));
		m_driverController.povRight()
			.onTrue(m_arm.setIntakeMode(IntakeMode.PORTAL));
		m_driverController.povDown()
			.onTrue(m_arm.setIntakeMode(IntakeMode.FLOOR));

		m_driverController.y()
			.onTrue(m_arm.setScoreLevel(ScoreLevel.HIGH));
		m_driverController.b()
			.onTrue(m_arm.setScoreLevel(ScoreLevel.MIDDLE));
		m_driverController.a()
			.onTrue(m_arm.setScoreLevel(ScoreLevel.LOW));

		m_driverController.start()
			.onTrue(m_arm.setRobotDirection(RobotDirection.FORWARD));
		m_driverController.back()
			.onTrue(m_arm.setRobotDirection(RobotDirection.REVERSE));

		m_driverController.leftTrigger().debounce(0.2)
			.onTrue(m_arm.getIntakeCommand())
			.onFalse(m_arm.getHoldCommand());
		m_driverController.rightTrigger().debounce(0.2)
			.onTrue(m_arm.getScoreCommand())
			.onFalse(m_arm.getHoldCommand());
	}

  public Command getAutonomousCommand() {
		return new InstantCommand();
	}

}
