// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.ArmPreset;
import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class Arm {
    private final Elbow m_elbow = new Elbow();
    private final Wrist m_wrist = new Wrist();
    private final Manipulator m_manipulator = new Manipulator();

    public static enum CargoType {
		CONE, CUBE
	}

	public static enum IntakeMode {
		SHELF, PORTAL, FLOOR
	}

	public static enum ScoreLevel {
		HIGH, MIDDLE, LOW
	}

	public static enum RobotDirection {
		FORWARD, REVERSE
	}

	private CargoType cargoType = CargoType.CONE;
	private IntakeMode intakeMode = IntakeMode.FLOOR;
	private ScoreLevel scoreLevel = ScoreLevel.HIGH;
	private RobotDirection robotDirection = RobotDirection.FORWARD;

    public Arm() {}

    public CommandBase toPreset(ArmPreset armPreset) {
        CommandBase armCommand = new SequentialCommandGroup(
            new InstantCommand(() -> {
                m_elbow.setTargetRadians(armPreset.ElbowAngleRadians);
                m_wrist.setTargetRadians(armPreset.WristAngleRadians);
            }),
            new WaitUntilCommand(() -> m_elbow.atGoal() && m_wrist.atGoal())
        );
        armCommand.addRequirements(m_elbow, m_wrist);

        return armCommand;
    }

	public Command setCargoType(CargoType cargoType) {
		return new InstantCommand(() -> this.cargoType = cargoType);
	}

	public Command setIntakeMode(IntakeMode intakeMode) {
		return new InstantCommand(() -> this.intakeMode = intakeMode);
	}

	public Command setScoreLevel(ScoreLevel scoreLevel) {
		return new InstantCommand(() -> this.scoreLevel = scoreLevel);
	}

	public Command setRobotDirection(RobotDirection robotDirection) {
		return new InstantCommand(() -> this.robotDirection = robotDirection);
	}

	public Command getIntakeCommand() {
		return new SelectCommand(
				Map.ofEntries(
					Map.entry(IntakeMode.SHELF, new SelectCommand(
						Map.ofEntries(
							Map.entry(RobotDirection.FORWARD, new SelectCommand(
								Map.ofEntries(
									Map.entry(CargoType.CONE, toPreset(ArmConstants.kForwardShelfCone)),
									Map.entry(CargoType.CUBE, toPreset(ArmConstants.kForwardShelfCube))),
								() -> cargoType)),
							Map.entry(RobotDirection.REVERSE, new SelectCommand(
								Map.ofEntries(
									Map.entry(CargoType.CONE, toPreset(ArmConstants.kForwardShelfCone.getInverse())),
									Map.entry(CargoType.CUBE, toPreset(ArmConstants.kForwardShelfCube.getInverse()))),
								() -> cargoType))),
						() -> robotDirection)),
					Map.entry(IntakeMode.PORTAL, new SelectCommand(
						Map.ofEntries(
							Map.entry(RobotDirection.FORWARD, new SelectCommand(
								Map.ofEntries(
									Map.entry(CargoType.CONE, toPreset(ArmConstants.kForwardPortalCone)),
									Map.entry(CargoType.CUBE, toPreset(ArmConstants.kForwardPortalCube))),
								() -> cargoType)),
							Map.entry(RobotDirection.REVERSE, new SelectCommand(
								Map.ofEntries(
									Map.entry(CargoType.CONE, toPreset(ArmConstants.kForwardPortalCone.getInverse())),
									Map.entry(CargoType.CUBE, toPreset(ArmConstants.kForwardPortalCube.getInverse()))),
								() -> cargoType))),
						() -> robotDirection)),
					Map.entry(IntakeMode.FLOOR, new SelectCommand(
						Map.ofEntries(
							Map.entry(RobotDirection.FORWARD, new SelectCommand(
								Map.ofEntries(
									Map.entry(CargoType.CONE, toPreset(ArmConstants.kForwardFloorCone)),
									Map.entry(CargoType.CUBE, toPreset(ArmConstants.kForwardFloorCube))),
								() -> cargoType)),
							Map.entry(RobotDirection.REVERSE, new SelectCommand(
								Map.ofEntries(
									Map.entry(CargoType.CONE, toPreset(ArmConstants.kForwardFloorCone.getInverse())),
									Map.entry(CargoType.CUBE, toPreset(ArmConstants.kForwardFloorCube.getInverse()))),
								() -> cargoType))),
						() -> robotDirection))
				), () -> intakeMode)
			.alongWith(m_manipulator.setIntake());
	}
	
	
	public Command getScoreCommand() {
		return new SelectCommand(
				Map.ofEntries(
					Map.entry(ScoreLevel.HIGH, new SelectCommand(
						Map.ofEntries(
							Map.entry(RobotDirection.FORWARD, new SelectCommand(
								Map.ofEntries(
									Map.entry(CargoType.CONE, toPreset(ArmConstants.kForwardHighCone)),
									Map.entry(CargoType.CUBE, toPreset(ArmConstants.kForwardHighCube))),
								() -> cargoType)),
							Map.entry(RobotDirection.REVERSE, new SelectCommand(
								Map.ofEntries(
									Map.entry(CargoType.CONE, toPreset(ArmConstants.kForwardMiddleCone.getInverse())),
									Map.entry(CargoType.CUBE, toPreset(ArmConstants.kForwardMiddleCube.getInverse()))),
								() -> cargoType))),
						() -> robotDirection)),
					Map.entry(ScoreLevel.MIDDLE, new SelectCommand(
						Map.ofEntries(
							Map.entry(RobotDirection.FORWARD, new SelectCommand(
								Map.ofEntries(
									Map.entry(CargoType.CONE, toPreset(ArmConstants.kForwardLowCone)),
									Map.entry(CargoType.CUBE, toPreset(ArmConstants.kForwardLowCube))),
								() -> cargoType)),
							Map.entry(RobotDirection.REVERSE, new SelectCommand(
								Map.ofEntries(
									Map.entry(CargoType.CONE, toPreset(ArmConstants.kForwardPortalCone.getInverse())),
									Map.entry(CargoType.CUBE, toPreset(ArmConstants.kForwardPortalCube.getInverse()))),
								() -> cargoType))),
						() -> robotDirection)),
					Map.entry(ScoreLevel.LOW, new SelectCommand(
						Map.ofEntries(
							Map.entry(RobotDirection.FORWARD, new SelectCommand(
								Map.ofEntries(
									Map.entry(CargoType.CONE, toPreset(ArmConstants.kForwardFloorCone)),
									Map.entry(CargoType.CUBE, toPreset(ArmConstants.kForwardFloorCube))),
								() -> cargoType)),
							Map.entry(RobotDirection.REVERSE, new SelectCommand(
								Map.ofEntries(
									Map.entry(CargoType.CONE, toPreset(ArmConstants.kForwardFloorCone.getInverse())),
									Map.entry(CargoType.CUBE, toPreset(ArmConstants.kForwardFloorCube.getInverse()))),
								() -> cargoType))),
						() -> robotDirection))
				), () -> scoreLevel)
			.andThen(m_manipulator.setOuttake());
	}

	public Command getHoldCommand() {
		return toPreset(ArmConstants.kHoldPreset).alongWith(m_manipulator.setHold());
	}

    public void updateTelemetry() {
		SmartDashboard.putString("Cargo Type", cargoType.toString());
		SmartDashboard.putString("Intake Mode", intakeMode.toString());
		SmartDashboard.putString("Score Level", scoreLevel.toString());
		SmartDashboard.putString("Robot Direction", robotDirection.toString());
	}
}