// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elbow;
import frc.robot.commands.ArmElbowSetpoints;
import frc.robot.commands.ReverseSequence;
import frc.robot.commands.SequentialSetpoint;
import frc.robot.commands.Drivetrain.SwerveDrive;
import frc.robot.commands.Arm.ControlArm;
import frc.robot.commands.Arm.IdleArm;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Elbow.ControlElbow;
import frc.robot.commands.Elbow.IdleElbow;
import frc.robot.commands.Elbow.SetElbow;
import frc.robot.commands.Intake.IdleIntake;
import frc.robot.commands.Intake.ReverseIntake;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.TimedIntake;
import frc.robot.commands.Intake.TimedIntake.Direction;
import frc.robot.commands.auto.AutoBalancing;
import frc.robot.commands.auto.AutoBase;
import frc.robot.commands.auto.OnePieceMobility;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  private SendableChooser<CommandBase> autonSelecter;

  public final XboxController d_controller = new XboxController(0);
  public final XboxController m_controller = new XboxController(1);

  CommandXboxController d_controllerCommand = new CommandXboxController(0);
  CommandXboxController m_controllerCommand = new CommandXboxController(1);



  private final int armAxis = XboxController.Axis.kLeftY.value;
  private final int elbowAxis = XboxController.Axis.kRightY.value;
  
  private final Trigger armMove = m_controllerCommand.axisLessThan(armAxis, -0.08);
  private final Trigger armMove2 = m_controllerCommand.axisGreaterThan(armAxis, 0.08);

  private final Trigger elbowMove = m_controllerCommand.axisLessThan(elbowAxis, -0.08);
  private final Trigger elbowMove2 = m_controllerCommand.axisGreaterThan(elbowAxis, 0.08);

  private final JoystickButton zeroGyro =
      new JoystickButton(d_controller, XboxController.Button.kA.value);

      private final JoystickButton reverseIntake =
      new JoystickButton(d_controller, XboxController.Button.kB.value);

      private final JoystickButton resetElbowEncoder =
      new JoystickButton(m_controller, XboxController.Button.kA.value);


      private final JoystickButton runIntake =
      new JoystickButton(m_controller, XboxController.Button.kX.value);

  

      private final int cubeModifyAxis = XboxController.Axis.kRightTrigger.value;
      private final Trigger cubeModify = m_controllerCommand.axisGreaterThan(cubeModifyAxis, 0.2);

      private final int manualControl = XboxController.Axis.kLeftTrigger.value;
      private final Trigger manualControlAxis = m_controllerCommand.axisGreaterThan(manualControl, 0.2);

      private final int creepModeAxis = XboxController.Axis.kRightTrigger.value;
      private final Trigger creepMode = d_controllerCommand.axisGreaterThan(creepModeAxis, 0.2);

      private final POVButton setBotHigh = new POVButton(m_controller, 0);

      private final POVButton setBotMidCone = new POVButton(m_controller, 90);
    
      private final POVButton setBotIntake = new POVButton(m_controller, 180);
    
      private final POVButton setBotInside = new POVButton(m_controller, 270);
    
      private final JoystickButton singleSubstationSetpoint =
          new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);

        private final JoystickButton doubleSubstationSetpoint =
          new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);



  private final SwerveSubsystem m_Swerve = new SwerveSubsystem();
  private final Arm m_Arm = new Arm();
  private final Elbow m_Elbow = new Elbow();
  private final Intake m_Intake = new Intake();
  
  /* Autonomous */
  private final SendableChooser<PathPlannerTrajectory> m_Chooser = new SendableChooser<>();
  private final AutoBase autoBase = new AutoBase(m_Swerve);
  private final SwerveAutoBuilder autoBuilder;
  private static Map<String, Command> eventMap;

  private final PathPlannerTrajectory flat2Piece = 
                PathPlanner.loadPath("Flat 2 Piece", 1, 3);
  
  private final PathPlannerTrajectory charge1ConeBalance = 
                PathPlanner.loadPath("Charge 1 Piece Bal", 1, 3);             



  public RobotContainer() {
    // Configure the trigger bindings
    boolean fieldRelative = true;
    m_Swerve.setDefaultCommand(new SwerveDrive(
        m_Swerve,
        () -> -d_controller.getRawAxis(OIConstants.kDriverYAxis),
        () -> -d_controller.getRawAxis(OIConstants.kDriverXAxis),
        () -> d_controller.getRawAxis(OIConstants.kDriverRotAxis), 
        fieldRelative));

    configureBindings();

    m_Intake.setDefaultCommand(new IdleIntake(m_Intake));
    m_Arm.setDefaultCommand(new IdleArm(m_Arm));
    m_Elbow.setDefaultCommand(new IdleElbow(m_Elbow));

    /* Autonomous */
    configureSmartDashboard();

    configureAutonomousEvents();
    
    autoBuilder = autoBase.getSwerveAutoBuilder(eventMap);

    autonSelecter = new SendableChooser<>();
    autonSelecter.setDefaultOption(null, autoBase);
    autonSelecter.addOption("One Piece Mobility", new OnePieceMobility(m_Swerve,m_Arm,m_Elbow,m_Intake));
    
    Shuffleboard.getTab("Auton").add(autonSelecter);
  }

  private void configureAutonomousEvents() {
    eventMap = new HashMap<>();

    eventMap.put(
            "AutoBalance",
            new AutoBalancing(m_Swerve));

    eventMap.put(
            "Eject",
            new TimedIntake(m_Intake, .5, Direction.OUTTAKE));

    eventMap.put(
            "Intake",
            new TimedIntake(m_Intake, 2.0, Direction.INTAKE));
    
    eventMap.put(
            "setHigh", 
            new SequentialSetpoint(m_Arm, Constants.Arm.ARM_SETPOINT_HIGH, m_Elbow, Constants.Elbow.ELBOW_SETPOINT_HIGH));

    eventMap.put(
            "setStow", 
            new SequentialSetpoint(m_Arm, Constants.Arm.ARM_SETPOINT_BOT, m_Elbow, Constants.Elbow.ELBOW_SETPOINT_BOT));
    
    eventMap.put(
            "setIntake", 
            new SequentialSetpoint(m_Arm, Constants.Arm.ARM_SETPOINT_GROUND_INTAKE_CONE, m_Elbow, Constants.Elbow.ELBOW_SETPOINT_GROUND_INTAKE_CONE));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    zeroGyro.onTrue(new InstantCommand(() -> m_Swerve.zeroGyro()));

    resetElbowEncoder.onTrue(new InstantCommand(() -> m_Elbow.resetEncoder()));
        armMove.and(manualControlAxis).whileTrue(new ControlArm(m_Arm, m_controller));
    armMove2.and(manualControlAxis).whileTrue(new ControlArm(m_Arm, m_controller));

	elbowMove.and(manualControlAxis).whileTrue(new ControlElbow(m_Elbow, m_controller));
	elbowMove2.and(manualControlAxis).whileTrue(new ControlElbow(m_Elbow, m_controller));


  runIntake.whileTrue(new RunIntake(m_Intake));

  reverseIntake.whileTrue(new ReverseIntake(m_Intake));

  //setBotInside.whileTrue(new ReverseSequence(m_Arm, Constants.Arm.ARM_SETPOINT_BOT, m_Elbow, Constants.Elbow.ELBOW_SETPOINT_BOT));
setBotInside.whileTrue(new ArmElbowSetpoints(m_Arm, Constants.Arm.ARM_SETPOINT_BOT, m_Elbow, Constants.Elbow.ELBOW_SETPOINT_BOT));
  setBotHigh.whileTrue(new SequentialSetpoint(m_Arm, Constants.Arm.ARM_SETPOINT_HIGH, m_Elbow, Constants.Elbow.ELBOW_SETPOINT_HIGH));

  
  setBotMidCone.whileTrue(new SequentialSetpoint(m_Arm, Constants.Arm.ARM_SETPOINT_MID_CONE, m_Elbow, Constants.Elbow.ELBOW_SETPOINT_MID_CONE));
  setBotMidCone.and(cubeModify).whileTrue(new ArmElbowSetpoints(m_Arm, Constants.Arm.ARM_SETPOINT_MID_CUBE, m_Elbow, Constants.Elbow.ELBOW_SETPOINT_MID_CUBE));

  setBotIntake.whileTrue(new SequentialSetpoint(m_Arm, Constants.Arm.ARM_SETPOINT_GROUND_INTAKE_CONE, m_Elbow, Constants.Elbow.ELBOW_SETPOINT_GROUND_INTAKE_CONE));

  setBotIntake.and(cubeModify).whileTrue(new SequentialSetpoint(m_Arm, Constants.Arm.ARM_SETPOINT_GROUND_INTAKE_CUBE, m_Elbow, Constants.Elbow.ELBOW_SETPOINT_GROUND_INTAKE_CUBE));
  

  doubleSubstationSetpoint.whileTrue(new SequentialSetpoint(m_Arm, Constants.Arm.ARM_SETPOINT_DOUBLE_SUBSTATION, m_Elbow, Constants.Elbow.ELBOW_SETPOINT_DOUBLE_SUBSTATION));
  singleSubstationSetpoint.whileTrue(new ArmElbowSetpoints(m_Arm, Constants.Arm.ARM_SETPOINT_SINGLE_SUBSTATION, m_Elbow, Constants.Elbow.ELBOW_SETPOINT_SINGLE_SUBSTATION));

  d_controllerCommand.leftBumper().onTrue(new InstantCommand(() -> m_Swerve.creepModeTrue()));
  d_controllerCommand.leftBumper().onFalse(new InstantCommand(() -> m_Swerve.creepModeFalse()));
  
  }
public void configureSmartDashboard() {
    m_Chooser.addOption("Flat Side: 2 Piece", flat2Piece);
    m_Chooser.addOption("Charge Station: 1 Piece+Balance", charge1ConeBalance);
    SmartDashboard.putData("Auto Choices", m_Chooser);
}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Executes the autonomous command chosen in smart dashboard
    return new ParallelCommandGroup(
            new InstantCommand(
                    () -> m_Swerve.getField().getObject("Field").setTrajectory(m_Chooser.getSelected())),
            autoBuilder.fullAuto(m_Chooser.getSelected()));
}

public Command getJankAuton(){
        return autonSelecter.getSelected();
}
}
