// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elbow;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.Arm.ControlArm;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Elbow.ControlElbow;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

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

  public final XboxController d_controller = new XboxController(0);
  public final XboxController m_controller = new XboxController(1);

  CommandXboxController m_controllerCommand = new CommandXboxController(1);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final int armAxis = XboxController.Axis.kLeftY.value;
  private final int elbowAxis = XboxController.Axis.kRightY.value;
  
  private final Trigger armMove = m_controllerCommand.axisLessThan(armAxis, -0.08);
  private final Trigger armMove2 = m_controllerCommand.axisGreaterThan(armAxis, 0.08);

  private final Trigger elbowMove = m_controllerCommand.axisLessThan(elbowAxis, -0.08);
  private final Trigger elbowMove2 = m_controllerCommand.axisGreaterThan(elbowAxis, 0.08);

  private final JoystickButton zeroGyro =
      new JoystickButton(d_controller, XboxController.Button.kY.value);
  private final JoystickButton robotCentric =
      new JoystickButton(d_controller, XboxController.Button.kLeftBumper.value);

      private final JoystickButton runIntake =
      new JoystickButton(m_controller, XboxController.Button.kX.value);

  private final JoystickButton reverseIntake =
      new JoystickButton(m_controller, XboxController.Button.kB.value);


      private final POVButton setBotHigh = new POVButton(m_controller, 0);

      private final POVButton setBotMid = new POVButton(m_controller, 90);
    
      private final POVButton setBotIntake = new POVButton(m_controller, 180);
      private final POVButton setBotIntake2 = new POVButton(m_controller, 225);
    
      private final POVButton setBotInside = new POVButton(m_controller, 270);
    
      private final JoystickButton substationSetpoint =
          new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);



  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Arm m_Arm = new Arm();
  private final Elbow m_Elbow = new Elbow();
  private final Intake m_Intake = new Intake();

  public RobotContainer() {
    // Configure the trigger bindings
    m_drivetrain.setDefaultCommand(
        new SwerveDrive(
            m_drivetrain,
            () -> -d_controller.getRawAxis(translationAxis),
            () -> -d_controller.getRawAxis(strafeAxis),
            () -> -d_controller.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean()));

    configureBindings();

    
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

    zeroGyro.onTrue(new InstantCommand(() -> m_drivetrain.zeroGyroscope()));

	armMove.whileTrue(new ControlArm(m_Arm, m_controller));
    armMove2.whileTrue(new ControlArm(m_Arm, m_controller));

	elbowMove.whileTrue(new ControlElbow(m_Elbow, m_controller));
	elbowMove2.whileTrue(new ControlElbow(m_Elbow, m_controller));

  setBotHigh.whileTrue(new SetArm(m_Arm, Constants.Arm.ARM_SETPOINT_HIGH));

  runIntake.whileTrue(new RunIntake(m_Intake));


	
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
