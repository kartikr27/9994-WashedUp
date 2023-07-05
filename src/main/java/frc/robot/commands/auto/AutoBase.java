// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auto;

// import java.util.HashMap;

// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.subsystems.SwerveSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class AutoBase extends SequentialCommandGroup {
//   private SwerveSubsystem m_drivetrain;

//   /** Creates a new AutoBase. */
//   public AutoBase(SwerveSubsystem m_drivetrain) {
//     this.m_drivetrain = m_drivetrain;
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands();
//   }

//   public SwerveAutoBuilder getSwerveAutoBuilder(HashMap<String, Command> AutoEventMap) {
//     return new SwerveAutoBuilder(
//       m_drivetrain::getPose, // pose2d supplier
//       m_drivetrain::resetOdometry, // reset odometry at the beginning of auto
//     DriveConstants.kDriveKinematics, // swerve kinematics
//       new PIDConstants(0, 0, 0), // x y controller
//       new PIDConstants(0, 0, 0), // theta controller
//       m_drivetrain::setModuleStates,
//       AutoEventMap,
//       true,
//       m_drivetrain);
//   }

//   public SwerveAutoBuilder getSwerveAutoBuilder() {
//     return new SwerveAutoBuilder(
//       m_drivetrain::getPose, // pose2d supplier
//       m_drivetrain::resetOdometry, // reset odometry at the beginning of auto
//       DriveConstants.kDriveKinematics, // swerve kinematics
//       new PIDConstants(0, 0, 0), // x y controller
//       new PIDConstants(0, 0, 0), // theta controller
//       m_drivetrain::setModuleStates,
//       new HashMap<String, Command>(),
//       true,
//       m_drivetrain);
//   }
// }