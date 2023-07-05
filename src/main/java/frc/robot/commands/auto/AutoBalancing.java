package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalancing extends CommandBase {

    private SwerveSubsystem m_Swerve;

    public AutoBalancing(SwerveSubsystem m_Swerve) {
        this.m_Swerve = m_Swerve;
        addRequirements(m_Swerve);
    }

    @Override
    public void execute() {
        double translationVal = m_Swerve.getPitch().getDegrees() > 0 ? -.45 : .45;
        m_Swerve.drive(translationVal, 0.0, 0.0, true);
    }

    @Override
    public void end(boolean interrupted) {
        m_Swerve.drive(0.0, 0.5, 0.0, false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_Swerve.getPitch().getDegrees()) < 7;
    }
}