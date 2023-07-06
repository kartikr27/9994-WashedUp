package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class TimedIntake extends CommandBase {

    private double time;
    private final Intake m_Intake;
    private Timer timer;
    private Direction direction;

    public enum Direction {
        INTAKE,
        OUTTAKE
    }

    public TimedIntake(
            Intake m_Intake, double time, Direction direction) {
        this.time = time;
        this.m_Intake = m_Intake;
        this.timer = new Timer();
        this.direction = direction;

        addRequirements(m_Intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
            if (direction == Direction.INTAKE) {
                m_Intake.runIntake(0.5);
            } else {
                m_Intake.runIntake(-0.5);
                }
    }

    @Override
    public void end(boolean interrupted) {

        timer.stop();
        timer.reset();

        m_Intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > time;
    }
}