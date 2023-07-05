package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.AnalogInput;

public class SwerveModule {

    private CANSparkMax drivingMotor;
    private CANSparkMax turningMotor;

    private RelativeEncoder drivingEncoder;
    private RelativeEncoder turningEncoder;

    private CANCoder absoluteEncoder;
    private boolean absoluteEncoderReversed;
    private double absoluteEncoderOffsetRad;

    private PIDController drivePIDController;
    private PIDController turningPIDController;

    public SwerveModule(
            int drivingMotorID,
            int turningMotorID,
            boolean drivingMotorReversed,
            boolean turningMotorReversed,
            int absoluteEncoderID,
            double absoluteEncoderOffset,
            boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderID);

        drivingMotor = new CANSparkMax(drivingMotorID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        drivingMotor.setInverted(drivingMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        drivingEncoder = drivingMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        drivingMotor.setSmartCurrentLimit(80, 30);
        turningMotor.setSmartCurrentLimit(25);

        drivingEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPIDController = new PIDController(ModuleConstants.kTurnP, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return drivingEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return drivingEncoder.getVelocity();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition();
        angle = Math.toRadians(angle);
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }
    public double getAbsoluteEncoderDegrees() {
        double angle = absoluteEncoder.getAbsolutePosition();
        
        angle -= Math.toDegrees(absoluteEncoderOffsetRad);
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        drivingEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        drivingMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    }

    public void stop() {
        drivingMotor.set(0);
        turningMotor.set(0);
    }

}