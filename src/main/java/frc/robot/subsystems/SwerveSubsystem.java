package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

    public SwerveDrivePoseEstimator m_poseEstimator;
	private SwerveDriveOdometry m_odometry;
	private SwerveModule[] m_swerveModules;

    public boolean creepMode=false;

	private AHRS m_gyro = new AHRS(Port.kMXP);

	private Field2d m_field;

    private final SwerveModule frontLeft = new SwerveModule(0,
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
            true);

    private final SwerveModule frontRight = new SwerveModule(1,
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            true);

    private final SwerveModule backLeft = new SwerveModule(2,
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            true);

    private final SwerveModule backRight = new SwerveModule(3,
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            true);


    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        m_swerveModules = new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
        m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getYaw(), getPositions());

		m_poseEstimator =
			new SwerveDrivePoseEstimator(
				DriveConstants.kDriveKinematics,
				getYaw(),
				getModulePositions(),
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

		m_field = new Field2d();

		SmartDashboard.putData("Field", m_field);
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public void zeroGyro(){
        m_gyro.reset();
        // m_gyro.setAngleAdjustment(180.0);
        // 
    }
    public void setPose(Pose2d pose){
        m_odometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-getHeading());
    }


    public Rotation2d getYaw() {
		return Rotation2d.fromDegrees(-m_gyro.getAngle());
	}

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(m_gyro.getPitch());
    }

    public Field2d getField() {
        return m_field;
    }

    public Pose2d getPose() {
		SmartDashboard.putNumber("pose X", m_odometry.getPoseMeters().getX());
		SmartDashboard.putNumber("pose Y", m_odometry.getPoseMeters().getY());
		SmartDashboard.putNumber("gyro angle", getYaw().getDegrees());
        SmartDashboard.putNumber("neo current ",frontLeft.getCurrentDrive());
		return m_odometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		m_odometry.resetPosition(getYaw(), getPositions(), pose);
	}

    public SwerveModulePosition[] getPositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];

		for (SwerveModule mod: m_swerveModules) {
			SmartDashboard.putNumber("position: module " + mod.moduleNumber, mod.getPosition().distanceMeters);
			SmartDashboard.putNumber("angle: module " + mod.moduleNumber, mod.getPosition().angle.getDegrees());
			positions[mod.moduleNumber] = mod.getPosition();
		}
        

		return positions;
	}

    public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];

		for (SwerveModule mod: m_swerveModules) {
			positions[mod.moduleNumber] = mod.getPosition();
		}

		return positions;
	}

    public SwerveModuleState[] getStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];

		for (SwerveModule mod: m_swerveModules) {
			states[mod.moduleNumber] = mod.getState();
		}

		return states;
	}
    
    public void drive(double xSpeed, double ySpeed, double theta, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds;
        if (fieldRelative) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, theta, getRotation2d());
        } else {
            // Relative to robots
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, theta);
        }
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }
    public void creepModeTrue() {
        creepMode = true;
}

    public void creepModeFalse() {
        creepMode = false;
}

    @Override
    public void periodic() {

        m_odometry.update(getYaw(), getPositions());
		m_field.setRobotPose(getPose());
		m_poseEstimator.update(getYaw(), getModulePositions());
        
        SmartDashboard.putNumber("Robot Heading", getHeading());

        SmartDashboard.putNumber(
          "Front Right Candcoder", frontRight.getAbsoluteEncoderDegrees());
          SmartDashboard.putNumber(
            "Front Left Candcoder", frontLeft.getAbsoluteEncoderDegrees());
            SmartDashboard.putNumber(
                "Back Right Candcoder", backRight.getAbsoluteEncoderDegrees());
                SmartDashboard.putNumber(
                    "Back Left Candcoder", backLeft.getAbsoluteEncoderDegrees());

    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

  

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
    public CommandBase driveCommandBase(double xSpeed,double ySpeed, double turnSpeed,boolean fieldRelative){
        return this.run(() -> {
            drive(xSpeed, ySpeed, turnSpeed, fieldRelative);
        });
    }
}