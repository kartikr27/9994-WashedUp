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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;


public class Drivetrain extends SubsystemBase {

	public SwerveDrivePoseEstimator m_poseEstimator;
	private SwerveDriveOdometry m_odometry;
	private SwerveModule[] m_swerveModules;

	private AHRS m_gyro = new AHRS(Port.kMXP);

	private Field2d m_field;

	public Drivetrain() {
		m_swerveModules =
			new SwerveModule[] {
				new SwerveModule(0, SwerveConstants.FrontLeftModule.constants),
					new SwerveModule(1, SwerveConstants.FrontRightModule.constants),
					new SwerveModule(2, SwerveConstants.BackLeftModule.constants),
					new SwerveModule(3, SwerveConstants.BackRightModule.constants)
			};
		m_odometry = new SwerveDriveOdometry(SwerveConstants.m_driveKinematics, getYaw(), getPositions());

		m_poseEstimator =
			new SwerveDrivePoseEstimator(
				SwerveConstants.m_driveKinematics,
				getYaw(),
				getModulePositions(),
				new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

		m_field = new Field2d();
		SmartDashboard.putData("Field", m_field);
	}

	public void drive(double forward, double side, double rotation, boolean fieldRelative, boolean isOpenLoop) {
		SwerveModuleState[] swerveModuleStates = SwerveConstants.m_driveKinematics.toSwerveModuleStates(
			fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
				forward * SwerveConstants.maxSpeed,
				side * SwerveConstants.maxSpeed,
				rotation * SwerveConstants.maxAngularVelocity,
				getYaw()) :
			new ChassisSpeeds(
				forward * SwerveConstants.maxSpeed,
				side * SwerveConstants.maxSpeed,
				rotation * SwerveConstants.maxAngularVelocity));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);
	
		for (SwerveModule mod: m_swerveModules) {
			mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
		}
	}

	/* Used by SwerveControllerCommand in Auto */

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

		for (SwerveModule mod: m_swerveModules) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], false); // false
		}
	}

	public Pose2d getPose() {
		SmartDashboard.putNumber("pose X", m_odometry.getPoseMeters().getX());
		SmartDashboard.putNumber("pose Y", m_odometry.getPoseMeters().getY());
		SmartDashboard.putNumber("gyro angle", getYaw().getDegrees());
		return m_odometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		m_odometry.resetPosition(getYaw(), getPositions(), pose);
	}

	public SwerveModuleState[] getStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];

		for (SwerveModule mod: m_swerveModules) {
			states[mod.moduleNumber] = mod.getState();
		}

		return states;
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

	public Rotation2d getYaw() {
		return Rotation2d.fromDegrees(-m_gyro.getAngle());
	}

	public void zeroGyroscope() {
		zeroGyroscope(new Rotation2d(0));
	}

	public void zeroGyroscope(Rotation2d rotation) {
		m_gyro.setAngleAdjustment(rotation.getDegrees());
		m_gyro.reset();
		m_poseEstimator.resetPosition(
			rotation, getModulePositions(), new Pose2d(getPose().getTranslation(), rotation));
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];

		for (SwerveModule mod: m_swerveModules) {
			positions[mod.moduleNumber] = mod.getPosition();
		}

		return positions;
	}

	@Override
	public void periodic() {
		m_odometry.update(getYaw(), getPositions());
		m_field.setRobotPose(getPose());
		m_poseEstimator.update(getYaw(), getModulePositions());

		for (SwerveModule mod: m_swerveModules) {
			SmartDashboard.putNumber(
				"Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
			SmartDashboard.putNumber(
				"Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
			SmartDashboard.putNumber(
				"Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
		}
	}
}