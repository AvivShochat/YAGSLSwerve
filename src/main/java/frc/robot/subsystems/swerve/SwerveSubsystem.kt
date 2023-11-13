// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.swerve

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.vision.Vision
import swervelib.SwerveController
import swervelib.SwerveDrive
import swervelib.parser.SwerveDriveConfiguration
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity
import java.io.File
import frc.robot.subsystems.swerve.SwerveConstants as Constants

object SwerveSubsystem : SubsystemBase() {
	
	/**
	 * Swerve drive object.
	 */
	private val swerveDrive: SwerveDrive
	
	/**
	 * The auto builder for PathPlanner, there can only ever be one created so we save it just incase we generate multiple
	 * paths with events.
	 */
	private var autoBuilder: SwerveAutoBuilder? = null
	
	/**
	 * Initialize [SwerveDrive] with the directory provided.
	 *
	 * @param directory Directory of swerve drive config files.
	 */
	init {
		// Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH
		swerveDrive = try {
			SwerveParser(File(Filesystem.getDeployDirectory(), Constants.directoryPath)).createSwerveDrive()
		} catch (e: Exception) {
			throw RuntimeException(e)
		}
		
		swerveDrive.enableSecondOrderKinematics()
		SmartDashboard.putData(swerveDrive.field)
	}
	
	/**
	 * The primary method for controlling the drivebase.  Takes a [Translation2d] and a rotation rate, and
	 * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
	 * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
	 *
	 * @param translation   [Translation2d] that is the commanded linear velocity of the robot, in meters per
	 * second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
	 * torwards port (left).  In field-relative mode, positive x is away from the alliance wall
	 * (field North) and positive y is torwards the left wall when looking through the driver station
	 * glass (field West).
	 * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
	 * relativity.
	 * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
	 * @param isOpenLoop    Whether to use closed-loop velocity control.  Set to true to disable closed-loop.
	 */
	fun drive(translation: Translation2d?, rotation: Double, fieldRelative: Boolean, isOpenLoop: Boolean) {
		swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop)
	}
	
	val kinematics: SwerveDriveKinematics
		/**
		 * Get the swerve drive kinematics object.
		 *
		 * @return [SwerveDriveKinematics] of the swerve drive.
		 */
		get() = swerveDrive.kinematics
	
	/**
	 * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
	 * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
	 * keep working.
	 *
	 * @param initialHolonomicPose The pose to set the odometry to
	 */
	fun resetOdometry(initialHolonomicPose: Pose2d?) {
		swerveDrive.resetOdometry(initialHolonomicPose)
	}
	
	val pose: Pose2d
		/**
		 * Gets the current pose (position and rotation) of the robot, as reported by odometry.
		 *
		 * @return The robot's pose
		 */
		get() = swerveDrive.pose
	
	/**
	 * Set chassis speeds with closed-loop velocity control.
	 *
	 * @param chassisSpeeds Chassis Speeds to set.
	 */
	fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds?) {
		swerveDrive.setChassisSpeeds(chassisSpeeds)
	}
	
	/**
	 * Post the trajectory to the field.
	 *
	 * @param trajectory The trajectory to post.
	 */
	fun postTrajectory(trajectory: Trajectory?) {
		swerveDrive.postTrajectory(trajectory)
	}
	
	/**
	 * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
	 */
	fun zeroGyro() {
		swerveDrive.zeroGyro()
	}
	
	/**
	 * Sets the drive motors to brake/coast mode.
	 *
	 * @param brake True to set motors to brake mode, false for coast.
	 */
	fun setMotorBrake(brake: Boolean) {
		swerveDrive.setMotorIdleMode(brake)
	}
	
	val heading: Rotation2d
		/**
		 * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
		 *
		 * @return The yaw angle
		 */
		get() = swerveDrive.yaw
	
	/**
	 * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
	 * the angle of the robot.
	 *
	 * @param xInput   X joystick input for the robot to move in the X direction.
	 * @param yInput   Y joystick input for the robot to move in the Y direction.
	 * @param headingX X joystick which controls the angle of the robot.
	 * @param headingY Y joystick which controls the angle of the robot.
	 * @return [ChassisSpeeds] which can be sent to th Swerve Drive.
	 */
	fun getTargetSpeeds(xInput: Double, yInput: Double, headingX: Double, headingY: Double): ChassisSpeeds {
		var xInput = xInput
		var yInput = yInput
		xInput = Math.pow(xInput, 3.0)
		yInput = Math.pow(yInput, 3.0)
		return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, heading.radians)
	}
	
	/**
	 * Get the chassis speeds based on controller input of 1 joystick and one angle.
	 *
	 * @param xInput X joystick input for the robot to move in the X direction.
	 * @param yInput Y joystick input for the robot to move in the Y direction.
	 * @param angle  The angle in as a [Rotation2d].
	 * @return [ChassisSpeeds] which can be sent to th Swerve Drive.
	 */
	fun getTargetSpeeds(xInput: Double, yInput: Double, angle: Rotation2d): ChassisSpeeds {
		var xInput = xInput
		var yInput = yInput
		xInput = Math.pow(xInput, 3.0)
		yInput = Math.pow(yInput, 3.0)
		return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.radians, heading.radians)
	}
	
	val fieldVelocity: ChassisSpeeds
		/**
		 * Gets the current field-relative velocity (x, y and omega) of the robot
		 *
		 * @return A ChassisSpeeds object of the current field-relative velocity
		 */
		get() = swerveDrive.fieldVelocity
	
	val robotVelocity: ChassisSpeeds
		/**
		 * Gets the current velocity (x, y and omega) of the robot
		 *
		 * @return A [ChassisSpeeds] object of the current velocity
		 */
		get() = swerveDrive.robotVelocity
	val swerveController: SwerveController
		/**
		 * Get the [SwerveController] in the swerve drive.
		 *
		 * @return [SwerveController] from the [SwerveDrive].
		 */
		get() = swerveDrive.swerveController
	val swerveDriveConfiguration: SwerveDriveConfiguration
		/**
		 * Get the [SwerveDriveConfiguration] object.
		 *
		 * @return The [SwerveDriveConfiguration] fpr the current drive.
		 */
		get() = swerveDrive.swerveDriveConfiguration
	
	/**
	 * Lock the swerve drive to prevent it from moving.
	 */
	fun lock() {
		swerveDrive.lockPose()
	}
	
	val pitch: Rotation2d
		/**
		 * Gets the current pitch angle of the robot, as reported by the imu.
		 *
		 * @return The heading as a [Rotation2d] angle
		 */
		get() = swerveDrive.pitch
	
	/**
	 * Add a fake vision reading for testing purposes.
	 */
	fun addFakeVisionReading() {
		swerveDrive.addVisionMeasurement(
			Pose2d(3.0, 3.0, Rotation2d.fromDegrees(65.0)),
			Timer.getFPGATimestamp(),
			true,
			4.0
		)
	}
	
	private fun addVisionMeasurement() {
		Vision.estimatedGlobalPose.ifPresent() {
			swerveDrive.addVisionMeasurement(
				it.estimatedPose.toPose2d(), it.timestampSeconds, false, 0.5
			)
		}
	}
	
	/**
	 * Factory to fetch the PathPlanner command to follow the defined path.
	 *
	 * @param path             Path planner path to specify.
	 * @param constraints      [PathConstraints] for [com.pathplanner.lib.PathPlanner.loadPathGroup] function
	 * limiting velocity and acceleration.
	 * @param eventMap         [java.util.HashMap] of commands corresponding to path planner events given as
	 * strings.
	 * @param translation      The [PIDConstants] for the translation of the robot while following the path.
	 * @param rotation         The [PIDConstants] for the rotation of the robot while following the path.
	 * @param useAllianceColor Automatically transform the path based on alliance color.
	 * @return PathPlanner command to follow the given path.
	 */
	fun creatPathPlannerCommand(
		path: String?, constraints: PathConstraints?, eventMap: Map<String?, Command?>?,
		translation: PIDConstants?, rotation: PIDConstants?, useAllianceColor: Boolean
	): Command {
		val pathGroup = PathPlanner.loadPathGroup(path, constraints)
		//    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
//      Pose2d supplier,
//      Pose2d consumer- used to reset odometry at the beginning of auto,
//      PID constants to correct for translation error (used to create the X and Y PID controllers),
//      PID constants to correct for rotation error (used to create the rotation controller),
//      Module states consumer used to output to the drive subsystem,
//      Should the path be automatically mirrored depending on alliance color. Optional- defaults to true
//   )
		if (autoBuilder == null) {
			autoBuilder = SwerveAutoBuilder(
				{ swerveDrive.pose },
				{ pose: Pose2d? -> swerveDrive.resetOdometry(pose) },
				translation,
				rotation,
				{ chassisSpeeds: ChassisSpeeds? ->
					swerveDrive.setChassisSpeeds(
						chassisSpeeds
					)
				},
				eventMap,
				useAllianceColor,
				this
			)
		}
		return autoBuilder!!.fullAuto(pathGroup)
	}
	
	override fun simulationPeriodic() {}
	
	override fun periodic() {
//		addVisionMeasurement()
	}
}
