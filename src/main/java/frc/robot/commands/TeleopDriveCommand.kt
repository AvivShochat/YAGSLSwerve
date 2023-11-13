// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.swervedrive.drivebase

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.swerve.SwerveSubsystem
import swervelib.SwerveController
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

/**
 * An example command that uses an example subsystem.
 */
class TeleopDriveCommand(
	private val swerve: SwerveSubsystem,
	private val vX: DoubleSupplier,
	private val vY: DoubleSupplier,
	private val omega: DoubleSupplier,
	private val isFieldRelative: BooleanSupplier,
	private val isOpenLoop: Boolean,
	private val headingCorrection: Boolean
) : CommandBase() {
	private val controller: SwerveController
	private val timer = Timer()
	private var angle = 0.0
	private var lastTime = 0.0
	
	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param swerve The subsystem used by this command.
	 */
	init {
		controller = swerve.swerveController
		if (headingCorrection) {
			timer.start()
		}
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(swerve)
	}
	
	// Called when the command is initially scheduled.
	override fun initialize() {
		if (headingCorrection) {
			lastTime = timer.get()
		}
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	override fun execute() {
		val xVelocity = Math.pow(vX.asDouble, 3.0)
		val yVelocity = Math.pow(vY.asDouble, 3.0)
		val angVelocity = Math.pow(omega.asDouble, 3.0)
		SmartDashboard.putNumber("vX", xVelocity)
		SmartDashboard.putNumber("vY", yVelocity)
		SmartDashboard.putNumber("omega", angVelocity)
		if (headingCorrection) {
			// Estimate the desired angle in radians.
			angle += angVelocity * (timer.get() - lastTime) * controller.config.maxAngularVelocity
			// Get the desired ChassisSpeeds given the desired angle and current angle.
			val correctedChassisSpeeds = controller.getTargetSpeeds(
				xVelocity, yVelocity, angle,
				swerve.heading.radians
			)
			// Drive using given data points.
			swerve.drive(
				SwerveController.getTranslation2d(correctedChassisSpeeds),
				correctedChassisSpeeds.omegaRadiansPerSecond,
				isFieldRelative.asBoolean,
				isOpenLoop
			)
			lastTime = timer.get()
		} else {
			// Drive using raw values.
			swerve.drive(
				Translation2d(xVelocity * controller.config.maxSpeed, yVelocity * controller.config.maxSpeed),
				angVelocity * controller.config.maxAngularVelocity,
				isFieldRelative.asBoolean, isOpenLoop
			)
		}
	}
	
	// Called once the command ends or is interrupted.
	override fun end(interrupted: Boolean) {}
	
	// Returns true when the command should end.
	override fun isFinished(): Boolean {
		return false
	}
}