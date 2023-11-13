package frc.robot

import edu.wpi.first.wpilibj.PS4Controller
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.swervedrive.drivebase.TeleopDriveCommand
import frc.robot.subsystems.swerve.SwerveSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer {
	
	private val swerve = SwerveSubsystem
	
	private val controller = PS4Controller(0)
	
	init {
		configureBindings()
		setDefaultCommands()
	}
	
	/** Use this method to define your `trigger->command` mappings. */
	private fun configureBindings() {
	
	}
	
	private fun setDefaultCommands() {
		swerve.defaultCommand = TeleopDriveCommand(
			swerve,
			vX = { controller.leftX },
			vY = { controller.leftY },
			omega = { controller.rightX },
			isFieldRelative = { false },
			isOpenLoop = false,
			headingCorrection = false
		)
	}
	
	fun getAutonomousCommand(): Command? {
		// TODO: Implement properly
		return null
	}
}