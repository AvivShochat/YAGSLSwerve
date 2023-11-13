package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Transform3d
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy

object Vision {
	
	private val kRobotToCam = Transform3d()
	private val camera = PhotonCamera("")
	
	private val poseEstimator =
		PhotonPoseEstimator(
			AprilTagFields.kDefaultField.loadAprilTagLayoutField(),
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			camera,
			kRobotToCam
		);
	
	init {
		poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
		
	}
	
	val estimatedGlobalPose = poseEstimator.update()
}