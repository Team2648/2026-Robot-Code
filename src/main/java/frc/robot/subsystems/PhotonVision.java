package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CompetitionConstants;
import frc.robot.constants.PhotonConstants;
import frc.robot.utilities.PhotonVisionConfig;
import frc.robot.utilities.VisualPose;

/**
 * This "Subsystem" is not actually a Subsystem. The intent is for this to be treated as
 * a "resource", that is, something that is not inherently a physical mechanism to be controlled.
 * 
 * A "resource" in this instance should be thought of as something that can be safely shared
 * by other Subsystems generally without collision if more that one Subsystem requires the 
 * "resource" at any given time.
 * 
 * Resources should <i>NOT</i> produce Commands, they should not have a default Command.
 * Resources do not have behaviors, and because Commands are in of themselves behaviors,
 * this class should not have Commands. 
 * 
 * Part of the thinking behind creating the PhotonVision components this way is to rely
 * on the CommandScheduler to call periodic. If this weren't the case, some other subsystem
 * would have to manage calling for periodic updates, while still sharing the resource with 
 * other subsystems <i>somehow</i>. 
 * 
 * This class is dynamic, by adding or removing PhotonVisionConfig objects to the "configs"
 * List in the PhotonConstants file, you change what is set up internally in this class. 
 * 1 config means 1 camera, 1 estimator, 1 stored pipeline result, 2 configs means 2 cameras,
 * 2 estimators, etc. etc. 
 */
public class PhotonVision extends SubsystemBase {
    private PhotonCamera[] cameras;
    private PhotonPoseEstimator[] estimators;
    private List<PhotonPipelineResult> latestResults;

    private ArrayList<Consumer<VisualPose>> poseEstimateConsumers;

    public PhotonVision() {
        cameras = new PhotonCamera[PhotonConstants.configs.size()];
        estimators = new PhotonPoseEstimator[PhotonConstants.configs.size()];
        latestResults = new ArrayList<PhotonPipelineResult>();

        for(int i = 0; i < PhotonConstants.configs.size(); i++) {
            cameras[i] = new PhotonCamera(PhotonConstants.configs.get(i).cameraName());
            estimators[i] = new PhotonPoseEstimator(
                CompetitionConstants.kTagLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                PhotonConstants.configs.get(i).robotToCamera()
            );
            latestResults.add(null);
        }

        poseEstimateConsumers = new ArrayList<Consumer<VisualPose>>();
    }

    @Override
    public void periodic() {
        for(int i = 0; i < cameras.length; i++) {
            List<PhotonPipelineResult> results = cameras[i].getAllUnreadResults();

            if(!results.isEmpty()) {
                latestResults.set(i, results.get(results.size() - 1));

                Optional<EstimatedRobotPose> pose = estimators[i].update(latestResults.get(i));

                if(!pose.isEmpty()) {
                    VisualPose visualPose = new VisualPose(
                        cameras[i].getName(),
                        pose.get().estimatedPose.toPose2d(), 
                        pose.get().timestampSeconds
                    );

                    for(Consumer<VisualPose> consumer: poseEstimateConsumers) {
                        consumer.accept(visualPose);
                    }
                }
            }
        }
    }

    /**
     * Returns the best 3D pose for a given AprilTag ID as seen by the cameras on the robot.
     * 
     * All cameras fields of view are observed, if no camera can see the given tag ID, this
     * method will return Optional.empty().
     * 
     * Note that this method has no minimum confidence threshold for a tag. This means that 
     * if one camera thinks it sees the tag, even with very low confidence, it'll still return
     * some sort of pose. 
     * 
     * @param tagID The ID of the tag to look for in the latest results from all cameras
     * @return An Optional object containing a Pose3d object, or Optional.empty() if
     *   the tag is not present anywhere in the robots field of view. 
     */
    public Optional<Pose3d> getBestPoseForTag(int tagID) {
        PhotonVisionConfig config = null;
        Transform3d bestCameraToTarget = null;
        float bestConfidence = -1;

        for(int cameraIndex = 0; cameraIndex < latestResults.size(); cameraIndex++) {
            if(latestResults.get(cameraIndex) == null) {
                continue;
            }

            for(PhotonTrackedTarget target: latestResults.get(cameraIndex).getTargets()) {
                if(target.getFiducialId() != tagID || bestConfidence > target.getDetectedObjectConfidence()) {
                    continue;
                }

                config = PhotonConstants.configs.get(cameraIndex);
                bestCameraToTarget = target.bestCameraToTarget;
                bestConfidence = target.getDetectedObjectConfidence();
            }
        }

        if(bestCameraToTarget == null) {
            return Optional.empty();
        }

        // This is based on what PhotonVision does for multitag Pose estimation
        // See PhotonPoseEstimator.multiTagOnCoprocStrategy
        // TODO This doesn't currently account for the offset of the tag relative to say the hub
        // unclear if that offset amount will be important or not
        return Optional.of(Pose3d.kZero
            .plus(bestCameraToTarget.inverse())
            .relativeTo(CompetitionConstants.kTagLayout.getOrigin())
            .plus(config.robotToCamera().inverse()));
    }

    /**
     * Returns the best yaw for a given AprilTag ID as seen by the cameras on the robot.
     * 
     * All cameras fields of view are observed, if no camera can see the given tag ID, this
     * method will return OptionalDouble.empty()
     * 
     * Note that this method has no minimum confidence threshold for a tag. This means that
     * if one camera thinks it sees the tag, even with very low confidence, it'll still
     * return some sort of yaw value.
     * 
     * Note that the yaw value here is the yaw of the observed tag relative to the center
     * of the cameras image frame. 
     * 
     * @param tagID The ID of the tag to look for in the latest results from all cameras
     * @return An OptionalDouble object containing a Double representing the described yaw
     *   of the AprilTag specified, or OptionalDouble.empty() if the tag is not present
     *   anywhere in the robots field of view
     */
    public OptionalDouble getBestYawForTag(int tagID) {
        double bestTagYaw = -1;
        float bestConfidence = -1;

        for(PhotonPipelineResult result: latestResults) {
            if(result == null) {
                continue;
            }

            for(PhotonTrackedTarget target: result.getTargets()) {
                if(target.getFiducialId() != tagID || bestConfidence > target.getDetectedObjectClassID()) {
                    continue;
                }

                bestTagYaw = target.getYaw();
                bestConfidence = target.getDetectedObjectConfidence();
            }
        }

        if(bestConfidence == -1) {
            return OptionalDouble.empty();
        }

        return OptionalDouble.of(bestTagYaw);
    }

    /**
     * Add a Consumer of VisualPose records to the PhotonVision resource. 
     * 
     * Each consumer will receive a VisualPose object when any camera produces a new
     * VisualPose. 
     * 
     * The number of Poses produced in a given 20ms cycle is the same number as how many
     * cameras there are on the robot, assuming those cameras see enough tags to generate a pose, 
     * as currently all cameras configuration will generate a Pose2d
     * 
     * @param consumer The lambda, functional reference, or Consumer implementing object 
     *  that will consume Poses produced by the PhotonVision resource.
     */
    public void addPoseEstimateConsumer(Consumer<VisualPose> consumer) {
        poseEstimateConsumers.add(consumer);
    }
}
