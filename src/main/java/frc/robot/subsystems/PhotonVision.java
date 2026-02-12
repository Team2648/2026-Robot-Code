package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Consumer;
import java.util.stream.Stream;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.CompetitionConstants;
import frc.robot.constants.PhotonConstants;
import frc.robot.interfaces.IAprilTagProvider;
import frc.robot.utilities.PhotonVisionConfig;
import frc.robot.utilities.VisualPose;

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
            }

            Optional<EstimatedRobotPose> pose = estimators[i].update(latestResults.get(i));

            if(!pose.isEmpty()) {
                VisualPose visualPose = new VisualPose(
                    pose.get().estimatedPose.toPose2d(), 
                    pose.get().timestampSeconds
                );

                for(Consumer<VisualPose> consumer: poseEstimateConsumers) {
                    consumer.accept(visualPose);
                }
            }
        }
    }

    public void testMethod(int targetID) {
        Optional<PhotonTrackedTarget> target = latestResults.stream()
            .filter((p) -> p != null)
            .map(PhotonPipelineResult::getTargets)
            .map(List::stream)
            .reduce(Stream::concat)
            .get()
            .filter((p) -> p.getFiducialId() == targetID)
            .max(
                Comparator.comparingDouble((ptt) -> {
                    return (double)ptt.getDetectedObjectConfidence();
                })
            );

    }

    public void addPoseEstimateConsumer(Consumer<VisualPose> consumer) {
        poseEstimateConsumers.add(consumer);
    }

    /*public Trigger tagPrescenseTrigger(int targetTag) {
        return new Trigger(() -> {
            return List.of(latestResults).stream()
                .filter((p) -> p != null)
                .anyMatch((p) -> {
                    return p.getTargets().stream().map(PhotonTrackedTarget::getFiducialId).anyMatch((i) -> {
                        return i == targetTag;
                    });
                });
            });
    }*/
/* 
    @Override
    public OptionalDouble getTagDistanceFromCameraByID(int id) {
        if (latestResult == null) {
            return OptionalDouble.empty();
        }

        if (!latestResult.hasTargets()) {
            return OptionalDouble.empty();
        }

        Optional<PhotonTrackedTarget> desiredTarget = getTargetFromList(latestResult.getTargets(), id);
        
        if (desiredTarget.isEmpty()) {
            return OptionalDouble.empty();
        }

        return OptionalDouble.of(
            PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeightMeters, 
                CompetitionConstants.kTagLayout.getTagPose(id).get().getZ(),
                cameraPitchRadians, 
                Units.degreesToRadians(desiredTarget.get().getPitch()))
        );
    }

    @Override
    public OptionalDouble getTagPitchByID(int id) {
        if(latestResult == null) {
            OptionalDouble.empty();
        }
        
        if (!latestResult.hasTargets()) {
            return OptionalDouble.empty();
        }

        Optional<PhotonTrackedTarget> desiredTarget = getTargetFromList(latestResult.getTargets(), id);

        if (desiredTarget.isEmpty()) {
            return OptionalDouble.empty();
        }

        return OptionalDouble.of(
            desiredTarget.get().getPitch()
        );
    }

    @Override
    public OptionalDouble getTagYawByID(int id) {
        if(latestResult == null) {
            OptionalDouble.empty();
        }
        
        if (!latestResult.hasTargets()) {
            return OptionalDouble.empty();
        }

        Optional<PhotonTrackedTarget> desiredTarget = getTargetFromList(latestResult.getTargets(), id);

        if (desiredTarget.isEmpty()) {
            return OptionalDouble.empty();
        }

        return OptionalDouble.of(
            desiredTarget.get().getYaw()
        );
    }

    private Optional<PhotonTrackedTarget> getTargetFromList(List<PhotonTrackedTarget> targets, int id) {
        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() == id) {
                return Optional.of(target);
            }
        }

        return Optional.empty();
    }

    @Override
    public int[] getVisibleTagIDs() {
        if(latestResult == null) {
            return new int[] {};
        }

        return latestResult.getTargets().stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
    }
    */
}
