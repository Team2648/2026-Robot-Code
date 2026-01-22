package frc.robot.utilities;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.CompetitionConstants;
import frc.robot.interfaces.IAprilTagProvider;
import frc.robot.interfaces.IVisualPoseProvider;

public class PhotonVision implements IAprilTagProvider,IVisualPoseProvider {

    private final PhotonCamera camera;

    private final PhotonPoseEstimator photonPoseEstimator;

    private final double cameraHeightMeters;
    private final double cameraPitchRadians;

    private PhotonPipelineResult latestResult;

    public PhotonVision(String cameraName, Transform3d robotToCam, double cameraHeightMeters, double cameraPitchRadians) throws IOException {
        camera = new PhotonCamera(cameraName);

        photonPoseEstimator = new PhotonPoseEstimator(
            CompetitionConstants.kTagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            robotToCam
        );

        this.cameraHeightMeters = cameraHeightMeters;
        this.cameraPitchRadians = cameraPitchRadians;

        this.latestResult = null;
    }

    // TODO This periodic method has to be called from somewhere, even though the cameras
    // could be used in multiple other subsystems
    public void periodic() {
        // TODO Do we care about missed results? Probably not, if we're taking long enough to miss results something else is wrong
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if(!results.isEmpty()) {
            latestResult = results.get(results.size() - 1);
        }
    }

    @Override
    public Optional<VisualPose> getVisualPose() {
        if(latestResult == null) {
            return Optional.empty();
        }

        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(latestResult);

        if (pose.isEmpty()) {
            return Optional.empty();
        }

        return Optional.of(
            new VisualPose(
                pose.get().estimatedPose.toPose2d(), 
                pose.get().timestampSeconds
            )
        );
    }

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
    
}
