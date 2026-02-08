package frc.robot.utilities;

import edu.wpi.first.math.geometry.Transform3d;

public record PhotonVisionConfig (
    String cameraName,
    Transform3d robotToCamera,
    double cameraHeightMeters,
    double cameraPitchRadians
) {}
