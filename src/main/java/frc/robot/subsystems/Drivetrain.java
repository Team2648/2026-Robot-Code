package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.OIConstants;
import frc.robot.utilities.SwerveModule;
import frc.robot.utilities.Utilities;
import frc.robot.utilities.VisualPose;

public class Drivetrain extends SubsystemBase {
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule rearLeft;
    private SwerveModule rearRight;

    private AHRS gyro;

    private SwerveDrivePoseEstimator estimator;

    private PIDController yawRotationController;

    public Drivetrain(Pose2d startupPose) {
        frontLeft = new SwerveModule(
            "FrontLeft", 
            DrivetrainConstants.kFrontLeftDrivingCANID, 
            DrivetrainConstants.kFrontLeftTurningCANID, 
            DrivetrainConstants.kFrontLeftAnalogInPort, 
            DrivetrainConstants.kFrontLeftMagEncoderOffset
        );

        frontRight = new SwerveModule(
            "FrontRight", 
            DrivetrainConstants.kFrontRightDrivingCANID, 
            DrivetrainConstants.kFrontRightTurningCANID, 
            DrivetrainConstants.kFrontRightAnalogInPort, 
            DrivetrainConstants.kFrontRightMagEncoderOffset
        );

        rearLeft = new SwerveModule(
            "RearLeft", 
            DrivetrainConstants.kRearLeftDrivingCANID, 
            DrivetrainConstants.kRearLeftTurningCANID, 
            DrivetrainConstants.kRearLeftAnalogInPort, 
            DrivetrainConstants.kRearLeftMagEncoderOffset
        );

        rearRight = new SwerveModule(
            "RearRight", 
            DrivetrainConstants.kRearRightDrivingCANID, 
            DrivetrainConstants.kRearRightTurningCANID, 
            DrivetrainConstants.kRearRightAnalogInPort, 
            DrivetrainConstants.kRearRightMagEncoderOffset
        );

        gyro = new AHRS(NavXComType.kMXP_SPI);

        yawRotationController = new PIDController(
            AutoConstants.kPThetaController, 
            0, 
            0
        );
        yawRotationController.enableContinuousInput(-Math.PI, Math.PI);
        yawRotationController.setTolerance(AutoConstants.kYawPIDTolerance);

        // TODO 2025 used non-standard deviations for encoder/gyro inputs and vision, will need to be tuned for 2026 in the future
        estimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.kDriveKinematics, 
            Rotation2d.fromDegrees(getGyroValue()), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            },
            startupPose != null ? startupPose : new Pose2d()
        );

        if(AutoConstants.kAutoConfigOk) {
            AutoBuilder.configure(
                this::getPose, 
                this::resetOdometry, 
                this::getCurrentChassisSpeeds, 
                (speeds, feedforwards) -> driveWithChassisSpeeds(speeds), 
                AutoConstants.kPPDriveController, 
                AutoConstants.kRobotConfig,
                () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                }, 
                this
            );
        }
    }

    @Override
    public void periodic() {
        estimator.update(
            Rotation2d.fromDegrees(getGyroValue()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            }
        );

        frontLeft.periodic();
        frontRight.periodic();
        rearLeft.periodic();
        rearRight.periodic();

        Logger.recordOutput("Drivetrain/Pose", getPose());
        Logger.recordOutput("Drivetrain/Gyro Angle", getGyroValue());
        Logger.recordOutput("Drivetrain/Heading", getHeadingDegrees());
    }

    public Command runFrontLeft(double staticSpeed, double staticAngleDegrees) {
        return run(() -> {
            frontLeft.setDesiredState(new SwerveModuleState(
                staticSpeed, 
                Rotation2d.fromDegrees(staticAngleDegrees)));
        });
    }
    public Command runFrontRight(double staticSpeed, double staticAngleDegrees) {
        return run(() -> {
            frontRight.setDesiredState(new SwerveModuleState(
                staticSpeed, 
                Rotation2d.fromDegrees(staticAngleDegrees)));
        });
    }
    public Command runRearLeft(double staticSpeed, double staticAngleDegrees) {
        return run(() -> {
            rearLeft.setDesiredState(new SwerveModuleState(
                staticSpeed, 
                Rotation2d.fromDegrees(staticAngleDegrees)));
        });
    }
    public Command runRearRight(double staticSpeed, double staticAngleDegrees) {
        return run(() -> {
            rearRight.setDesiredState(new SwerveModuleState(
                staticSpeed, 
                Rotation2d.fromDegrees(staticAngleDegrees)));
        });
    }

    public Command disableOutputs() {
        return run(() -> {
            frontLeft.disableOutput();
            frontRight.disableOutput();
            rearLeft.disableOutput();
            rearRight.disableOutput();
        });  
    }

    /**
     * Rotates the robot to a face a given Pose2d position on the field
     * 
     * Note that this Command does not provide a means of timeout. If you are
     * using this in an auto context, this Command should be decorated with 
     * withTimeout(<some_value>). Otherwise, you will be waiting for the PID
     * Controller doing the work to report that it is at the desired setpoint. 
     * 
     * @param targetPose The Pose2d object to rotate the robot towards
     * @param rotate180 When false, the front of the robot faces the specified pose, when true
     *   the back of the robot faces the specified pose
     * @return A complete Command structure that performs the specified action
     */
    public Command rotateToPose(Pose2d targetPose, boolean rotate180) {
        return runOnce(yawRotationController::reset).andThen(
            drive(
                () -> 0,
                () -> 0, 
                () -> {
                    Rotation2d targetRotation = new Rotation2d(
                        targetPose.getX() - getPose().getX(), 
                        targetPose.getY() - getPose().getY()
                    );

                    if(rotate180) {
                        targetRotation = targetRotation.rotateBy(Rotation2d.k180deg);
                    }

                    return yawRotationController.calculate(
                        getHeading().getRadians(),
                        targetRotation.getRadians()
                    );
                }, 
                () -> true
            ))
            .until(yawRotationController::atSetpoint);
    }

    /**
     * Locks the robots rotation to face the Alliance Hub on the field. 
     * 
     * This method is innately aware of which hub to face based on the assigned alliance color.
     * 
     * This method is <i>NOT</i> for autonomous, see rotateToPose
     * 
     * This method provides a field oriented mechanism of driving the robot, such that the robot
     * is always facing the point on the field that is the center of the alliance hub. This
     * method assumes that the robots estimated pose is reasonably accurate. 
     * 
     * @param xSpeed The X (forward/backward) translational speed of the robot
     * @param ySpeed The Y (left/right) translational speed of the robot
     * @param rotate180 When false, the front of the robot faces the hub, when true, the back
     *   of the robot faces the hub
     * @return A complete Command structure that performs the specified action
     */
    public Command lockRotationToHub(DoubleSupplier xSpeed, DoubleSupplier ySpeed, boolean rotate180) {
        return runOnce(yawRotationController::reset).andThen(
            drive(
                xSpeed,
                ySpeed, 
                () -> {
                    Pose2d faceTowards = Utilities.getHubPose();

                    Rotation2d targetRotation = new Rotation2d(
                        faceTowards.getX() - getPose().getX(), 
                        faceTowards.getY() - getPose().getY()
                    );

                    if(rotate180) {
                        targetRotation = targetRotation.rotateBy(Rotation2d.k180deg);
                    }

                    return yawRotationController.calculate(
                        getHeading().getRadians(),
                        targetRotation.getRadians()
                    );
                }, 
                () -> true
            )
        );
    }

    // TODO check both cameras
    /*public Command driveAprilTagLock(DoubleSupplier xSpeed, DoubleSupplier ySpeed, double deadband, int tagID) {
        if (camera1 == null) {
            return new PrintCommand("Camera 1 not available");
        }

        // TODO The process variable is different here than what these constants are used for, may need to use something different
        PIDController controller = new PIDController(
            AutoConstants.kPThetaController, 
            0, 
            0
        );

        return runOnce(controller::reset).andThen(
            drive(
                xSpeed, 
                ySpeed, 
                () -> {
                    OptionalDouble tagYaw = camera1.getTagYawByID(tagID);

                    return (tagYaw.isEmpty() ? 0 : controller.calculate(tagYaw.getAsDouble(), 0));
                },
                () -> false
           )
        );
    }*/

    public Command drivePathPlannerPath(PathPlannerPath path) {
        if(AutoConstants.kAutoConfigOk) {
            return AutoBuilder.followPath(path);
        } else {
            return new PrintCommand("Robot Config loading failed, on the fly PathPlanner disabled");
        }
        
    }

    public Command drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation, BooleanSupplier fieldRelative) {
        // TODO Specific Alliance code?
        return run(() -> {
            drive(
                -MathUtil.applyDeadband(xSpeed.getAsDouble(), OIConstants.kDriveDeadband), 
                -MathUtil.applyDeadband(ySpeed.getAsDouble(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(rotation.getAsDouble(), OIConstants.kDriveDeadband), 
                fieldRelative.getAsBoolean()
            );
        });
    }

    public Command setX() {
        return run(() -> {
            frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
            frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        });
    }

    public void consumeVisualPose(VisualPose pose) {
        estimator.addVisionMeasurement(
            pose.visualPose(), 
            pose.timestamp()
        );
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        rearLeft.resetEncoders();
        rearRight.resetEncoders();
    }

    public void resetOdometry(Pose2d pose) {
        estimator.resetPosition(
            Rotation2d.fromDegrees(getGyroValue()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            },
            pose
        );
    }

    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
        double p = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
        double xSpeedDelivered = 0;
        double ySpeedDelivered = 0;

        if(p != 0){
            xSpeedDelivered = xSpeed * (Math.pow(p, OIConstants.kJoystickExponential) / p) * DrivetrainConstants.kMaxSpeedMetersPerSecond;
            ySpeedDelivered = ySpeed * (Math.pow(p, OIConstants.kJoystickExponential) / p) * DrivetrainConstants.kMaxSpeedMetersPerSecond;
        }else{
            xSpeedDelivered = 0;
            ySpeedDelivered = 0;
        }

        double rotationDelivered = rotation * DrivetrainConstants.kMaxAngularSpeed;

        SwerveModuleState[] swerveModuleStates = DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative ?
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered, 
                    estimator.getEstimatedPosition().getRotation()) :
                new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered)   
        );

        setModuleStates(swerveModuleStates);
    }

    public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.2);
        SwerveModuleState[] newStates = DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);

        setModuleStates(newStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DrivetrainConstants.kMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        return DrivetrainConstants.kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            rearLeft.getState(),
            rearRight.getState()
        );
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public double getGyroValue() {
        return gyro.getAngle() * (DrivetrainConstants.kGyroReversed ? -1 : 1);
    }

    public Rotation2d getHeading() {
        return estimator.getEstimatedPosition().getRotation();
    }

    public double getHeadingDegrees() {
        return estimator.getEstimatedPosition().getRotation().getDegrees();
    }
}
