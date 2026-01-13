package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.OIConstants;
import frc.robot.utilities.SwerveModule;

public class Drivetrain extends SubsystemBase {
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule rearLeft;
    private SwerveModule rearRight;

    private AHRS gyro;

    private SwerveDrivePoseEstimator estimator;

    public Drivetrain() {
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
            new Pose2d()
        );
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
        Logger.recordOutput("Drivetrain/Heading", getHeading());
    }

    public Command drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation, BooleanSupplier fieldRelative) {
        // TODO Inversions? Specific Alliance code?
        return run(() -> {
            drive(
                MathUtil.applyDeadband(xSpeed.getAsDouble(), OIConstants.kDriveDeadband), 
                MathUtil.applyDeadband(ySpeed.getAsDouble(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(rotation.getAsDouble(), OIConstants.kDriveDeadband), 
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

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        rearLeft.resetEncoders();
        rearRight.resetEncoders();
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

    public double getHeading() {
        return estimator.getEstimatedPosition().getRotation().getDegrees();
    }
}
