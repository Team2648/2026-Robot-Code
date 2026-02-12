// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.CompetitionConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShooterSpeeds;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Elastic;
import frc.robot.utilities.Utilities;

public class RobotContainer {
    private PhotonVision vision;
    private Drivetrain drivetrain;
    private Hood hood;
    private Shooter shooter;

    private CommandXboxController driver;

    private SendableChooser<Command> autoChooser;

    private Timer shiftTimer;

    public RobotContainer() {
        vision = new PhotonVision();
        drivetrain = new Drivetrain(null);
        hood = new Hood();
        shooter = new Shooter();

        vision.addPoseEstimateConsumer(drivetrain::consumeVisualPose);

        driver = new CommandXboxController(OIConstants.kDriverControllerPort);

        shiftTimer = new Timer();
        shiftTimer.reset();

        configureBindings();
        configureShiftDisplay();

        if(AutoConstants.kAutoConfigOk) {
            autoChooser = AutoBuilder.buildAutoChooser();
            configureNamedCommands();
        }
    }
  
    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.drive(
                driver::getLeftY,
                driver::getLeftX, 
                driver::getRightX, 
                () -> true
            )
        );

        driver.a().whileTrue(
            drivetrain.lockRotationToHub(
                driver::getLeftY, 
                driver::getLeftX, 
                false // TODO Should this be true by default?
            ) 
        );

        shooter.setDefaultCommand(
            shooter.maintainSpeed(ShooterSpeeds.kHubSpeed)
        );

        hood.setDefaultCommand(hood.trackToAngle(() -> {
            Pose2d drivetrainPose = drivetrain.getPose();
            Pose2d hubPose = Utilities.getHubPose();

            double distance = drivetrainPose.getTranslation()
                .plus(CompetitionConstants.KRobotToShooter.getTranslation().toTranslation2d())
                .getDistance(hubPose.getTranslation());
                
            if(HoodConstants.kUseInterpolatorForAngle) {
                return HoodConstants.kDistanceToAngle.get(distance);
            } else {
                // TODO The average actual speeds isn't <i>really</i> the exit velocity of the ball
                // on a hooded shooter, based on documentation, it's more like 30-50% depending on
                // hood material, surface friction, etc.
                return Utilities.shotAngle(
                    shooter.getAverageActualSpeeds(), 
                    distance, 
                    CompetitionConstants.kHubGoalHeightMeters - ShooterConstants.kShooterHeightMeters, 
                    false
                );
            }
        }));
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand(
            "Drivetrain Set X", 
            drivetrain.setX()
        );

        NamedCommands.registerCommand(
            "Drivetrain Face Hub", 
            drivetrain.rotateToPose(
                Utilities.getHubPose(),
                false // TODO Should this be true by default?
            )
        );
    }

    public Command getAutonomousCommand() {
        if(AutoConstants.kAutoConfigOk) {
            return autoChooser.getSelected();
        } else {
            return new PrintCommand("Robot Config loading failed, autonomous disabled");
        }
    }

    private void configureShiftDisplay() {
        SmartDashboard.putStringArray(OIConstants.kCurrentActiveHub, OIConstants.kRedBlueDisplay);
        
        RobotModeTriggers.autonomous().onTrue(new InstantCommand(() -> {
            shiftTimer.stop();
            SmartDashboard.putStringArray(OIConstants.kCurrentActiveHub, OIConstants.kRedBlueDisplay);
        }));

        RobotModeTriggers.teleop().onTrue(new InstantCommand(() -> {
            Elastic.selectTab(OIConstants.kTeleopTab);
            shiftTimer.reset();
            shiftTimer.start();
        }));

        new Trigger(() -> shiftTimer.get() <= 10).onTrue(new InstantCommand(() -> {
            SmartDashboard.putStringArray(OIConstants.kCurrentActiveHub, OIConstants.kRedBlueDisplay);
        }));

        new Trigger(() -> shiftTimer.get() > 10 && shiftTimer.get() <= 35).onTrue(new InstantCommand(() -> {
            SmartDashboard.putStringArray(
                OIConstants.kCurrentActiveHub, 
                Utilities.whoHasFirstShift() == Alliance.Red ? OIConstants.kRedDisplay : OIConstants.kBlueDisplay
            );
        }));

        new Trigger(() -> shiftTimer.get() > 35 && shiftTimer.get() <= 60).onTrue(new InstantCommand(() -> {
            SmartDashboard.putStringArray(
                OIConstants.kCurrentActiveHub, 
                Utilities.whoHasFirstShift() == Alliance.Red ? OIConstants.kBlueDisplay : OIConstants.kRedDisplay
            );
        }));

        new Trigger(() -> shiftTimer.get() > 60 && shiftTimer.get() <= 85).onTrue(new InstantCommand(() -> {
            SmartDashboard.putStringArray(
                OIConstants.kCurrentActiveHub, 
                Utilities.whoHasFirstShift() == Alliance.Red ? OIConstants.kRedDisplay : OIConstants.kBlueDisplay
            );
        }));

        new Trigger(() -> shiftTimer.get() > 85 && shiftTimer.get() <= 110).onTrue(new InstantCommand(() -> {
            SmartDashboard.putStringArray(
                OIConstants.kCurrentActiveHub, 
                Utilities.whoHasFirstShift() == Alliance.Red ? OIConstants.kBlueDisplay : OIConstants.kRedDisplay
            );
        }));

        new Trigger(() -> shiftTimer.get() > 110).onTrue(new InstantCommand(() -> {
            SmartDashboard.putStringArray(OIConstants.kCurrentActiveHub, OIConstants.kRedBlueDisplay);
        }));
    }
}
