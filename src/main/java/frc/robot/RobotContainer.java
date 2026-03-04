// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.constants.IntakePivotConstants.IntakePivotPosition;
import frc.robot.constants.ShooterConstants.ShooterSpeeds;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.utilities.Elastic;
import frc.robot.utilities.Utilities;

public class RobotContainer {
    private PhotonVision vision;
    private Drivetrain drivetrain;
    private Hood hood;
    private Shooter shooter;
    private IntakePivot intakePivot;
    private IntakeRoller intakeRoller;
    private Spindexer spindexer;
    //private Climber climber;

    private CommandXboxController driver;
    private CommandXboxController secondary;

    private SendableChooser<Command> autoChooser;

    private Timer shiftTimer;

    public RobotContainer() {
        vision = new PhotonVision();
        drivetrain = new Drivetrain(null);
        hood = new Hood();
        shooter = new Shooter();
        intakePivot = new IntakePivot();
        intakeRoller = new IntakeRoller();
        spindexer = new Spindexer();
        //climber = new Climber();

        
        vision.addPoseEstimateConsumer(drivetrain::consumeVisualPose);
        vision.addPoseEstimateConsumer((vp) -> {
            Logger.recordOutput(
                "Vision/" + vp.cameraName() + "/Pose", 
                vp.visualPose()
            );
        });

        driver = new CommandXboxController(OIConstants.kDriverControllerPort);
        secondary = new CommandXboxController(OIConstants.kOperatorControllerPort);

        shiftTimer = new Timer();
        shiftTimer.reset();

        //configureBindings();
        testConfigureBindings();
        configureShiftDisplay();

        if(AutoConstants.kAutoConfigOk) {
            autoChooser = AutoBuilder.buildAutoChooser();
            configureNamedCommands();
        }
    }

    /**
     * Before you @ mention me for terrible match controls, these are <i>TEST BINDINGS</i>
     * 
     * The are configured in such a way to make testing of multiple systems possible without
     * having to constantly change bindings.
     * 
     * Most of the configurations here won't make sense for actual competition play. See
     * {@link #configureBindings()} for actual competition play
     * 
     * The intent of each binding is outlined by comments above each binding. 
     */
    private void testConfigureBindings() {
        // This should just work, if it doesn't it's likely modules aren't assigned the right IDs
        // after the electronics rebuild. For testing normal operation nothing about the Drivetrain
        // class should need to change
        //drivetrain.setDefaultCommand(drivetrain.drive(() -> 0, () -> 0, () -> 0, () -> true));
        drivetrain.setDefaultCommand(
            drivetrain.drive(
                driver::getLeftY,
                driver::getLeftX, 
                driver::getRightX, 
                () -> true
            )
        );

        driver.y().whileTrue(drivetrain.zeroHeading());
/* 
        // This needs to be tested after a prolonged amount of driving around <i>aggressively</i>.
        // Do things like going over the bump repeatedly, spin around a bunch, etc. 
        // If this works well over time, then this is likely all we need
        driver.a().whileTrue(
            drivetrain.lockRotationToHub(
                driver::getLeftY, 
                driver::getLeftX, 
                false
            ) 
        );

        // This can be tested as an alternative to the above, it's less dynamic, but is a simpler
        // alternative. 
        driver.b().whileTrue(
            drivetrain.lockToYaw(
                () -> {
                    OptionalDouble maybeYaw = vision.getBestYawForTag(Utilities.getHubCenterAprilTagID());

                    return maybeYaw.isEmpty() ? 0 : maybeYaw.getAsDouble();
                }, 
                driver::getLeftY, 
                driver::getLeftX
            )
        );*/

        // Stop everything by default other than the drivetrain
        shooter.setDefaultCommand(shooter.stop());
        intakePivot.setDefaultCommand(intakePivot.stop());
        intakeRoller.setDefaultCommand(intakeRoller.stop());
        hood.setDefaultCommand(hood.stop());
        spindexer.setDefaultCommand(spindexer.stop());
        //climber.setDefaultCommand(climber.stop());

        // While holding POV up of the driver controller, the climber
        // should move such that its motor moves the climber down with the left
        // driver controller trigger axis, and up with the right driver controller
        // trigger axis.
        // DO NOT INVERT MOTION WITH UNARY MINUS (-). Every motor can be inverted
        // from the constants file for the subsystem having the problem. 
        //driver.povUp().whileTrue(climber.manualSpeed(() -> {
        //    return driver.getLeftTriggerAxis() * -1 + driver.getRightTriggerAxis();
        //}));

        // While holding the right bumper of the driver controller, the intake rollers
        // and the spindexer and feeder should move such that all motors are moving in such a way
        // that it would draw balls from the floor, through the spindexer, and into the
        // feeder. 
        // DO NOT INVERT MOTION WITH UNARY MINUS (-). Every motor can be inverted from the 
        // constants file for the subsystem having the problem
        driver.rightBumper().whileTrue(
            spindexer.spinToShooter()  
        );

        // While holding the left bumper of the driver controller, the intake rollers
        // and the spindexer and feeder should move such that all motors are moving in such a way
        // that it would try to eject balls through the intake.
        // DO NOT INVERT MOTION WITH UNARY MINUS (-). Every motor can be inverted from the 
        // constants file for the subsystem having the problem
        driver.leftBumper().whileTrue(
            intakeRoller.runIn()
            //intakeRoller.runOut().alongWith(spindexer.spinToIntake())
        );

        // While holding D-Pad up on the secondary controller, the shooter should spin
        // while holding down the secondary controllers right trigger some amount.
        // DO NOT INVERT MOTION WITH UNARY MINUS (-). Every motor can be inverted from the
        // constants file for the subsystem having the problem
        secondary.povUp().whileTrue(
            shooter.manualSpeed(secondary::getRightTriggerAxis)
        );

        // While holding D-Pad down on the seconadry controller, the intakePivot should move
        // such that left trigger on the secondary controller moves the pivot down, and 
        // right trigger on the secondary controller moves the pivot up.
        // DO NOT INVERT MOTION WITH UNARY MINUS (-). Every motor can be inverted from the
        // constants file for the subsystem having the problem
        secondary.povDown().whileTrue(
            intakePivot.manualSpeed(() -> {
                return secondary.getLeftTriggerAxis() * -1 + secondary.getRightTriggerAxis();
            })
        );

        // While holding D-Pad left on the secondary controller, the hood should move
        // such that the left trigger on the secondary controller moves the hood down, and
        // right trigger on the secondary controller moves the hood up.
        // DO NOT INVERT MOTION WITH UNARY MINUS (-). Every motor can be inverted from the 
        // constants file for the subsystem having the problem
        secondary.povLeft().whileTrue(
            hood.manualSpeed(() -> {
                return secondary.getLeftTriggerAxis() * -1 + secondary.getRightTriggerAxis();
            })
        );

        // STOP STOP STOP STOP STOP STOP STOP STOP STOP STOP STOP STOP STOP STOP STOP STOP
        // Don't proceed unless you've verified by hand or with the above bindings, that sensing
        // systems are providing correct values in the correct direction of rotation. 
    

        // Useful for testing PID and FF responses of the shooter
        // You need to have graphs up of the logged data to make sure the response is correct
        secondary.a().whileTrue(shooter.maintainSpeed(ShooterSpeeds.kHubSpeed));
        secondary.b().whileTrue(shooter.maintainSpeed(ShooterSpeeds.kFeedSpeed));

        // Useful for testing PID and FF responses of the intake pivot
        // You need to have graphs up of the logged data to make sure the response is correct
        secondary.x().whileTrue(intakePivot.maintainPosition(IntakePivotPosition.kDown));
        secondary.y().whileTrue(intakePivot.maintainPosition(IntakePivotPosition.kUp));

        secondary.povUp().whileTrue(hood.trackToAngle(() -> Math.toRadians(40.0)));
        secondary.povDown().whileTrue(hood.trackToAngle(() -> Math.toRadians(0.0)));
        

        // TODO Some means of testing hood PIDF
        // TODO Some means of testing climber PIDF
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
                false
            ) 
        );

        /*
        driver.b().whileTrue(
            drivetrain.lockToYaw(
                () -> {
                    OptionalDouble maybeYaw = vision.getBestYawForTag(Utilities.getHubCenterAprilTagID());

                    return maybeYaw.isEmpty() ? 0 : maybeYaw.getAsDouble();
                }, 
                driver::getLeftY, 
                driver::getLeftX
            )
        );*/

        shooter.setDefaultCommand(
            shooter.maintainSpeed(ShooterSpeeds.kHubSpeed)
        );

        hood.setDefaultCommand(hood.trackToAngle(() -> {
            Pose2d drivetrainPose = drivetrain.getPose();
            Pose2d hubPose = Utilities.getHubPose();

            double distance = drivetrainPose.getTranslation()
                .plus(CompetitionConstants.kRobotToShooter.getTranslation().toTranslation2d())
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

    /**
     * The "shift display" relies on Elastic's ability to show 1 or more colors
     * in a box on the dashboard. 
     * 
     * Using the RobotModeTriggers and a Timer based on the FPGA, a reasonably
     * accurate "shift display" can be created to indicate whose hub is active
     * and when. 
     * 
     * During autonomous, and the first 10 seconds of teleop, the shift display
     * will display a gradient of both red and blue, indicating the fact that 
     * both hubs are active.
     * 
     * For the rest of teleop, with the exception of the endgame, the display
     * will present either the color red, or the color blue, based on the returned
     * value of Utilities.whoHasFirstShift(). Because shifts change on a known cycle,
     * we can use the known state of who has first shift, to determine the remaining three shifts
     * that come after.
     * 
     * For the endgame portion of teleop, the shift display returns to the gradient
     * of both red and blue. 
     * 
     * Because this relies on the RobotModeTriggers and an FPGA timer, it should be 
     * <i>reasonably</i> accurate, it's unlikely to be perfect relative to field time
     * but it will be very very very (likely unnoticably) close. 
     */
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
