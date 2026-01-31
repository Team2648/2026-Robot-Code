// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Elastic;
import frc.robot.utilities.Utilities;

public class RobotContainer {
  private Drivetrain drivetrain;

  private CommandXboxController driver;

  private SendableChooser<Command> autoChooser;

  private Timer shiftTimer;

  public RobotContainer() {
    drivetrain = new Drivetrain();

    driver = new CommandXboxController(OIConstants.kDriverControllerPort);

    shiftTimer = new Timer();
    shiftTimer.reset();

    configureBindings();
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

    driver.x().whileTrue(drivetrain.runFrontLeft());
    driver.y().whileTrue(drivetrain.runFrontRight());
    driver.a().whileTrue(drivetrain.runRearLeft());
    driver.b().whileTrue(drivetrain.runRearRight());

    //drivetrain.setDefaultCommand(drivetrain.disableOutputs());

    configureShiftDisplay();
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
        Utilities.ShiftFirst() == Alliance.Red ? OIConstants.kRedDisplay : OIConstants.kBlueDisplay
      );
    }));

    new Trigger(() -> shiftTimer.get() > 35 && shiftTimer.get() <= 60).onTrue(new InstantCommand(() -> {
      SmartDashboard.putStringArray(
        OIConstants.kCurrentActiveHub, 
        Utilities.ShiftFirst() == Alliance.Red ? OIConstants.kBlueDisplay : OIConstants.kRedDisplay
      );
    }));

    new Trigger(() -> shiftTimer.get() > 60 && shiftTimer.get() <= 85).onTrue(new InstantCommand(() -> {
      SmartDashboard.putStringArray(
        OIConstants.kCurrentActiveHub, 
        Utilities.ShiftFirst() == Alliance.Red ? OIConstants.kRedDisplay : OIConstants.kBlueDisplay
      );
    }));

    new Trigger(() -> shiftTimer.get() > 85 && shiftTimer.get() <= 110).onTrue(new InstantCommand(() -> {
      SmartDashboard.putStringArray(
        OIConstants.kCurrentActiveHub, 
        Utilities.ShiftFirst() == Alliance.Red ? OIConstants.kBlueDisplay : OIConstants.kRedDisplay
      );
    }));

    new Trigger(() -> shiftTimer.get() > 110).onTrue(new InstantCommand(() -> {
      SmartDashboard.putStringArray(OIConstants.kCurrentActiveHub, OIConstants.kRedBlueDisplay);
    }));
  }
}
