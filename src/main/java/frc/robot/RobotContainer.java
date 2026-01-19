// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private Drivetrain drivetrain;

  private CommandXboxController driver;

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    drivetrain = new Drivetrain();

    driver = new CommandXboxController(OIConstants.kDriverControllerPort);

    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
      drivetrain.drive(
        driver::getLeftX, 
        driver::getLeftY, 
        driver::getRightX, 
        () -> true
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
}
