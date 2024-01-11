package org.robolancers321;

import org.robolancers321.commands.drivetrain.TeleopDrive;
import org.robolancers321.commands.drivetrain.TuneDriveToTarget;
import org.robolancers321.subsystems.drivetrain.Drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  Drivetrain drivetrain;

  XboxController controller;

  public RobotContainer() {
    this.drivetrain = Drivetrain.getInstance();

    this.controller = new XboxController(0);

    configureBindings();
  }

  private void configureBindings() {
    this.drivetrain.setDefaultCommand(new TeleopDrive(this.controller));

    new Trigger(this.controller::getXButton).whileTrue(new TuneDriveToTarget(0.0, 1.0, 0.0));
    
    new Trigger(this.controller::getAButton).onTrue(new InstantCommand(this.drivetrain::zeroYaw));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
