package org.robolancers321;

import org.robolancers321.commands.drivetrain.TeleopDrive;
import org.robolancers321.subsystems.drivetrain.Drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  Drivetrain drivetrain;

  XboxController controller;

  public RobotContainer() {
    this.drivetrain = Drivetrain.getInstance();

    this.controller = new XboxController(0);

    configureBindings();
  }

  private void configureBindings() {
    this.drivetrain.setDefaultCommand(new TeleopDrive(
      this.controller::getLeftY,
      this.controller::getLeftX,
      this.controller::getRightX
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
