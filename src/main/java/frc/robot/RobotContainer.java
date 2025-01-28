// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSpark;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
 

public class RobotContainer {

  private final Drive drive;
  private final CommandXboxController controller = new CommandXboxController(0);

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive = new Drive(new DriveIOSpark(), new GyroIONavX());
        break;

      case SIM:
        drive = new Drive(new DriveIOSim(), new GyroIO() {
        });
        break;

      default:
        drive = new Drive(new DriveIO() {
        }, new GyroIO() {
        });
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    Logger.recordOutput("Drive/Controller/X Axis", controller.getRightX());
    Logger.recordOutput("Drive/Controller/Y Axis", controller.getLeftY());

    drive.setDefaultCommand(
        DriveCommands.arcadeDrive(
            drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    controller.x().onTrue(SysIdCommand());
  }

  public Command SysIdCommand() {
    return new SequentialCommandGroup(
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
        drive.sysIdDynamic(SysIdRoutine.Direction.kForward),
        drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
