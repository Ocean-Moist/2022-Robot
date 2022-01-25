// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.outtake.ToggleShooter;
import frc.robot.lib.controllers.FightStick;
import frc.robot.subsystems.*;

public class RobotContainer {
  // CONTROLLERS
  public static JoystickButton xboxA;
  public static JoystickButton xboxB;
  public static JoystickButton xboxX;
  public static JoystickButton xboxY;
  public static JoystickButton xboxLB;
  public static JoystickButton xboxRB;
  public static JoystickButton xboxSquares;
  public static JoystickButton xboxHamburger;
  public static Trigger xboxLS;
  public static XboxController.Axis xboxRS;
  public static XboxController xboxController = new XboxController(Constants.OIConstants.xboxControllerPort);
  // SUBSYSTEMS
  public static DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  public static IntakeSubsystem intake = new IntakeSubsystem();
  public static IndexerSubsystem indexer = new IndexerSubsystem();
  public static LimelightSubsystem limelight = new LimelightSubsystem("limelight-two");
  public static OuttakeSubsystem outtake = new OuttakeSubsystem();

  // Sets up controllers, configures controllers, and sets the default drive mode (tank or arcade)
  public RobotContainer() {
    xboxButtonSetup();
    configureButtonBindings();

    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, xboxController)); // Check for Arcade or Tank
  }

  // Configures xbox buttons to commands
  private void configureButtonBindings() {
    FightStick.fightStickB.whenPressed(new ToggleShooter(outtake));
    // have fun with this - jason and jacob '22
    xboxHamburger.whenPressed(new FunctionalCommand(() -> drivetrain.resetGyro(), () -> {}, interrupted -> {}, () -> true, drivetrain)); // Reset gyro
    xboxY.whenPressed(new FunctionalCommand( () -> drivetrain.toggleShifter(), () -> {}, interrupted -> {}, () -> true, drivetrain)); // Toggle shifter
  }

  // Connects xbox buttons to button #'s for the driver station
  private void xboxButtonSetup() {
    xboxA = new JoystickButton(xboxController, 1);
    xboxB = new JoystickButton(xboxController, 2);
    xboxX = new JoystickButton(xboxController, 3);
    xboxY = new JoystickButton(xboxController, 4);
    xboxLB = new JoystickButton(xboxController, 5);
    xboxRB = new JoystickButton(xboxController, 6);
    xboxSquares = new JoystickButton(xboxController, 7);
    xboxHamburger = new JoystickButton(xboxController, 8);
    xboxLS = new Trigger();
  }

  // Disables all robot subsystems (Emergency only)
  private void disableAll() {
    drivetrain.disable();
    indexer.disable();
    intake.disable();
    limelight.disable();
    outtake.disable();
  }

  // Returns the robot's main autonomous command
  public Command getAutonomousCommand() {
    return null;
  }
}
