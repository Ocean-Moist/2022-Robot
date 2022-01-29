// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.auto.BetterAutoForwardDistance;
import frc.robot.commands.auto.components.AutoTurnAngle;
//import frc.robot.commands.drive.ToggleShifter;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.BetterAutoForwardDistance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.TankDrive;
<<<<<<< HEAD
import frc.robot.commands.intake.ToggleIntake;
=======
>>>>>>> upstream/main
import frc.robot.lib.controllers.FightStick;
import frc.robot.subsystems.*;

public class RobotContainer {
  public static JoystickButton xboxA;
  public static JoystickButton xboxB;
  public static JoystickButton xboxX;
  public static JoystickButton xboxY;
  public static JoystickButton xboxLB; // Define all the xbox controller functionality
  public static JoystickButton xboxRB;
  public static JoystickButton xboxSquares;
  public static JoystickButton xboxHamburger;
  public static Trigger xboxLS;
  public static XboxController.Axis xboxRS;

  public static XboxController xboxController = new XboxController(Constants.OIConstants.xboxControllerPort);

  public static DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  public static IntakeSubsystem intake = new IntakeSubsystem();
  public static IndexerSubsystem indexer = new IndexerSubsystem();
  public static LimelightSubsystem limelight = new LimelightSubsystem("limelight-two");
  public static OuttakeSubsystem outtake = new OuttakeSubsystem();

  public RobotContainer() {
    xboxButtonSetup();
    configureButtonBindings();

    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, xboxController)); // Check for Arcade or Tank
  }

  private void configureButtonBindings() {

  }

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

  private void disableAll() {
    drivetrain.disable();
    indexer.disable();
    intake.disable();
    limelight.disable();
    outtake.disable();
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
