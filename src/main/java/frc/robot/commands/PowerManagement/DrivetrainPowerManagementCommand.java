package frc.robot.commands.PowerManagement;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PowerManagementSubsystem;

public class DrivetrainPowerManagementCommand extends CommandBase {
    private final PowerManagementSubsystem powerManagementSubsystem;

    public DrivetrainPowerManagementCommand(PowerManagementSubsystem powerManagementSubsystem) {
        this.powerManagementSubsystem = powerManagementSubsystem;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg
        // of Subsystem)
        addRequirements(this.powerManagementSubsystem);
    }

    @Override
    public void initialize() {
        powerManagementSubsystem.setDrivetrainCurrentLimit(45, 60);

    }

    @Override
    public void execute() {
        if (powerManagementSubsystem.getVoltage() < 7.2) {
            powerManagementSubsystem.setDrivetrainCurrentLimit(30,45);
            // send message to shuffleboard
            // 'ur fcked lmao drivetrain limiting'

        } else if (powerManagementSubsystem.getVoltage() > 8.0) {
            powerManagementSubsystem.setDrivetrainCurrentLimit(45,60);
        }

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
