package frc.robot.commands.PowerManagement;



public class DrivetrainPowerManagementCommand extends CommandBase {
    private final PowerManagementSubsystem powerManagementSubsystem = PowerManagementSubsystem.getInstance();

    public DrivetrainPowerManagementCommand() {
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.powerManagementSubsystem);
    }

    @Override
    public void initialize() {
        powerManagementSubsystem.setDrivetrainCurrentLimit(180);

    }

    @Override
    public void execute() {
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
