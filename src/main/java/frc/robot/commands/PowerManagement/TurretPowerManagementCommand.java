package frc.robot.commands.PowerManagement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;


public class TurretPowerManagementCommand extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;

    public TurretPowerManagementCommand(OuttakeSubsystem outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg
        // of Subsystem)
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
