package frc.robot.commands.PowerManagement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PowerManagementSubsystem;


public class IndexerPowerManagementCommand extends CommandBase {
    private PowerManagementSubsystem powerManagementSubsystem = PowerManagementSubsystem.getInstance();

    public IndexerPowerManagementCommand(PowerManagementSubsystem powerManagementSubsystem) {
        this.powerManagementSubsystem = powerManagementSubsystem;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg
        // of Subsystem)
        addRequirements(this.powerManagementSubsystem);
    }

    @Override
    public void initialize() {
        powerManagementSubsystem.setIndexerCurrentLimit(20,30);
    }

    @Override
    public void execute() {
        if (powerManagementSubsystem.getVoltage() < 7.7) {
            // indexer is off
            powerManagementSubsystem.setIndexerCurrentLimit(0,0);
        }
        if (powerManagementSubsystem.getVoltage() > 7.7) {
            powerManagementSubsystem.setIndexerCurrentLimit(20,30);
        }
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
