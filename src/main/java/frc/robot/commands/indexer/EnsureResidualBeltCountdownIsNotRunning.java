package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IndexerSubsystem;


final public class EnsureResidualBeltCountdownIsNotRunning extends InstantCommand {
    private final IndexerSubsystem indexerSubsystem;

    public EnsureResidualBeltCountdownIsNotRunning(final IndexerSubsystem indexer) {
        this.indexerSubsystem = indexer;
        addRequirements(this.indexerSubsystem);
    }

    @Override
    public void initialize() {
        indexerSubsystem.residualBeltFlag = true;
    }
}
