package frc.robot.commands.PowerManagement;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.RobotContainer.*;

public class DefaultPowerManagementCommandGroup extends SequentialCommandGroup {
    public DefaultPowerManagementCommandGroup() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new FooCommand(), new BarCommand());
        super();
        addCommands(
                new DrivetrainPowerManagementCommand(powerManagementSubsystem),
                new IntakePowerManagementCommand(intake),
                new ShooterPowerManagementCommand(outtake),
                new TurretPowerManagementCommand(outtake),
                new IndexerPowerManagementCommand(indexer)
        );
    }
}