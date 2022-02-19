package frc.robot.commands.PowerManagement;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DefaultPowerManagementCommandGroup extends SequentialCommandGroup {
    public DefaultPowerManagementCommandGroup() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new FooCommand(), new BarCommand());
        super();
        addCommands(
          new DrivetrainPowerManagementCommand(),
          new IntakePowerManagementCommand(),
          new ShooterPowerManagementCommand(),
          new TurretPowerManagementCommand(),
          new IndexerPowerManagementCommand()
        );
    }
}