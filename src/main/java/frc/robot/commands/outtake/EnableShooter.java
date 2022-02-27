package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.OuttakeSubsystem;


public class EnableShooter extends InstantCommand {
    private final OuttakeSubsystem outtakeSubsystem;

    public EnableShooter(OuttakeSubsystem outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() { outtakeSubsystem.setRPS(outtakeSubsystem.shuffleboardShooterPower);}

}
