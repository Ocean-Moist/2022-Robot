package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.DetourableCommand;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.Constants.MechanismConstants.*;


public class TurretTurnToGoalWithLimelight extends CommandBase {
    private final LimelightSubsystem limelightSubsystem;
    private final OuttakeSubsystem outtakeSubsystem;
    private LimelightDataLatch offsetLatch;
    private double offset = Double.MAX_VALUE;

    public TurretTurnToGoalWithLimelight(LimelightSubsystem limelightSubsystem, OuttakeSubsystem outtakeSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        offsetLatch = new LimelightDataLatch(LimelightDataType.HORIZONTAL_OFFSET, 5);
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {
        outtakeSubsystem.limelightTurretAnglePID.reset();
        limelightSubsystem.addLatch(offsetLatch.reset());
        offset = Double.MAX_VALUE;
    }

    @Override
    public void execute() {
        try {
            System.out.println(offsetLatch.unlocked());
            if (offsetLatch.unlocked()) {
                offset = offsetLatch.open();
                outtakeSubsystem.turnTurret(-outtakeSubsystem.limelightTurretAnglePID.calculate(offset));
                throw new GoalNotFoundException(); //shortcut to latch reset  vvv  (since we've expended it)
            }
        } catch (GoalNotFoundException e) {
            limelightSubsystem.addLatch(offsetLatch.reset()); //assuming we want to look for the goal forever
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(offset) < 1 && Math.abs(outtakeSubsystem.limelightTurretAnglePID.getVelocityError()) < 0.08;
    }

    @Override
    public void end(boolean interrupted) {outtakeSubsystem.limelightTurretAnglePID.reset();}
}
