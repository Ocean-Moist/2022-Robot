package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.drivetrain;
import static frc.robot.RobotContainer.indexer;

public class PowerManagementSubsystem extends SubsystemBase {

    private final PowerDistribution pd = new PowerDistribution();

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this PowerManagementSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static PowerManagementSubsystem INSTANCE = new PowerManagementSubsystem();

    /**
     * Returns the Singleton instance of this PowerManagementSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code PowerManagementSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static PowerManagementSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this PowerManagementSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private PowerManagementSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

    }

    public double getDrivetrainCurrent() {
        // return current from all drivetrain motor
        return pd.getCurrent(Constants.PDPConstants.drivetrainMotor1) + pd.getCurrent(Constants.PDPConstants.drivetrainMotor2) + pd.getCurrent(Constants.PDPConstants.drivetrainMotor3) + pd.getCurrent(Constants.PDPConstants.drivetrainMotor4);
    }

    public double getShooterCurrent() {
        return pd.getCurrent(Constants.PDPConstants.shooterMotorA) + pd.getCurrent(Constants.PDPConstants.shooterMotorB);
    }

    public double getIntakeCurrent() {
        return pd.getCurrent(Constants.PDPConstants.intakeMotor);
    }

    public void setShooterPower(int i, int j) {
        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration(true, i, j, 1);
        WPI_TalonFX shooterMotorA = drivetrain.getShooterMotorA();
        WPI_TalonFX shooterMotorB = drivetrain.getShooterMotorB();
        shooterMotorA.configSupplyCurrentLimit(config, 0);
        shooterMotorB.configSupplyCurrentLimit(config, 0);
    }
    public void setIntakeCurrent(int i, int j) {
        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration(true, i, j, 1);
        WPI_TalonFX intakeMotor = drivetrain.getIntakeMotor();
        intakeMotor.configSupplyCurrentLimit(config, 0);
    }

    public void setIndexerCurrentLimit(int i, int j) {
        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration(true, i, j, 1);
        WPI_TalonFX indexerMotor = indexer.getIndexerMotor();
        indexerMotor.configSupplyCurrentLimit(config, 0);
    }
    public void setDrivetrainCurrentLimit(int i, int j) {
        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration(true, i, j, 1);
        WPI_TalonFX[] driveMotors = drivetrain.getDriveMotors();
        for (WPI_TalonFX motor : driveMotors) {
            motor.configSupplyCurrentLimit(config, 0);
        }
    }
}

