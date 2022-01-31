//package frc.robot.subsystems;
//
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
//import edu.wpi.first.math.Nat;
//import edu.wpi.first.math.VecBuilder;
//import edu.wpi.first.math.controller.LinearQuadraticRegulator;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.estimator.KalmanFilter;
//import edu.wpi.first.math.numbers.N1;
//import edu.wpi.first.math.system.LinearSystem;
//import edu.wpi.first.math.system.LinearSystemLoop;
//import edu.wpi.first.math.system.plant.LinearSystemId;
//import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.DutyCycleEncoder;
//import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
//
//import java.util.function.Supplier;
//
//import static frc.robot.Constants.MechanismConstants.*;
//
//
//public class OuttakeSubsystem extends SubsystemBase {
//    // Setup motors, pid controller, and booleans
//    Supplier<Double> encoderRate;
//    Supplier<Double> encoderPosition;
//    private final WPI_TalonFX shooterMotorFront = new WPI_TalonFX(shooterMotorPortA);
//    private final WPI_TalonFX shooterMotorBack = new WPI_TalonFX(shooterMotorPortB);
//    private final CANSparkMax hoodAngleMotor = new CANSparkMax(hoodAngleMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
//    private static final int ENCODER_EDGES_PER_REV = 1;
//    private static final int PIDIDX = 0;
//    private static double kSpinupRadPerSec;
//
//    private static final int kTimeoutMs = 30;
//    private static final int filterWindowSize = 1;
//    private final DutyCycleEncoder hoodAngleEncoder = new DutyCycleEncoder(hoodAngleEncoderPort);
//
//    PIDController hoodAnglePID;
//    // The plant holds a state-space model of our flywheel. This system has the following properties:
//    //
//    // States: [velocity], in radians per second.
//    // Inputs (what we can "put in"): [voltage], in volts.
//    // Outputs (what we can measure): [velocity], in radians per second.
//    //
//    // The Kv and Ka constants are found using the FRC Characterization toolsuite.
//    private final LinearSystem<N1, N1, N1> m_flywheelPlant = LinearSystemId.identifyVelocitySystem(
//            Constants.AutoConstants.kFlywheelKv, Constants.AutoConstants.kFlywheelKa);
//
//    // The observer fuses our encoder data and voltage inputs to reject noise.
//    private final KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
//            Nat.N1(), Nat.N1(),
//            m_flywheelPlant,
//            VecBuilder.fill(3.0), // How accurate we think our model is
//            VecBuilder.fill(0.01), // How accurate we think our encoder data is
//            0.020);
//
//    // A LQR uses feedback to create voltage commands.
//    private final LinearQuadraticRegulator<N1, N1, N1> m_controller
//            = new LinearQuadraticRegulator<>(m_flywheelPlant,
//            VecBuilder.fill(8.0),  // Velocity error tolerance
//            VecBuilder.fill(12.0), // Control effort (voltage) tolerance
//            0.020);
//
//    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
//    private final LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(
//            m_flywheelPlant,
//            m_controller,
//            m_observer,
//            12.0,
//            0.020);
//
//    public boolean shooterRunning = false;
//    private double currentFrontShooterPower = 0.0;
//    private double currentBackShooterPower = 0.0;
//    private double currentHoodAngle = defaultHoodAngle;
//    double encoderConstant = (1 / ENCODER_EDGES_PER_REV) * 1;
//
//
//
//    public OuttakeSubsystem() {
//        shooterMotorFront.configFactoryDefault();
//        shooterMotorBack.configFactoryDefault();
//        shooterMotorFront.setNeutralMode(NeutralMode.Coast);
//        shooterMotorBack.setNeutralMode(NeutralMode.Coast);
//        shooterMotorFront.setSensorPhase(false);
//
//        hoodAngleEncoder.reset();
//        hoodAngleEncoder.setDistancePerRotation(2*Math.PI);
//        shooterMotorFront.setInverted(true);
//        shooterMotorBack.setInverted(false);
//        hoodAnglePID = new PIDController(0, 0 ,0);
//        hoodAnglePID.setSetpoint(defaultHoodAngle);
//
//        shooterMotorBack.follow(shooterMotorFront);
//
//
//        encoderPosition = ()
//                -> shooterMotorFront.getSelectedSensorPosition(PIDIDX) * encoderConstant/1024;
//        encoderRate = ()
//                -> (shooterMotorFront.getSelectedSensorVelocity(PIDIDX) * encoderConstant * 10/1024);
//
//        m_loop.reset(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(60*encoderRate.get())));
//
//        // Default command to stop()
//        this.setDefaultCommand(new RunCommand(() -> stop(), this));
//    }
//
//    public void stop(){
//        m_loop.setNextR(VecBuilder.fill(0.0));
//        shooterMotorFront.stopMotor();
//    }
//
//    private void setShooterPower(double power) { // Enables both wheels
//        setShooterFront(power);
//        setShooterBack(power);
//        shooterRunning = true;
//    }
//
//    public void setShooterFront(double power) { // Enables front wheels
//        if (power>1.0 || power<0.0) return; currentFrontShooterPower = power;
//    }
//
//    public void setShooterBack(double power) { // Enables back wheels
//        shooterMotorBack.set(ControlMode.PercentOutput, power);
//        shooterRunning = true;
//    }
//
//
//
//    public void setHoodAngle(double angle) { if (angle>45.0 || angle<10.0) return; this.currentHoodAngle = angle; hoodAnglePID.setSetpoint(angle); }
//
//    public double getHoodAngle() { return hoodAngleEncoder.getDistance(); } //RADIANS
//
//    public void disable() {
//        stop();
//        //stopHood();
//        //stopTurret();
//    }
//
//    public void start() {
//        m_loop.setNextR(VecBuilder.fill(kSpinupRadPerSec));
//    }
//
//    @Override
//    public void periodic() {
//        // Correct our Kalman filter's state vector estimate with encoder data.
//        m_loop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(60*encoderRate.get())));
//
//        // Update our LQR to generate new voltage commands and use the voltages to predict the next
//        // state with out Kalman filter.
//        m_loop.predict(0.020);
//
//        // Send the new calculated voltage to the motors.
//        // voltage = duty cycle * battery voltage, so
//        // duty cycle = voltage / battery voltage
//        double nextVoltage = m_loop.getU(0);
//        shooterMotorFront.setVoltage(nextVoltage);
//    }
//}
//
