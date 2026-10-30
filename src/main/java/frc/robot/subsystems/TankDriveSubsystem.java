package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotPreferences;
import frc.robot.util.DebugUtil;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class TankDriveSubsystem extends SubsystemBase {
    private final double kTrackWidthMeters = Meters.convertFrom(24.0, Inches);
    // Left side controllers
    private final WPI_VictorSPX m_leftLeader;
    private final WPI_VictorSPX m_leftFollower;
    // Right side controllers
    private final WPI_VictorSPX m_rightLeader;
    private final WPI_VictorSPX m_rightFollower;
    // WPILib DifferentialDrive wrapper
    private final DifferentialDriveOdometry m_odometry;
    private final DifferentialDrive m_drive;
    private final Field2d m_field = new Field2d();
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kMXP);
    // Telemetry Trackers
    private final Pose2d m_robotPose = new Pose2d(); // Storage variable for our tracking state
    // Gyro Stablized Drive
    private final PIDController turnCorrectionForward = new PIDController(0.01, 0.01, 0.0);
    private final PIDController turnCorrectionReverse = new PIDController(0.01, 0.01, 0.0);
    private final double kMaxTurnRateDegPerSec = 180.0;
    private final RobotPreferences config = RobotPreferences.getInstance();
    // Simulation Engines
    private DifferentialDrivetrainSim m_drivetrainSim;
    private ADXRS450_GyroSim m_gyroSim;
    private double m_leftPositionMeters = 0.0;
    private double m_rightPositionMeters = 0.0;
    private double m_targetHeading;

    public TankDriveSubsystem() {
        // 1. Initialize the motor controllers with their respective CAN IDs
        m_leftLeader = new WPI_VictorSPX(1);
        m_leftFollower = new WPI_VictorSPX(3);
        m_rightLeader = new WPI_VictorSPX(2);
        m_rightFollower = new WPI_VictorSPX(4);

        // 2. Factory default all controllers to avoid leftover configuration bugs
        m_leftLeader.configFactoryDefault();
        m_leftFollower.configFactoryDefault();
        m_rightLeader.configFactoryDefault();
        m_rightFollower.configFactoryDefault();

        // 3. Set up hardware-level follower mode
        m_leftFollower.follow(m_leftLeader);
        m_rightFollower.follow(m_rightLeader);

        // 4. Configure motor inversion rules
        // Invert one complete side so that identical inputs drive the robot straight forward
        m_leftLeader.setInverted(false);
        m_rightLeader.setInverted(true);

        // Ensure followers perfectly mirror their designated leader's orientation
        m_leftFollower.setInverted(InvertType.FollowMaster);
        m_rightFollower.setInverted(InvertType.FollowMaster);

        // 5. Instantiate DifferentialDrive using functional lambdas pointing to the leaders
        m_drive = new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);

        var initialPose = new Pose2d(8.27, 4.11, new Rotation2d());

        m_odometry = new DifferentialDriveOdometry(
                m_gyro.getRotation2d(),
                0.0, 0.0,
                initialPose
        );

        SmartDashboard.putData("Field", m_field);

        // Setup physics rendering if running in a Simulation environment
        if (RobotBase.isSimulation()) {
            setupSimulation();

            ResetPose(initialPose);
        }
    }

    /**
     * Drives the robot using classic tank style (separate left and right stick control).
     *
     * @param leftSpeed  Speed of the left side (-1.0 to 1.0)
     * @param rightSpeed Speed of the right side (-1.0 to 1.0)
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_drive.tankDrive(leftSpeed, rightSpeed);
    }

    //
    // Simulation
    //

    /**
     * Drives the robot using arcade style (one joystick for throttle, one for rotation).
     *
     * @param xSpeed    Forward/backward speed (-1.0 to 1.0)
     * @param zRotation Rotation speed (-1.0 to 1.0)
     */
    public void arcadeDrive(double xSpeed, double zRotation) {
        m_drive.arcadeDrive(xSpeed, zRotation);
    }

    /**
     * Stops the drivetrain completely.
     */
    public void stop() {
        m_drive.stopMotor();
    }

    @Override
    public void periodic() {
        // 1. Always update odometry with current sensor reads (handles real robot and sim variables seamlessly)
        m_odometry.update(
                m_gyro.getRotation2d(),
                m_leftPositionMeters,
                m_rightPositionMeters
        );

        // 2. Update field visualizer coordinate tracker
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    private void ResetPose(Pose2d pose) {
        // A. Reset the physics engine state
        m_drivetrainSim.setPose(pose);

        m_leftPositionMeters = 0.0;
        m_rightPositionMeters = 0.0;

        // C. Force the simulated gyro register to mirror the new angle
        // (Negate the angle if your physical gyro configuration is CW-positive)
        m_gyroSim.setAngle(-pose.getRotation().getDegrees());

        // D. Re-sync standard odometry tracking framework
        m_odometry.resetPosition(
                pose.getRotation(),
                0.0, // Match the 0.0 encoder distance reset above
                0.0,
                pose
        );
    }

    private void setupSimulation() {
        var idealPlant = LinearSystemId.identifyDrivetrainSystem(
                1.5,
                0.2,
                1.5,
                0.3
        );
        var drivetrainPlant = WeakenMotors(
                idealPlant,
                config.getLeftWheelEfficiency(),
                config.getRightWheelEfficiency()
        );

        m_drivetrainSim = new DifferentialDrivetrainSim(
                drivetrainPlant,
                DCMotor.getCIM(2),
                1.0,
                kTrackWidthMeters,
                Meters.convertFrom(4.0, Inches),
                VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        );

        m_gyroSim = new ADXRS450_GyroSim(m_gyro);
    }

    //
    // Commands
    //

    private LinearSystem<N2, N2, N2> WeakenMotors(LinearSystem<N2, N2, N2> plant, double leftEfficiency, double rightEfficiency) {
        // New Matrix
        Matrix<N2, N2> idealA = plant.getA();
        Matrix<N2, N2> idealB = plant.getB();

        // Create new altered Left Matrix A
        Matrix<N2, N2> weakA = new Matrix<>(N2.instance, N2.instance);
        weakA.set(0, 0, idealA.get(0, 0));
        weakA.set(0, 1, idealA.get(0, 1));

        weakA.set(1, 0, idealA.get(1, 0));
        weakA.set(1, 1, idealA.get(1, 1) * leftEfficiency);

        // Create new altered Right Matrix B
        Matrix<N2, N2> weakB = new Matrix<>(N2.instance, N2.instance);
        weakB.set(0, 0, idealB.get(0, 0));
        weakB.set(0, 1, idealB.get(0, 1));

        weakB.set(1, 0, idealB.get(1, 0));
        weakB.set(1, 1, idealB.get(1, 1) * rightEfficiency);

        // Return new LinearSystem
        return new LinearSystem<>(weakA, weakB, plant.getC(), plant.getD());
    }

    @Override
    public void simulationPeriodic() {
        // 1. Calculate physics movement based on motor voltages
        var leftVoltage = m_leftLeader.get() * RobotController.getBatteryVoltage();
        var rightVoltage = m_rightLeader.get() * RobotController.getBatteryVoltage();

        m_drivetrainSim.setInputs(
                leftVoltage,
                rightVoltage
        );

        m_drivetrainSim.update(0.020);

        // 2. Sync the simulated gyro register with physics
        double leftVelocityMetersPerSec = m_drivetrainSim.getLeftVelocityMetersPerSecond();
        double rightVelocityMetersPerSec = m_drivetrainSim.getRightVelocityMetersPerSecond();
        double angularVelocityRadPerSec = (rightVelocityMetersPerSec - leftVelocityMetersPerSec) / kTrackWidthMeters;

        double currentRateDegPerSec = Math.toDegrees(angularVelocityRadPerSec);

        // 4. Manually push BOTH values into the simulation instance
        m_gyroSim.setRate(-currentRateDegPerSec);
        //m_gyroSim.setAngle(-m_drivetrainSim.getHeading().getDegrees());
        m_leftPositionMeters = m_drivetrainSim.getLeftPositionMeters();
        m_rightPositionMeters = m_drivetrainSim.getRightPositionMeters();

        // 3. Directly extract the true 2D simulation pose coordinate frame and overwrite our state variable
        m_field.setRobotPose(m_drivetrainSim.getPose());
    }

    public Pose2d getPose() {
        return m_field.getRobotPose();
    }

    public Command arcadeDriveCommand(DoubleSupplier speed, DoubleSupplier rotation) {
        return this.runEnd(
                () -> {
                    if (config.getDirectionCorrection()) {
                        double xSpeed = speed.getAsDouble();
                        double joystickTurnInput = rotation.getAsDouble();
                        double zRotationCorrection = joystickTurnInput;

                        if(Math.abs(xSpeed) > 0.01) {
                            if (Math.abs(joystickTurnInput) < 0.05) {
                                joystickTurnInput = 0.0;
                            }

                            // Map the joystick input (-1.0 to 1.0) to a target turn rate (e.g., -180 to 180 deg/sec)
                            double targetTurnRate = joystickTurnInput * kMaxTurnRateDegPerSec;

                            // Get the robot's actual turning rate from the gyro
                            double currentTurnRate = -m_gyro.getRate();

                            DebugUtil.Publish("GyroTurn", currentTurnRate);

                            // The PID calculates a motor correction value (-1.0 to 1.0) to achieve that rate
                            if(xSpeed > 0.0) {
                                zRotationCorrection = turnCorrectionForward.calculate(currentTurnRate, targetTurnRate);
                            } else {
                                zRotationCorrection = turnCorrectionReverse.calculate(currentTurnRate, targetTurnRate);
                            }
                        }
                        // Pass the combined forward speed and the PID's output directly to arcadeDrive
                        m_drive.arcadeDrive(xSpeed, zRotationCorrection);
                    } else {
                        m_drive.arcadeDrive(speed.getAsDouble(), rotation.getAsDouble());
                    }
                },
                () -> {
                    stop();
                }
        );
    }

    public Command stopCommand() {
        return this.runOnce(this::stop);
    }
}
