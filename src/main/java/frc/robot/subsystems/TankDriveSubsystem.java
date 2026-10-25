package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.InvertType;
import frc.robot.RobotContainer;
import frc.robot.util.DebugUtil;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class TankDriveSubsystem extends SubsystemBase {
    // Left side controllers
    private final WPI_VictorSPX m_leftLeader;
    private final WPI_VictorSPX m_leftFollower;

    // Right side controllers
    private final WPI_VictorSPX m_rightLeader;
    private final WPI_VictorSPX m_rightFollower;

    // WPILib DifferentialDrive wrapper
    private final DifferentialDriveOdometry m_odometry;
    private final DifferentialDrive m_drive;

    // Telemetry Trackers
    private Pose2d m_robotPose = new Pose2d(); // Storage variable for our tracking state
    private final Field2d m_field = new Field2d();
    private final AnalogGyro m_gyro = new AnalogGyro(0);
    private final Encoder m_leftEncoder = new Encoder(6, 1);
    private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private final Encoder m_rightEncoder = new Encoder(7, 3);
    private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

    // Simulation Engines
    private DifferentialDrivetrainSim m_drivetrainSim;
    private AnalogGyroSim m_gyroSim;

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

        double distancePerPulse = (Math.PI * Meters.convertFrom(4.0, Inches)) / 4096.0;
        m_leftEncoder.setDistancePerPulse(distancePerPulse);
        m_rightEncoder.setDistancePerPulse(distancePerPulse);

        m_odometry = new DifferentialDriveOdometry(
                m_gyro.getRotation2d(),
                m_leftEncoder.getDistance(),
                m_rightEncoder.getDistance(),
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
     * @param leftSpeed Speed of the left side (-1.0 to 1.0)
     * @param rightSpeed Speed of the right side (-1.0 to 1.0)
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_drive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Drives the robot using arcade style (one joystick for throttle, one for rotation).
     * @param xSpeed Forward/backward speed (-1.0 to 1.0)
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
                m_leftEncoder.getDistance(),
                m_rightEncoder.getDistance()
        );

        // 2. Update field visualizer coordinate tracker
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    //
    // Simulation
    //

    private void ResetPose(Pose2d pose) {
        // A. Reset the physics engine state
        m_drivetrainSim.setPose(pose);

        // B. Clear the raw sensor history back to zero distance
        m_leftEncoder.reset();
        m_rightEncoder.reset();
        m_leftEncoderSim.setDistance(0.0);
        m_rightEncoderSim.setDistance(0.0);

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
        var drivetrainPlant = LinearSystemId.identifyDrivetrainSystem(1.5, 0.2, 1.5, 0.3);

        m_drivetrainSim = new DifferentialDrivetrainSim(
                drivetrainPlant,
                DCMotor.getCIM(2),
                1.0,
                0.75,
                Meters.convertFrom(4.0, Inches),
                VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        );

//        m_drivetrainSim = new DifferentialDrivetrainSim(
//                DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
//                1.0,                    // 7.29:1 gearing reduction.
//                7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
//                60.0,                    // The mass of the robot is 60 kg.
//                Units.inchesToMeters(4), // The robot uses 3" radius wheels.
//                0.7112,                  // The track width is 0.7112 meters.
//                // The standard deviations for measurement noise:
//                // x and y:          0.001 m
//                // heading:          0.001 rad
//                // l and r velocity: 0.1   m/s
//                // l and r position: 0.005 m
//                VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

        m_gyroSim = new AnalogGyroSim(m_gyro);
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
        m_gyroSim.setAngle(-m_drivetrainSim.getHeading().getDegrees());
        m_leftEncoderSim.setDistance(m_drivetrainSim.getLeftPositionMeters());
        m_rightEncoderSim.setDistance(m_drivetrainSim.getRightPositionMeters());

        // 3. Directly extract the true 2D simulation pose coordinate frame and overwrite our state variable
        m_field.setRobotPose(m_drivetrainSim.getPose());
    }

    public Pose2d getPose() {
        return m_field.getRobotPose();
    }

    //
    // Commands
    //

    public Command arcadeDriveCommand(DoubleSupplier speed, DoubleSupplier rotation)
    {
        return this.runEnd(
                () -> {
            arcadeDrive(
                    speed.getAsDouble(),
                    rotation.getAsDouble()
            );
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
