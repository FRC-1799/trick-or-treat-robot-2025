package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.units.Units.Degrees;

public class RobotPreferences {

    public static final String forwardAxisPower = "GameController/Forward Power";
    public static final String turnAxisPower = "GameController/Turn Power";
    public static final String directionCorrection = "Drive/Direction Correction";
    public static final String leftWheelEfficiency = "Simulation/Left Wheel Efficiency";
    public static final String rightWheelEfficiency = "Simulation/Right Wheel Efficiency";


    // Singleton
    private RobotPreferences() {

        registerAxisPowerPreference(forwardAxisPower, 0.1, 4.0, 0.5);
        registerAxisPowerPreference(turnAxisPower, 0.1, 4.0, 0.5);
        registerDoublePreference(directionCorrection, 0.0);

        registerDoublePreference(leftWheelEfficiency, 1.0);
        registerDoublePreference(rightWheelEfficiency, 1.0);
    }

    private static class RobotContainerHolder {
        private static final RobotPreferences INSTANCE = new RobotPreferences();
    }

    public static RobotPreferences getInstance() {
        return RobotPreferences.RobotContainerHolder.INSTANCE;
    }

    public static void registerAxisPowerPreference(String key, double deadzone, double exponent, double maxPower) {
        registerDoublePreference(key + "/Deadzone", deadzone);
        registerDoublePreference(key + "/Exponent", exponent);
        registerDoublePreference(key + "/Max Power", maxPower);
    }

    public record AxisPowerPreferences(double deadzone, double exponent, double maxPower) {}
    public static AxisPowerPreferences getAxisPowerPreference(String key) {
        return new AxisPowerPreferences(
                Preferences.getDouble(key + "/Deadzone", 0.0),
                Preferences.getDouble(key + "/Exponent", 0.0),
                Preferences.getDouble(key + "/Max Power", 0.0)
        );
    }

    public static void registerPose3dPreference(String key, Pose3d val) {
        registerDoublePreference(key + "/Roll", val.getRotation().getMeasureX().in(Degrees));
        registerDoublePreference(key + "/Pitch", val.getRotation().getMeasureY().in(Degrees));
        registerDoublePreference(key + "/Yaw", val.getRotation().getMeasureZ().in(Degrees));
        registerDoublePreference(key + "/X", val.getX());
        registerDoublePreference(key + "/Y", val.getY());
        registerDoublePreference(key + "/Z", val.getZ());
    }

    public static void registerPose3dPreference(String key) {
        registerPose3dPreference(key, Pose3d.kZero);
    }

    private static void registerDoublePreference(String key, double val) {
        if (!Preferences.containsKey(key)) Preferences.initDouble(key, val);
    }

    public Pose3d getPose3dPreference(String key) {
        return new Pose3d(
                new Translation3d(
                        Preferences.getDouble(key + "/X", 0),
                        Preferences.getDouble(key + "/Y", 0),
                        Preferences.getDouble(key + "/Z", 0)
                ),
                new Rotation3d(
                        Degrees.of(Preferences.getDouble(key + "/Roll", 0)),
                        Degrees.of(Preferences.getDouble(key + "/Pitch", 0)),
                        Degrees.of(Preferences.getDouble(key + "/Yaw", 0))
                )
        );
    }

    //
    // Properties
    //

    public AxisPowerPreferences getForwardAxisPower() {
        String key = forwardAxisPower;
        return new AxisPowerPreferences(
                Preferences.getDouble(key + "/Deadzone", 0.1),
                Preferences.getDouble(key + "/Exponent", 4.0),
                Preferences.getDouble(key + "/Max Power", 0.5)
        );
    }

    public AxisPowerPreferences getTurnAxisPower() {
        String key = turnAxisPower;
        return new AxisPowerPreferences(
                Preferences.getDouble(key + "/Deadzone", 0.1),
                Preferences.getDouble(key + "/Exponent", 4.0),
                Preferences.getDouble(key + "/Max Power", 0.5)
        );
    }

    public boolean getDirectionCorrection() {
        return (Preferences.getDouble(directionCorrection, 0.0) > 0.0);
    }

    public double getLeftWheelEfficiency() {
        return Preferences.getDouble(leftWheelEfficiency, 1.0);
    }

    public double getRightWheelEfficiency() {
        return Preferences.getDouble(rightWheelEfficiency, 1.0);
    }


}
