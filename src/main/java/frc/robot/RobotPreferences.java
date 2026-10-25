package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Preferences;

import static edu.wpi.first.units.Units.Degrees;

public class RobotPreferences {

    public static final String forwardAxisPower = "GameController/Forward Power";
    public static final String turnAxisPower = "GameController/Turn Power";

    // Singleton
    private RobotPreferences() {

        registerAxisPowerPreference(forwardAxisPower, 0.1, 4.0, 0.5);
        registerAxisPowerPreference(turnAxisPower, 0.1, 4.0, 0.5);

    }

    private static class RobotContainerHolder {
        private static final RobotPreferences INSTANCE = new RobotPreferences();
    }

    public static RobotPreferences getInstance() {
        return RobotPreferences.RobotContainerHolder.INSTANCE;
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

}
