package frc.robot.util;

import edu.wpi.first.networktables.*;

import java.util.HashMap;
import java.util.Map;

public class DebugUtil {

    private static final String DebugTopics = "Debug";

    private static Map<String, Publisher> publishers = new HashMap<>();

    private static <T> T getPublisher(Class<T> clazz, String name) {
        Object publisher = publishers.get(name);

        if(clazz.isInstance(publisher)) {
            return clazz.cast(publisher);
        }

        return null;
    }

    private static NetworkTable getDebugTable() {
        return NetworkTableInstance.getDefault().getTable(DebugTopics);
    }

    public static <T> void Publish(String name, int value) {
        var publisher = getPublisher(IntegerPublisher.class, name);
        if(publisher == null) {
            publisher = getDebugTable().getIntegerTopic(name).publish();
            publishers.put(name, publisher);
        }

        publisher.set(value);
    }

    public static <T> void Publish(String name, double value) {
        var publisher = getPublisher(DoublePublisher.class, name);
        if(publisher == null) {
            publisher = getDebugTable().getDoubleTopic(name).publish();
            publishers.put(name, publisher);
        }

        publisher.set(value);
    }

    public static void Publish(String name, String value) {
        var publisher = getPublisher(StringPublisher.class, name);
        if(publisher == null) {
            publisher = getDebugTable().getStringTopic(name).publish();
            publishers.put(name, publisher);
        }

        publisher.set(value);
    }

}
