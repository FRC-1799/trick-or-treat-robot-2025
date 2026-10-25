package frc.robot.util;

public class MathUtilExt {

//    /**
//     * @param rawInput joystick axis input -1 to 1
//     * @param exponent power curve 1 = linear anything above that limits power
//     * @param deadband axis deadband
//     * @param max max output
//     * @return new axis value
//     */
//    public static double axisPowerCurve(double rawInput, double exponent, double deadband, double max) {
//        double deadBandInput = edu.wpi.first.math.MathUtil.applyDeadband(rawInput, deadband);
//        double sign = Math.signum(deadBandInput);
//        double absoluteValue = Math.abs(deadBandInput);
//        return sign * Math.min(Math.pow(absoluteValue, exponent), max);
//    }

    /**
     * @param rawInput joystick axis input -1 to 1
     * @param exponent power curve 1 = linear anything above that limits power
     * @param deadband axis deadband
     * @param max max output
     * @return new axis value
     */
    public static double axisPowerCurve(double rawInput, double exponent, double deadband, double max) {
        /// Clamp raw input to ensure it never exceeds the [-1.0, 1.0] domain
        double clampedInput = Math.max(-1, Math.min(1, rawInput));
        double absInput = Math.abs(clampedInput);

        // 1. Return exactly 0.0 if the input falls within the deadband threshold
        if (absInput <= deadband) {
            return 0.0;
        }

        // 2. Normalize across the remaining active travel range.
        // This scales the range [DEADBAND, 1.0] onto [0.0, 1.0], ensuring output starts at 0
        double activeRangeFactor = (absInput - deadband) / (1 - deadband);

        // 3. Apply exponential shaping and scale directly to your 0.5 maximum output
        double output = max * Math.pow(activeRangeFactor, exponent);

        // 4. Restore the original directional sign
        return Math.copySign(output, clampedInput);
    }

}
