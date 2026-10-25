import frc.robot.util.MathUtilExt;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.function.Function;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class MathUtilTests {

    @Test
    void testPowerCurve() {
        // max tests
        assertEquals(0.0, MathUtilExt.axisPowerCurve(0.0, 1.0, 0.0, 1.0));
        assertEquals(1.0, MathUtilExt.axisPowerCurve(1.0, 1.0, 0.0, 1.0));
        assertEquals(0.5, MathUtilExt.axisPowerCurve(1.0, 1.0, 0.0, 0.5));
        assertEquals(-0.5, MathUtilExt.axisPowerCurve(-1.0, 1.0, 0.0, 0.5));
        assertEquals(0.5, MathUtilExt.axisPowerCurve(0.5, 1.0, 0.0, 1.0));
        assertEquals(-0.5, MathUtilExt.axisPowerCurve(-0.5, 1.0, 0.0, 1.0));

        // deadband
        assertEquals(0.0, MathUtilExt.axisPowerCurve(0.01, 1.0, 0.1, 1.0));
        assertEquals(0.0, MathUtilExt.axisPowerCurve(0.1, 1.0, 0.1, 1.0));
        assertEquals(0.4444444444444445, MathUtilExt.axisPowerCurve(0.5, 1.0, 0.1, 1.0));

        // exponent
        assertEquals(0.5, MathUtilExt.axisPowerCurve(0.5, 1.0, 0.0, 1.0));
        assertEquals(0.25, MathUtilExt.axisPowerCurve(0.5, 2.0, 0.0, 1.0));
        assertEquals(0.125, MathUtilExt.axisPowerCurve(0.5, 3.0, 0.0, 1.0));

        // exponent and max
        assertEquals(0.5, MathUtilExt.axisPowerCurve(1.0, 1.0, 0.0, 0.5));
        assertEquals(0.25, MathUtilExt.axisPowerCurve(0.5, 1.0, 0.0, 0.5));
        assertEquals(0.125, MathUtilExt.axisPowerCurve(0.5, 2.0, 0.0, 0.5));
        assertEquals(0.03125, MathUtilExt.axisPowerCurve(0.5, 4.0, 0.0, 0.5));

        // exponent and max and deadband
        assertEquals(0.5, MathUtilExt.axisPowerCurve(1.0, 1.0, 0.1, 0.5));
        assertEquals(0.22222222222222224, MathUtilExt.axisPowerCurve(0.5, 1.0, 0.1, 0.5));
        assertEquals(0.09876543209876544, MathUtilExt.axisPowerCurve(0.5, 2.0, 0.1, 0.5));
        assertEquals(0.019509221155311694, MathUtilExt.axisPowerCurve(0.5, 4.0, 0.1, 0.5));

        ChartUtils.generateChart("PowerCurveOutput01.png", -1.0, 1.0, 0.05, (input) -> {
            return MathUtilExt.axisPowerCurve(input, 2.0, 0.1, 0.75);
        });
    }



}
