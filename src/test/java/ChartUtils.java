import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.function.Function;

public class ChartUtils {

    public static void generateChart(String fileName, double from, double to, double by, Function<Double, Double> function) {
        XYSeries series = new XYSeries("Power Curve Output");

        // Sampling loop matching your logic
        for (double input = from; input <= to + by; input += by) {
            double output = function.apply(input);
            series.add(input, output);
        }

        XYSeriesCollection dataset = new XYSeriesCollection(series);
        JFreeChart chart = ChartFactory.createXYLineChart(
                "Power Curve", "Input Axis", "Output Axis", dataset,
                PlotOrientation.VERTICAL, true, true, false
        );

        String resultsDir = System.getProperty("junit.output.dir");
        if (resultsDir != null) {
            try {
                Path path = Path.of(resultsDir, fileName);
                if (path.getParent() != null) {
                    Files.createDirectories(path.getParent());
                }

                Files.deleteIfExists(path);
                Path filePath = Files.createFile(path);
                File file = filePath.toFile();

                ChartUtilities.saveChartAsJPEG(file, chart, 800, 600);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

}
