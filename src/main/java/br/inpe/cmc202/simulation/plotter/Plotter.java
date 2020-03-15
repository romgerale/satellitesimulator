package br.inpe.cmc202.simulation.plotter;

import java.awt.Color;
import java.awt.Font;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import javax.swing.JFrame;

import org.apache.commons.lang.ArrayUtils;
import org.math.plot.Plot2DPanel;
import org.math.plot.Plot3DPanel;
import org.math.plot.utils.FastMath;

/**
 * Plotter. It offers a lot of functions to plot graphs. It is based on
 * jmathplot.
 * 
 * @author alessandro.g.romero
 * 
 */
public class Plotter {

	/**
	 * General used of 2d scatter.
	 * 
	 * @param map
	 * @param name
	 * @param yLabel
	 */
	public static void plot2DScatter(Map<Double, Double> map, String name,
			String yLabel) {

		if (map.size() == 0) {
			return;
		}

		// formatting data
		double[] keys = ArrayUtils.toPrimitive(map.keySet().toArray(
				new Double[0]));
		double[][] values = new double[keys.length][2];
		for (int i = 0; i < keys.length; i++) {
			values[i][0] = keys[i];
			values[i][1] = map.get(keys[i]);
		}

		// ploting
		Plot2DPanel plot = new Plot2DPanel();
		plot.setAxisLabels("time (s)", yLabel);
		plot.addLegend("SOUTH");
		plot.addScatterPlot(name, values);
		JFrame frame = new JFrame(name);
		frame.setSize(600, 600);
		frame.setContentPane(plot);
		frame.setVisible(true);

		// changing the x axis - only works after the render
		plot.setFixedBounds(0, values[0][0], values[values.length - 1][0]);

		// listener to close
		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) {
				System.exit(0);
			}
		});

	}

	/**
	 * General used 2d line.
	 * 
	 * @param map
	 * @param name
	 * @param toDegrees
	 * @param label
	 */
	public static void plot2DLine(Map<Double, double[]> map, String name,
			boolean toDegrees, String label) {

		if (map.size() == 0) {
			return;
		}

		// formatting data
		double[] keys = ArrayUtils.toPrimitive(map.keySet().toArray(
				new Double[0]));
		double[][] values = new double[keys.length][2];
		for (int i = 0; i < keys.length; i++) {
			values[i][0] = keys[i] / 10;
			values[i][1] = (toDegrees) ? FastMath
					.toDegrees(map.get(keys[i])[0]) : map.get(keys[i])[0];
		}

		double[][] values2 = new double[keys.length][2];
		for (int i = 0; i < keys.length; i++) {
			values2[i][0] = keys[i] / 10;
			values2[i][1] = (toDegrees) ? FastMath
					.toDegrees(map.get(keys[i])[1]) : map.get(keys[i])[1];
		}

		double[][] values3 = new double[keys.length][2];
		for (int i = 0; i < keys.length; i++) {
			values3[i][0] = keys[i] / 10;
			values3[i][1] = (toDegrees) ? FastMath
					.toDegrees(map.get(keys[i])[2]) : map.get(keys[i])[2];
		}

		// ploting
		Plot2DPanel plot = new Plot2DPanel();
		plot.setAxisLabels("time (s) x 10", label);
		// (toDegrees) ? "degrees" : "radians");
		plot.addLegend("SOUTH");
		plot.addLinePlot("x", values);
		plot.addLinePlot("y", values2);
		plot.addLinePlot("z", values3);

		// IFTOOM
		// customize X axe
		// rotate light labels
		// plot.getAxis(0).setLightLabelAngle(-FastMath.PI / 4);
		// change axe title position relatively to the base of the plot
		plot.getAxis(0).setLabelPosition(0.5, -0.1);
		plot.getAxis(0).setLabelFont(new Font("Arial", Font.PLAIN, 12));
		// customize Y axe
		// rotate light labels
		// plot.getAxis(1).setLightLabelAngle(-FastMath.PI / 4);
		// change axe title position relatively to the base of the plot
		// plot.getAxis(1).setLabelPosition(-0.15, 0.5);
		// change axe title angle
		// plot.getAxis(1).setLabelAngle(-FastMath.PI / 2);
		plot.getAxis(1).setLabelFont(new Font("Arial", Font.PLAIN, 12));

		JFrame frame = new JFrame(name);
		frame.setSize(600, 600);
		// SPACEOPS
		// IFTOM
		// frame.setSize(400, 400);
		frame.setContentPane(plot);
		frame.setVisible(true);

		// changing the x axis - only works after the render
		plot.setFixedBounds(0, values[0][0], values[values.length - 1][0]);

		// listener to close
		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) {
				System.exit(0);
			}
		});

	}

	/**
	 * General use 3d scatter.
	 * 
	 * @param toPlot
	 * @param name
	 */
	public static void plot3DScatter(Map<Double, double[]> toPlot, String name) {

		// formatting data
		Collection<double[]> values = toPlot.values();
		double[] valuesX = new double[values.size()];
		double[] valuesY = new double[values.size()];
		double[] valuesZ = new double[values.size()];
		int i = 0;
		for (double[] value : values) {
			valuesX[i] = value[0];
			valuesY[i] = value[1];
			valuesZ[i] = value[2];
			i++;
		}

		// ploting
		Plot3DPanel plot = new Plot3DPanel();
		plot.addLegend("SOUTH");
		plot.addScatterPlot(name, valuesX, valuesY, valuesZ);
		plot.addScatterPlot("Origin", new double[] { 0 }, new double[] { 0 },
				new double[] { 0 });
		plot.addScatterPlot("Initial State", new double[] { valuesX[0] },
				new double[] { valuesY[0] }, new double[] { valuesZ[0] });
		plot.addScatterPlot("Final State", new double[] { valuesX[i - 1] },
				new double[] { valuesY[i - 1] },
				new double[] { valuesZ[i - 1] });
		JFrame frame = new JFrame(name);
		frame.setSize(600, 600);
		frame.setContentPane(plot);
		frame.setVisible(true);

		// listener to close
		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) {
				System.exit(0);
			}
		});

	}

	/**
	 * Specific version of plotting in order to support LLA.
	 * 
	 * @param toPlot
	 * @param name
	 */
	public static void plot2DScatterLLA(Map<Double, double[]> toPlot,
			String name) {

		if (toPlot.size() == 0) {
			return;
		}

		{
			// formatting data
			Collection<double[]> values = toPlot.values();
			double[] valuesX = new double[toPlot.size()];
			double[] valuesY = new double[toPlot.size()];
			int i = 0;
			for (double[] value : values) {
				// Longitude
				valuesX[i] = FastMath.toDegrees(value[1]);
				// Latitude
				valuesY[i] = FastMath.toDegrees(value[0]);
				i++;
			}

			// ploting
			Plot2DPanel plot = new Plot2DPanel();
			plot.setAxisLabels("longitude", "latitude");
			plot.addLegend("SOUTH");
			plot.addScatterPlot(name + " Longitute/Latitude", valuesX, valuesY);
			plot.addScatterPlot("Brasilia", new double[] { -47 },
					new double[] { -15 });
			plot.addScatterPlot("Initial Position",
					new double[] { valuesX[0] }, new double[] { valuesY[0] });
			JFrame frame = new JFrame(name + " Longitute/Latitude");
			frame.setSize(600, 600);
			frame.setContentPane(plot);
			frame.setVisible(true);

			// changing the x axis - only works after the render
			plot.setFixedBounds(0, -180, 180);
			// changing the y axis - only works after the render
			plot.setFixedBounds(1, -90, 90);

			// listener to close
			frame.addWindowListener(new WindowAdapter() {
				public void windowClosing(WindowEvent e) {
					System.exit(0);
				}
			});
		}
		{
			// formatting data - altitude
			double[] keys = ArrayUtils.toPrimitive(toPlot.keySet().toArray(
					new Double[0]));
			double[][] values = new double[keys.length][2];
			for (int i = 0; i < keys.length; i++) {
				values[i][0] = keys[i];
				values[i][1] = toPlot.get(keys[i])[2];
			}

			// ploting
			Plot2DPanel plot = new Plot2DPanel();
			plot.setAxisLabels("time", "altitude (meter)");
			plot.addLegend("SOUTH");
			plot.addScatterPlot(name + " Altitude", values);
			JFrame frame = new JFrame(name + " Altitude");
			frame.setSize(600, 600);
			frame.setContentPane(plot);
			frame.setVisible(true);

			// changing the x axis - only works after the render
			plot.setFixedBounds(0, values[0][0], values[values.length - 1][0]);

			// listener to close
			frame.addWindowListener(new WindowAdapter() {
				public void windowClosing(WindowEvent e) {
					System.exit(0);
				}
			});
		}
	}

	/**
	 * General used 2d line.
	 * 
	 * @param map
	 * @param name
	 * @param toDegrees
	 * @param label
	 */
	public static void plot2DLine(Map<Double, double[]> map, String name,
			String label) {

		if (map.size() == 0) {
			return;
		}

		// formatting data
		Map<String, double[][]> allValues = new TreeMap<String, double[][]>();
		double[] keys = ArrayUtils.toPrimitive(map.keySet().toArray(
				new Double[0]));
		for (int kk = 0; kk < map.get(keys[0]).length; kk++) {
			double[][] values = new double[keys.length][2];
			for (int i = 0; i < keys.length; i++) {
				values[i][0] = keys[i];
				values[i][1] = map.get(keys[i])[kk];
			}
			allValues.put("q" + (kk + 1), values);
		}

		// ploting
		Plot2DPanel plot = new Plot2DPanel();
		plot.setAxisLabels("time (s)", label);
		plot.addLegend("SOUTH");
		for (String valueLabel : allValues.keySet()) {
			plot.addLinePlot(valueLabel, allValues.get(valueLabel));
		}

		// SPACEOPS
		plot.getAxis(0).setLabelPosition(0.5, -0.1);
		plot.getAxis(0).setLabelFont(new Font("Arial", Font.PLAIN, 12));
		plot.getAxis(1).setLabelFont(new Font("Arial", Font.PLAIN, 12));

		JFrame frame = new JFrame(name);

		frame.setSize(600, 600);
		// SPACEOPS
		// frame.setSize(400, 400);
		frame.setContentPane(plot);
		frame.setVisible(true);

		// listener to close
		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) {
				System.exit(0);
			}
		});

	}

	/**
	 * General used 2d line.
	 * 
	 * @param map
	 * @param name
	 * @param toDegrees
	 * @param label
	 */
	public static void plot2DLine(List<Map<Double, double[]>> mmap,
			String name, String label, String details) {

		if (mmap.size() == 0) {
			return;
		}

		// formatting data
		int j = 0;
		Map<String, double[][]> allValues = new TreeMap<String, double[][]>();
		for (Map<Double, double[]> map : mmap) {
			double[] keys = ArrayUtils.toPrimitive(map.keySet().toArray(
					new Double[0]));
			for (int kk = 0; kk < map.get(keys[0]).length; kk++) {
				double[][] values = new double[keys.length][2];
				for (int i = 0; i < keys.length; i++) {
					values[i][0] = keys[i];
					values[i][1] = map.get(keys[i])[kk];
				}
				allValues.put("q" + (kk + 1) + "_" + ++j, values);
			}
		}

		// ploting
		Plot2DPanel plot = new Plot2DPanel();
		plot.setAxisLabels("time (s)", label);
		plot.addLegend("INVISIBLE");
		for (String valueLabel : allValues.keySet()) {
			Color color;
			String slabel;
			if (valueLabel.startsWith("q1")) {
				slabel = "q1";
				color = Color.BLUE;
			} else if (valueLabel.startsWith("q2")) {
				slabel = "q2";
				color = Color.GREEN;
			} else if (valueLabel.startsWith("q3")) {
				slabel = "q3";
				color = Color.RED;
			} else {
				slabel = "q4";
				color = Color.YELLOW;
			}
			plot.addLinePlot(slabel, color, allValues.get(valueLabel));
		}

		// Panel a = new Panel();
		// a.setLayout(new BorderLayout());
		// Panel b = new Panel();
		// b.setLayout(new BorderLayout());
		JFrame frame = new JFrame(name);
		// frame.setLayout(new GridLayout(2, 1));
		frame.setSize(600, 600);
		// frame.add(a);
		// a.add(plot);
		// frame.add(b);
		// b.add(new JLabel(details));
		frame.setContentPane(plot);
		frame.setVisible(true);

		// listener to close
		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) {
				System.exit(0);
			}
		});

	}

	/**
	 * General used 2d line.
	 * 
	 * @param map
	 * @param name
	 * @param toDegrees
	 * @param label
	 */
	public static void plot2DLine(Map<String, Map<Double, Double>> mmap,
			String name) {

		if (mmap.size() == 0) {
			return;
		}

		// formatting data
		Map<String, double[][]> allValues = new TreeMap<String, double[][]>();
		for (String alphaIden : mmap.keySet()) {
			Map<Double, Double> one = mmap.get(alphaIden);
			double[][] values = new double[one.size()][2];
			int i = 0;
			for (Double t : one.keySet()) {
				values[i][0] = t;
				values[i][1] = one.get(t);
				i++;
			}
			allValues.put(alphaIden, values);
		}

		// ploting
		Plot2DPanel plot = new Plot2DPanel();
		plot.setAxisLabels("time (s)", "det. controllability");
		plot.addLegend("SOUTH");
		for (String valueLabel : allValues.keySet()) {
			plot.addLinePlot(valueLabel, allValues.get(valueLabel));
		}

		// Panel a = new Panel();
		// a.setLayout(new BorderLayout());
		// Panel b = new Panel();
		// b.setLayout(new BorderLayout());
		JFrame frame = new JFrame(name);
		// frame.setLayout(new GridLayout(2, 1));
		frame.setSize(600, 600);
		// frame.add(a);
		// a.add(plot);
		// frame.add(b);
		// b.add(new JLabel(details));
		frame.setContentPane(plot);
		frame.setVisible(true);

		// listener to close
		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) {
				System.exit(0);
			}
		});

	}

	/**
	 * General use 3d scatter.
	 * 
	 * @param toPlot
	 * @param name
	 */
	public static void plot3DScatterStateSpace(
			Map<String, Map<Double, double[]>> mmap, String name) {
		if (mmap == null || mmap.isEmpty()) {
			return;
		}

		// iterating over initial states
		Plot3DPanel plot = new Plot3DPanel();
		plot.addLegend("SOUTH");
		plot.addScatterPlot("Origin", new double[] { 0 }, new double[] { 0 },
				new double[] { 0 });
		for (String initialState : mmap.keySet()) {
			// formatting data
			Collection<double[]> values = mmap.get(initialState).values();
			double[] valuesX = new double[values.size()];
			double[] valuesY = new double[values.size()];
			double[] valuesZ = new double[values.size()];
			int i = 0;
			for (double[] value : values) {
				valuesX[i] = value[0];
				valuesY[i] = value[1];
				valuesZ[i] = value[2];
				i++;
			}

			// ploting
			plot.addScatterPlot(initialState, valuesX, valuesY, valuesZ);
			if(!initialState.contains("Poincare")) {
				plot.addScatterPlot(initialState + " Initial State",
						new double[] { valuesX[0] }, new double[] { valuesY[0] },
						new double[] { valuesZ[0] });
				plot.addScatterPlot(initialState + " Final State",
						new double[] { valuesX[i - 1] },
						new double[] { valuesY[i - 1] },
						new double[] { valuesZ[i - 1] });
			}
		}

		JFrame frame = new JFrame(name);
		frame.setSize(600, 600);
		frame.setContentPane(plot);
		frame.setVisible(true);

		// listener to close
		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) {
				System.exit(0);
			}
		});

	}

	/**
	 * General use 2d scatter.
	 * 
	 * @param toPlot
	 * @param name
	 */
	public static void plot2DScatterStateSpace(
			Map<String, Map<Double, double[]>> mmap, String name) {
		if (mmap == null || mmap.isEmpty()) {
			return;
		}

		// iterating over initial states
		Plot2DPanel plot = new Plot2DPanel();
		plot.addLegend("SOUTH");
		plot.addScatterPlot("Origin", new double[] { 0 }, new double[] { 0 });
		for (String initialState : mmap.keySet()) {
			// formatting data
			Collection<double[]> values = mmap.get(initialState).values();
			double[] valuesX = new double[values.size()];
			double[] valuesY = new double[values.size()];
			int i = 0;
			for (double[] value : values) {
				valuesX[i] = value[0];
				valuesY[i] = value[1];
				i++;
			}

			// ploting
			plot.addScatterPlot(initialState, valuesX, valuesY);
			plot.addScatterPlot(initialState + " Initial State",
					new double[] { valuesX[0] }, new double[] { valuesY[0] });
			plot.addScatterPlot(initialState + " Final State",
					new double[] { valuesX[i - 1] },
					new double[] { valuesY[i - 1] });
		}

		JFrame frame = new JFrame(name);
		frame.setSize(600, 600);
		frame.setContentPane(plot);
		frame.setVisible(true);

		// listener to close
		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) {
				System.exit(0);
			}
		});

	}
}
