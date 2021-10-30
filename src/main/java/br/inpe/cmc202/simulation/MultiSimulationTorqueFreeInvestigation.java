package br.inpe.cmc202.simulation;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.errors.OrekitException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.simulation.plotter.Plotter;

/**
 * 
 * Main class for the comparison od the sensitivity of the general conservative
 * 3d attitude.
 * 
 * @author alessandro.g.romero
 * 
 */
public class MultiSimulationTorqueFreeInvestigation extends MultiSimulationController {

	// STATESPACE
	private static final String CONTROLLER = "NopeController";

	static final private Logger logger = LoggerFactory
			.getLogger(MultiSimulationTorqueFreeInvestigation.class);

	/**
	 * @param numberOfSimulations
	 * @param stable
	 * @throws OrekitException
	 */
	public MultiSimulationTorqueFreeInvestigation(int numberOfSimulations, int stable)
			throws OrekitException {
		logger.info("Configuring multi simulation... NumberOfTrials: {}",
				numberOfSimulations);

		double[] initialAttitudeEulerAngles = new double[] { 0d, 0d, 0d };

		double[][] initialAngularVelocity = null;
		if (stable == 0) {
//			initialAngularVelocity = new double[][] { { 0d, 0d, 0d },
//					{ 0.001d, 0d, 0d }, { 0d, 0d, 0.001d }, { 1d, 0d, 0d },
//					{ 0d, 0d, 1d } };
			initialAngularVelocity = new double[][] { { 1.0d, 0.0d, .0d } };
		} else {
			initialAngularVelocity = new double[][] { { 0d, 0.100001d, 0d },
					{ 0d, 0.100000d, 0d } };
			/*
			RandomDataGenerator gaussianVelocity = new RandomDataGenerator();

			for (int i = 0; i < numberOfSimulations; i++) {
				int currentLength = initialAngularVelocity.length;
				double gaussianVelocityNumber = gaussianVelocity
						.nextNormal(.1d, 1d);
				double[][] initialAngularVelocityTemp = new double[currentLength + 4][3];
				System.arraycopy(initialAngularVelocity, 0,
						initialAngularVelocityTemp, 0, currentLength);
				initialAngularVelocity = initialAngularVelocityTemp;
				initialAngularVelocity[currentLength] = new double[] {
						1d + gaussianVelocityNumber, 0d, 0d };
				initialAngularVelocity[currentLength + 1] = new double[] { 0d,
						1d + gaussianVelocityNumber, 0d };
				initialAngularVelocity[currentLength + 2] = new double[] { 0d,
						0d, 1d + gaussianVelocityNumber };
				initialAngularVelocity[currentLength + 3] = new double[] {
						1d + gaussianVelocityNumber,
						1d + gaussianVelocityNumber,
						1d + gaussianVelocityNumber };
			}*/
		}

		for (int i = 0; i < initialAngularVelocity.length; i++) {

			// NOPECONTROLER
			SimulationController s = new SimulationController(CONTROLLER,
					initialAttitudeEulerAngles, initialAngularVelocity[i]);
			listSimulations.add(s);
			List<SimulationController> l = mapSimulations.get(CONTROLLER);
			if (l == null) {
				l = new ArrayList<SimulationController>();
				mapSimulations.put(CONTROLLER, l);
			}
			l.add(s);
		}
	}

	protected void plotSimulations() {
		// plotting
		NumberFormat numberFormatter = new DecimalFormat("##.000000");
		for (String key : mapSimulations.keySet()) {
			String details = "";
			List<Map<Double, double[]>> quaternionError = new ArrayList<Map<Double, double[]>>();
			List<Map<Double, double[]>> angularVelocity = new ArrayList<Map<Double, double[]>>();
			Map<String, Map<Double, double[]>> vetQuaternionError = new LinkedHashMap<String, Map<Double, double[]>>();
			Map<String, Map<Double, double[]>> vetAngularVelocity = new TreeMap<String, Map<Double, double[]>>();
			Map<String, Map<Double, Double>> poincareSection = new TreeMap<String, Map<Double, Double>>();

			// preparing poincare section q3 = 0
			Map<Double, double[]> poincareSectionStateSpace = new TreeMap<Double, double[]>();
			vetQuaternionError.put("Two-sided Poincare Section - q3 = 0", poincareSectionStateSpace);
			double count = 0;
			for(double x =-1; x <= 1; x+=0.1d) {
				for(double y =-1; y <= 1; y+=0.05d) {
					count++;
					poincareSectionStateSpace.put(count, new double[]{x,y,0d});
				}		
			}
			
			// collecting the simulations
			for (SimulationController s : mapSimulations.get(key)) {
				String detail = "";
				if (s.initialAngularVelocity != null
						&& s.initialAttitude != null) {
					detail = " initial attitude rad ("
							+ numberFormatter.format(s.initialAttitude[0])
							+ ";"
							+ numberFormatter.format(s.initialAttitude[1])
							+ ";"
							+ numberFormatter.format(s.initialAttitude[2])
							+ ") initial angular velocity rad/s ("
							+ numberFormatter
									.format(s.initialAngularVelocity[0])
							+ ";"
							+ numberFormatter
									.format(s.initialAngularVelocity[1])
							+ ";"
							+ numberFormatter
									.format(s.initialAngularVelocity[2]) + ")";
				}
				quaternionError.add(s.stepHandler.quaternionError);
				angularVelocity.add(s.stepHandler.angularVelocityBody);
				vetQuaternionError.put(detail,
						s.stepHandler.stateSpaceQuaternions);
				vetAngularVelocity.put(detail,
						s.stepHandler.angularVelocityBody);
				poincareSection.put(detail,s.stepHandler.poincareSection);
				details += "\n" + detail;
			}
			details += "";
			logger.info("**** ");
			logger.info(details);
			Plotter.plot2DLine(quaternionError, key, "quaternion error",
					details);
			Plotter.plot2DLine(angularVelocity, key, "angular velocity",
					details);

			// STATE SPACE
			Plotter.plot3DScatterStateSpace(vetQuaternionError,
					"state space quaternion - BODY");
			Plotter.plot3DScatterStateSpace(vetAngularVelocity,
					"state space velocity - BODY");
			
			// POINCARE SECTION - QUATERNION
			Map<Double,Double> poincareSection1 = poincareSection.get(poincareSection.keySet().toArray()[0]);
			Plotter.plot2DScatter(poincareSection1,
					"poincare section - q3 = 0", "q1");
			if (poincareSection.keySet().size() == 2){
				poincareSection1 = poincareSection.get(poincareSection.keySet().toArray()[1]);
				Plotter.plot2DScatter(poincareSection1,
						"poincare section - q3 = 0", "q1");
				
			}

			// evaluation of sensitivity to initial conditions - QUATERNION
			if (vetQuaternionError.keySet().size() == 3){
				Map<Double,Double> sensitivityToInitialConditions = new TreeMap<Double,Double>();
				logger.info("Using the {} dataset to check sensitivity to initial conditions ", vetQuaternionError.keySet().toArray()[1]);
				Map<Double, double[]> quaternionError1 = vetQuaternionError.get(vetQuaternionError.keySet().toArray()[1]);
				logger.info("Using the {} dataset to check sensitivity to initial conditions ", vetQuaternionError.keySet().toArray()[2]);
				Map<Double, double[]> quaternionError2 = vetQuaternionError.get(vetQuaternionError.keySet().toArray()[2]);
				Set<Double> time = quaternionError1.keySet();
				double largestLyapunovExponentSum = 0;
				double lastT =0;
				for (double t : time) {
					double[] qe1 = quaternionError1.get(t);
					double[] qe2 = quaternionError2.get(t);
					Vector3D v1 = new Vector3D(qe1);
					Vector3D v2 = new Vector3D(qe2);
					double dt = v1.distance(v2);
					sensitivityToInitialConditions.put(t, dt);
					largestLyapunovExponentSum+= FastMath.log(dt);
					lastT = t;
				}
				double largestLyapunovExponent = (largestLyapunovExponentSum / time.size()) * 1 / lastT;
				Plotter.plot2DScatter(sensitivityToInitialConditions,
						"sensitivity to initial conditions - Largest Lyapunov Exponent Computed = " + largestLyapunovExponent, "d");
			}
		}
	}

	/**
	 * Entry point for the simulation.
	 * 
	 * @param args
	 * @throws OrekitException
	 */
	public static void main(String[] args) throws OrekitException {
		logger.info("**********************************");
		logger.info("Satellite Multi Simulation Torque Free Investigation"
				+ (MultiSimulationTorqueFreeInvestigation.class.getPackage()
						.getImplementationVersion() == null ? ""
						: MultiSimulationTorqueFreeInvestigation.class.getPackage()
								.getImplementationVersion()));
		logger.info("**********************************");

		if (args.length > 0) {
			Integer numberOfSimulations = Integer.parseInt(args[0]);
			Integer stable = Integer.parseInt(args[1]);
			new MultiSimulationTorqueFreeInvestigation(numberOfSimulations, stable)
					.run();
		} else {
			throw new RuntimeException(
					"It should be informed the number of trials!");
		}

	}
}