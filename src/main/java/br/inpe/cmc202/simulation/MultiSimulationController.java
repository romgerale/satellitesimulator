package br.inpe.cmc202.simulation;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import org.hipparchus.random.RandomDataGenerator;
import org.orekit.errors.OrekitException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.simulation.plotter.Plotter;

/**
 * 
 * Main class for the comparison of the controller for a given Monte Carlo
 * perturbation model.
 * 
 * @author alessandro.g.romero
 * 
 */
public class MultiSimulationController implements Runnable {

	// SPACEOPS
	// private static final String CONTROLLER1 =
	// "ProportionalDerivativeLinearSunVectorController";
	// private static final String CONTROLLER2 =
	// "ProportionalLinearQuaternionPartialLQRController";
	// private static final String CONTROLLER3 =
	// "ProportionalNonLinearQuaternionSDREController_GIBBS";

	// ICEDYN
	// private static final String CONTROLLER1 =
	// "ProportionalNonLinearQuaternionSDREController_GIBBS";
	// private static final String CONTROLLER2 =
	// "ProportionalNonLinearMRPSDREController_FIRST";
	// private static final String CONTROLLER3 =
	// "ProportionalNonLinearMRPSDREController_SECOND";

	// IACLAW - SPIE
	// private static final String CONTROLLER1 =
	// "ProportionalNonLinearQuaternionSDREController_GIBBS";
	// private static final String CONTROLLER2 =
	// "ProportionalDerivativeLinearSunVectorController";
	// private static final String CONTROLLER3 = null;

	// STATESPACE
	// private static final String CONTROLLER1 =
	// "ProportionalNonLinearMRPSDREController_FIRST";
	// private static final String CONTROLLER2 =
	// "ProportionalDerivativeLinearSunVectorController";
	// private static final String CONTROLLER3 = "NopeController";

	// IACLAW - 2020
	private static final List<String> CONTROLLERS = new ArrayList<String>(
			Arrays.asList("ProportionalNonLinearQuaternionSDREController_GIBBS",
					"ProportionalNonLinearQuaternionFullSDREHInfinityController"));

	static final private Logger logger = LoggerFactory.getLogger(MultiSimulationController.class);

	// MONTE CARLO PARAMETERS
	private static final double MEAN_ANGLE = 0d;
	// private static final double BASE_ANGULAR_VELOCITY = .010d; //
	// SPACEOPS/ICEDYN
	// private static final double BASE_ANGULAR_VELOCITY = .11d; // IACLAW
	private static final double MEAN_ANGULAR_VELOCITY = 0d; // .087d; // SPIE

	// standard deviation
	private static final double STD_ANGLE = 0.001d;
	private static final double STD_ANGULAR_VELOCITY = 0.009d;

	// MONTE CARLO PARAMETERS - UNIFORM
	// CHANGED STRATEGY FOR MONTE CARLO FROM NORMAL TO UNIFORM
	// IAALACW - 2020
	private static final double LOWER_ANGLE = -180d;
	private static final double UPPER_ANGLE = 180d;
	private static final double LOWER_ANGULAR_VELOCITY = -0.15d;
	private static final double UPPER_ANGULAR_VELOCITY = 0.15d;

	// FOR STORING
	final List<SimulationController> listSimulations = new ArrayList<SimulationController>();
	final Map<String, List<SimulationController>> mapSimulations = new HashMap<String, List<SimulationController>>();

	/**
	 * @param monteCarlo
	 * @throws OrekitException
	 */
	public MultiSimulationController(int numberOfSimulations) throws OrekitException {
		logger.info("Configuring multi simulation... NumberOfTrials: {}", numberOfSimulations);
		
		RandomDataGenerator gaussianAnglesX = new RandomDataGenerator();
		RandomDataGenerator gaussianAnglesY = new RandomDataGenerator();
		RandomDataGenerator gaussianAnglesZ = new RandomDataGenerator();

		RandomDataGenerator gaussianVelocityX = new RandomDataGenerator();
		RandomDataGenerator gaussianVelocityY = new RandomDataGenerator();
		RandomDataGenerator gaussianVelocityZ = new RandomDataGenerator();
		for (int i = 1; i <= numberOfSimulations; i++) {

			//double[] initialAttitudeEulerAngles = new double[] { gaussianAnglesX.nextNormal(MEAN_ANGLE, STD_ANGLE),
			//		gaussianAnglesY.nextNormal(MEAN_ANGLE, STD_ANGLE),
			//		gaussianAnglesZ.nextNormal(MEAN_ANGLE, STD_ANGLE) };

			//double[] initialAngularVelocity = new double[] {
			//		gaussianVelocityX.nextNormal(MEAN_ANGULAR_VELOCITY, STD_ANGULAR_VELOCITY),
			//		gaussianVelocityY.nextNormal(MEAN_ANGULAR_VELOCITY, STD_ANGULAR_VELOCITY),
			//		gaussianVelocityZ.nextNormal(MEAN_ANGULAR_VELOCITY, STD_ANGULAR_VELOCITY) };

			double[] initialAttitudeEulerAngles = new double[] { gaussianAnglesX.nextUniform(LOWER_ANGLE, UPPER_ANGLE),
					gaussianAnglesY.nextUniform(LOWER_ANGLE, UPPER_ANGLE),
					gaussianAnglesZ.nextUniform(LOWER_ANGLE, UPPER_ANGLE) };

			double[] initialAngularVelocity = new double[] {
					gaussianVelocityX.nextUniform(LOWER_ANGULAR_VELOCITY, UPPER_ANGULAR_VELOCITY),
					gaussianVelocityY.nextUniform(LOWER_ANGULAR_VELOCITY, UPPER_ANGULAR_VELOCITY),
					gaussianVelocityZ.nextUniform(LOWER_ANGULAR_VELOCITY, UPPER_ANGULAR_VELOCITY) };

			logger.info("Monte Carlo iteration - Euler Angles: {} {} {}", initialAttitudeEulerAngles[0],
					initialAttitudeEulerAngles[1], initialAttitudeEulerAngles[2]);
			logger.info("Monte Carlo iteration - Angular Velocity: {} {} {}", initialAngularVelocity[0],
					initialAngularVelocity[1], initialAngularVelocity[2]);

			for (String controller : CONTROLLERS) {
				
				SimulationController s = new SimulationController(controller, initialAttitudeEulerAngles,
						initialAngularVelocity);
				listSimulations.add(s);
				List<SimulationController> l = mapSimulations.get(controller);
				if (l == null) {
					l = new ArrayList<SimulationController>();
					mapSimulations.put(controller, l);
				}
				l.add(s);
			}

		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Runnable#run()
	 */
	public void run() {
		long start = System.currentTimeMillis();

		// Get the ThreadFactory implementation to use
		ThreadFactory threadFactory = Executors.defaultThreadFactory();
		// creating the ThreadPoolExecutor
		ThreadPoolExecutor executorPool = new ThreadPoolExecutor(6, 6, 0, TimeUnit.SECONDS,
				new LinkedBlockingQueue<Runnable>(), threadFactory);

		// starting threads
		for (SimulationController s : listSimulations) {
			executorPool.execute(s);
		}

		logger.info("Total of {} simulations are scheduled to run in {} threads", listSimulations.size(),
				executorPool.getActiveCount());

		// joinning all threads
		executorPool.shutdown();
		while (!executorPool.isTerminated()) {
			try {
				Thread.sleep(10000);
				logger.info("{} simulations concluded from the total of {} in {} s",
						executorPool.getCompletedTaskCount(), listSimulations.size(),
						(System.currentTimeMillis() - start) / 1000d);
			} catch (InterruptedException e) {
				throw new RuntimeException("Simulation was interrupted", e);
			}
		}

		logger.info("**********************************");
		logger.info("{} simulations concluded from the total of {} in {} s", executorPool.getCompletedTaskCount(),
				listSimulations.size(), (System.currentTimeMillis() - start) / 1000d);
		logger.info("**********************************");

		// plotting
		NumberFormat numberFormatter = new DecimalFormat("##.00");
		for (String key : mapSimulations.keySet()) {
			String details = "";
			List<Map<Double, double[]>> quaternionError = new ArrayList<Map<Double, double[]>>();
			List<Map<Double, double[]>> angularVelocity = new ArrayList<Map<Double, double[]>>();
			Map<String, Map<Double, Double>> detControllability = new TreeMap<String, Map<Double, Double>>();
			List<Map<Double, double[]>> reactionWheelAngularVelocity = new ArrayList<Map<Double, double[]>>();
			Map<String, Map<Double, Double>> reactionWheelNormAngularMomentum = new TreeMap<String, Map<Double, Double>>();
			Map<String, Map<Double, double[]>> vetQuaternionError = new TreeMap<String, Map<Double, double[]>>();
			Map<String, Map<Double, double[]>> vetAngularVelocity = new TreeMap<String, Map<Double, double[]>>();
			Map<String, Map<Double, Double>> gama = new TreeMap<String, Map<Double, Double>>();
			for (SimulationController s : mapSimulations.get(key)) {
				String detail = "";
				if (s.initialAngularVelocity != null && s.initialAttitude != null) {
					detail = " initial attitude rad (" + numberFormatter.format(s.initialAttitude[0]) + ";"
							+ numberFormatter.format(s.initialAttitude[1]) + ";"
							+ numberFormatter.format(s.initialAttitude[2]) + ") initial angular velocity rad/s ("
							+ numberFormatter.format(s.initialAngularVelocity[0]) + ";"
							+ numberFormatter.format(s.initialAngularVelocity[1]) + ";"
							+ numberFormatter.format(s.initialAngularVelocity[2]) + ")";
				}
				quaternionError.add(s.stepHandler.quaternionError);
				angularVelocity.add(s.stepHandler.angularVelocityBody);
				detControllability.put(detail, s.stepHandler.detControllability);
				reactionWheelAngularVelocity.add(s.stepHandler.reactionWheelAngularVelocity);
				reactionWheelNormAngularMomentum.put(detail, s.stepHandler.reactionWheelNormAngularMomentum);
				vetQuaternionError.put(detail, s.stepHandler.stateSpaceQuaternions);
				vetAngularVelocity.put(detail, s.stepHandler.angularVelocityBody);
				gama.put(detail, s.stepHandler.gama);
				details += "\n" + detail;
			}
			details += "";
			logger.info("**** ");
			logger.info(details);
			Plotter.plot2DLine(quaternionError, key, "quaternion error", details);
			Plotter.plot2DLine(angularVelocity, key, "angular velocity", details);
			Plotter.plot2DLine(detControllability, key + " detControllability");
			Plotter.plot2DLine(reactionWheelAngularVelocity, key, "reaction wheel angular velocity", details);
			Plotter.plot2DLine(reactionWheelNormAngularMomentum, key + "reactionWheelAngularMomentum");

			// STATE SPACE
			Plotter.plot3DScatterStateSpace(vetQuaternionError, "state space quaternion - BODY");
			Plotter.plot3DScatterStateSpace(vetAngularVelocity, "state space velocity - BODY");

			Plotter.plot2DLine(gama, key + " gama");

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
		logger.info("Satellite Multi Simulation "
				+ (MultiSimulationController.class.getPackage().getImplementationVersion() == null ? ""
						: MultiSimulationController.class.getPackage().getImplementationVersion()));
		logger.info("**********************************");

		if (args.length > 0) {
			Integer numberOfSimulations = new Integer(args[0]);
			new MultiSimulationController(numberOfSimulations).run();
		} else {
			throw new RuntimeException("It should be informed the number of trials!");
		}

	}
}