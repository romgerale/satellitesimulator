package br.inpe.cmc202.simulation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;
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
 * CILAMCE2020
 * 
 * @author alessandro.g.romero
 * 
 */
public class MultiSimulationParametricUncertaintyPlusController implements Runnable {

	// CILAMCE 2020
	// "ProportionalNonLinearQuaternionSDREController_GIBBS",
	private static final List<String> CONTROLLERS = new ArrayList<String>(
			Arrays.asList(
					"ProportionalNonLinearQuaternionFullSDREHInfinityController"));

	static final private Logger logger = LoggerFactory
			.getLogger(MultiSimulationParametricUncertaintyPlusController.class);

	// MONTE CARLO PARAMETERS - GAUSSIAN
	// standard deviation for INERTIA TENSOR - 3 standard deviation equals 5%
	private static final double STD = 0.016666d;

	// UNIFORM 
	// CILAMCE - 2020
	private static final double LOWER_ANGLE = -180d;
	private static final double UPPER_ANGLE = 180d;
	private static final double LOWER_ANGULAR_VELOCITY = -0.01d;
	private static final double UPPER_ANGULAR_VELOCITY = 0.01d;

	// FOR STORING
	final List<SimulationController> listSimulations = new ArrayList<SimulationController>();
	final Map<String, List<SimulationController>> mapSimulations = new HashMap<String, List<SimulationController>>();

	/**
	 * @param monteCarlo
	 * @throws OrekitException
	 */
	public MultiSimulationParametricUncertaintyPlusController(int numberOfSimulations) throws OrekitException {
		logger.info("Configuring multi simulation... NumberOfTrials: {}", numberOfSimulations);

		RandomDataGenerator inertiaTensorRandom = new RandomDataGenerator();

		RandomDataGenerator gaussianAnglesX = new RandomDataGenerator();
		RandomDataGenerator gaussianAnglesY = new RandomDataGenerator();
		RandomDataGenerator gaussianAnglesZ = new RandomDataGenerator();

		RandomDataGenerator gaussianVelocityX = new RandomDataGenerator();
		RandomDataGenerator gaussianVelocityY = new RandomDataGenerator();
		RandomDataGenerator gaussianVelocityZ = new RandomDataGenerator();

		for (int i = 1; i <= numberOfSimulations; i++) {

			//final Properties inertiaTensor = calculateInertiaTensorUsingUniform(inertiaTensorRandom);
			final Properties inertiaTensor =
			 calculateInertiaTensorUsingNormal(inertiaTensorRandom);

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

			// CONTROLLERS
			for (String controller : CONTROLLERS) {
				SimulationController s = new SimulationController(controller, inertiaTensor, initialAttitudeEulerAngles,
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

	/**
	 * @param inertiaTensorRandom
	 * @return
	 */
	private Properties calculateInertiaTensorUsingNormal(RandomDataGenerator inertiaTensorRandom) {
		double v12 = inertiaTensorRandom.nextNormal(1.11d, 1.11d * STD);
		double v13 = inertiaTensorRandom.nextNormal(1.01d, 1.01d * STD);
		double v23 = inertiaTensorRandom.nextNormal(-0.35d, 0.35d * STD);
		// computing the random inertia tensor
		final Properties inertiaTensor = new Properties();
		inertiaTensor.put("inertiaMoment.1.1", Double.toString(inertiaTensorRandom.nextNormal(310d, 310d * STD)));
		inertiaTensor.put("inertiaMoment.1.2", Double.toString(v12));
		inertiaTensor.put("inertiaMoment.1.3", Double.toString(v13));
		inertiaTensor.put("inertiaMoment.2.1", Double.toString(v12));
		inertiaTensor.put("inertiaMoment.2.2", Double.toString(inertiaTensorRandom.nextNormal(360d, 360d * STD)));
		inertiaTensor.put("inertiaMoment.2.3", Double.toString(v23));
		inertiaTensor.put("inertiaMoment.3.1", Double.toString(v13));
		inertiaTensor.put("inertiaMoment.3.2", Double.toString(v23));
		inertiaTensor.put("inertiaMoment.3.3", Double.toString(inertiaTensorRandom.nextNormal(530.7d, 530.7d * STD)));
		logger.info("Monte Carlo iteration - Inertia Tensor: {} ", inertiaTensor);
		return inertiaTensor;
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
		ThreadPoolExecutor executorPool = new ThreadPoolExecutor(3, 3, 0, TimeUnit.SECONDS,
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
					detail = "Initial Attitude:" + Arrays.toString(s.initialAttitude) +
							 " Initial Angular Velocity: "+ Arrays.toString(s.initialAngularVelocity) +
							 " Inertia Tensor Nominal: "+ s.satellite.getI_nominal().toString()+
							 " Inertia Tensor Real: "+ s.satellite.getI().toString();
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
			//Plotter.plot2DLine(detControllability, key + " detControllability");
			Plotter.plot2DLine(reactionWheelAngularVelocity, key, "reaction wheel angular velocity", details);
			//Plotter.plot2DLine(reactionWheelNormAngularMomentum, key + "reactionWheelAngularMomentum");
			//Plotter.plot2DLine(gama, key + " gama");
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
		logger.info("Satellite Multi Simulation Parametric Uncertainty PLUS " + (MultiSimulationParametricUncertaintyPlusController.class
				.getPackage().getImplementationVersion() == null ? ""
						: MultiSimulationParametricUncertaintyPlusController.class.getPackage()
								.getImplementationVersion()));
		logger.info("**********************************");

		if (args.length > 0) {
			Integer numberOfSimulations = new Integer(args[0]);
			new MultiSimulationParametricUncertaintyPlusController(numberOfSimulations).run();
		} else {
			throw new RuntimeException("It should be informed the number of trials!");
		}

	}
}