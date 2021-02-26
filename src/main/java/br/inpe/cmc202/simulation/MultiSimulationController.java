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

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.RotationOrder;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.RealVector;
import org.hipparchus.random.RandomDataGenerator;
import org.hipparchus.stat.descriptive.DescriptiveStatistics;
import org.math.plot.utils.FastMath;
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
	//private static final List<String> CONTROLLERS = new ArrayList<String>(
	//		Arrays.asList("ProportionalLinearQuaternionPartialLQRController",
	//				"ProportionalNonLinearQuaternionSDREController_GIBBS",
	//				"ProportionalNonLinearQuaternionFullSDREHInfinityController"));

	private static final List<String> CONTROLLERS = new ArrayList<String>(
			Arrays.asList("ProportionalLinearQuaternionPartialLQRController",
					"ProportionalNonLinearQuaternionSDREController_GIBBS",
					"ProportionalNonLinearQuaternionFullSDREHInfinityController",
					"ProportionalNonLinearMRPSDREController_FIRST",
					"ProportionalNonLinearMRPSDREHInfinityController",
					"NopeController"));

	static final private Logger logger = LoggerFactory.getLogger(MultiSimulationController.class);

	// MONTE CARLO PARAMETERS
	//private static final double MEAN_ANGLE = 0d;
	// private static final double BASE_ANGULAR_VELOCITY = .010d; //
	// SPACEOPS/ICEDYN
	// private static final double BASE_ANGULAR_VELOCITY = .11d; // IACLAW
	//private static final double MEAN_ANGULAR_VELOCITY = 0d; // .087d; // SPIE

	// standard deviation
	//private static final double STD_ANGLE = 0.001d;
	//private static final double STD_ANGULAR_VELOCITY = 0.009d;

	// MONTE CARLO PARAMETERS - UNIFORM
	// CHANGED STRATEGY FOR MONTE CARLO FROM NORMAL TO UNIFORM
	// IAALACW - 2020 - CUBESAT
	private static final double LOWER_ANGLE = -180d;
	private static final double UPPER_ANGLE = 180d;
	// IAALACW - 2020 - CUBESAT
	// private static final double LOWER_ANGULAR_VELOCITY = -0.15d;
	// private static final double UPPER_ANGULAR_VELOCITY = 0.15d;
	// CILAMCE - 2020 - AMAZONIA1
	//private static final double LOWER_ANGULAR_VELOCITY = -0.02d;
	//private static final double UPPER_ANGULAR_VELOCITY = 0.02d;

	private static final double LOWER_ANGULAR_VELOCITY = -1E-9d; //GIBBS and GIBBS H-Infinity require the min angular velocity as 1E-4d while others not: LQR AND MRP
	private static final double UPPER_ANGULAR_VELOCITY = 1E-9d;

	// FOR STORING
	final List<SimulationController> listSimulations = new ArrayList<SimulationController>();
	final Map<String, List<SimulationController>> mapSimulations = new HashMap<String, List<SimulationController>>();

	// FOR STORING INITIAL CONDITIONS
	final Map<String, Map<Double, double[]>> initialAngles = new TreeMap<String, Map<Double, double[]>>();
	final Map<String, Map<Double, double[]>> initialAngularVelocities = new TreeMap<String, Map<Double, double[]>>();

	
	MultiSimulationController(){}

	/**
	 * @param monteCarlo
	 * @throws OrekitException
	 */
	public MultiSimulationController(int numberOfSimulations) throws OrekitException {
		logger.info("Configuring multi simulation... NumberOfTrials: {}", numberOfSimulations);
		
		initialAngles.put("initialAngles", new TreeMap<Double, double[]>());
		initialAngularVelocities.put("initialAngularVelocities", new TreeMap<Double, double[]>());

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
			
			// CHECKING BOUNDARIES for angular velocity
			final RealVector angVelocity = new ArrayRealVector(initialAngularVelocity);
			final SimulationController ss = new SimulationController("NopeController", initialAttitudeEulerAngles,
					initialAngularVelocity);
			RealVector max = ss.satellite.getMaximumAngularVelocityControllableByReactionWheels();
			if (angVelocity.getEntry(0) > max.getEntry(0) ||
				angVelocity.getEntry(1) > max.getEntry(1) ||
				angVelocity.getEntry(2) > max.getEntry(2)) {
				logger.warn("Monte Carlo iteration - Angular Velocity: {} {} {} OUT OF THE EXTERNAL BOUNDARIES OF THE DOMAIN OF ATTRACTION", initialAngularVelocity[0],
						initialAngularVelocity[1], initialAngularVelocity[2]);
			} else {

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
				
				// INITIAL CONDITIONS
				// showing initial Euler angles as rotations of the unit vector
				final Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.FRAME_TRANSFORM, initialAttitudeEulerAngles[0], 
						initialAttitudeEulerAngles[1], 
						initialAttitudeEulerAngles[2]);
				Vector3D init3d = rot.applyTo(new Vector3D(1,1,1));
				initialAngles.get("initialAngles").put((double)i, init3d.toArray());
				// angular velocities
				initialAngularVelocities.get("initialAngularVelocities").put((double)i, new double[] {
						initialAngularVelocity[0], 
						initialAngularVelocity[1], 
						initialAngularVelocity[2]});
			}

		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Runnable#run()
	 */
	public void run() {
		runSimulations();
		
		calculateStatistics();

		plotSimulations();
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
			Integer numberOfSimulations = Integer.parseInt(args[0]);
			new MultiSimulationController(numberOfSimulations).run();
		} else {
			throw new RuntimeException("It should be informed the number of trials!");
		}

	}
	
	//****
	protected void plotSimulations() {
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
			Map<String, Map<Double, Double>> conditionNumberA = new TreeMap<String, Map<Double, Double>>();
			Map<String, Map<Double, Double>> countNumericalErrors = new TreeMap<String, Map<Double, Double>>();
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
				conditionNumberA.put(detail, s.stepHandler.conditionNumberA);
				countNumericalErrors.put(detail, s.stepHandler.countNumericalErrors);
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

			// H-INFINITY
			Plotter.plot2DLine(gama, key + " gama");
			// Track numerical Errors
			Plotter.plot2DLine(conditionNumberA, key + " conditionNumberA");
			Plotter.plot2DLine(countNumericalErrors, key + " countNumericalErrors");

		}
		Plotter.plot3DScatterStateSpace(initialAngles, "initial Euler Angles (n = " + initialAngles.get("initialAngles").size() + ") for the unit vector");
		Plotter.plot3DScatterStateSpace(initialAngularVelocities, "initial Angular Velocities (n = " + initialAngularVelocities.get("initialAngularVelocities").size() + ")");
	}

	protected void runSimulations() {
		final long start = System.currentTimeMillis();
		final int numberOfProcessors = Runtime.getRuntime()
				.availableProcessors();

		// Get the ThreadFactory implementation to use
		ThreadFactory threadFactory = Executors.defaultThreadFactory();
		// creating the ThreadPoolExecutor
		ThreadPoolExecutor executorPool = new ThreadPoolExecutor(numberOfProcessors, numberOfProcessors, 0, TimeUnit.SECONDS,
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
	}

	
	protected void calculateStatistics() {
		logger.info("**********************************");
		logger.info("Computing statistics...");
		
		
		final Map<String, Map<Double, Double>> angularVelocityStd = new TreeMap<String, Map<Double, Double>>();
		final Map<String, Map<Double, Double>> vectorialQuaternionErrorStd = new TreeMap<String, Map<Double, Double>>();

		final Map<String, Map<Double, Double>> stateSpaceStd = new TreeMap<String, Map<Double, Double>>();

		// for each controller
		for (String controller : mapSimulations.keySet()) {
			double i = 0d;
			final Map<Double, Double> valuesAng = new TreeMap<Double, Double>();
			final Map<Double, Double> valuesQuat = new TreeMap<Double, Double>();
			final Map<Double, Double> valuesStateSpace = new TreeMap<Double, Double>();
			
			boolean convergenceAng = true;
			boolean convergenceQuat = true;
			boolean convergenceStateSpace = true;
			// for each simulation for a given controller
			for (SimulationController s : mapSimulations.get(controller)) {
				final DescriptiveStatistics normVectorialQuaternionError = new DescriptiveStatistics();
				final DescriptiveStatistics normAngularVelocity = new DescriptiveStatistics();
				final DescriptiveStatistics normStateSpace = new DescriptiveStatistics();
				
				// getting last 5% of time TO TEST CONVERGENCE
				final double tToTestConvergence = s.stepHandler.lastStoredTime - (s.stepHandler.lastStoredTime * .05d);
				logger.info("time to test convergence {}",tToTestConvergence);
				
				for (Double t : s.stepHandler.quaternionError.keySet()) {
					// calculate statistics of the norm of vectorial part of quaternion 
					final double[] quartenionError = s.stepHandler.quaternionError.get(t);
					final double normQ = new Vector3D(quartenionError[0],quartenionError[1],quartenionError[2]).getNorm();
					logger.debug("Norm of Vectorial Part of Quaternion Error {} {}",controller, normQ);
					normVectorialQuaternionError.addValue(normQ);
					if (t > tToTestConvergence && normQ > 1.E-2) {
						convergenceQuat = false;
					}

					// calculate statistics of the norm of angular velocity
					final double[] angularVelocity = s.stepHandler.angularVelocityBody.get(t);
					final double normV = new Vector3D(angularVelocity[0],angularVelocity[1],angularVelocity[2]).getNorm();
					logger.debug("Norm of Angular Velocity {} {}",controller, normV);
					normAngularVelocity.addValue(normV);
					if (t > tToTestConvergence && normV > 1.E-3) {
						convergenceAng = false;
					}
					
					// calculate statistics of the norm of state space: 
					// quaternion (last entry as scalar and adjusted to origin) 
					// and angular velocity
					final RealVector stateSpace = new ArrayRealVector(new double[] { 
							quartenionError[0],
							quartenionError[1],
							quartenionError[2], 
							1-FastMath.abs(quartenionError[3]), // adjusting to origin 0
							angularVelocity[0],
							angularVelocity[1],
							angularVelocity[2]});
					logger.info("Norm of StateSpace {} {}",controller, stateSpace.getNorm());
					normStateSpace.addValue(stateSpace.getNorm());
					if (t > tToTestConvergence && stateSpace.getNorm() > 1.E-2) {
						convergenceStateSpace = false;
					}
				}
								
				valuesQuat.put(++i, normVectorialQuaternionError.getPercentile(90d));
				valuesAng.put(i, normAngularVelocity.getPercentile(90d));
				valuesStateSpace.put(i, normStateSpace.getPercentile(90));
			}

			if (convergenceAng) {
				angularVelocityStd.put(controller, valuesAng);
			} else {
				angularVelocityStd.put(controller+"_UNSTABLE", valuesAng);
			}
			if (convergenceQuat) {
				vectorialQuaternionErrorStd.put(controller, valuesQuat);
			} else {
				vectorialQuaternionErrorStd.put(controller+ "_UNSTABLE", valuesQuat);
			}
			if (convergenceStateSpace) {
				stateSpaceStd.put(controller,  valuesStateSpace);
			} else {
				stateSpaceStd.put(controller+"_UNSTABLE",  valuesStateSpace);
			}			
		}

		logger.info("Statistics computed!");
		logger.info(angularVelocityStd.entrySet().toString());
		logger.info(vectorialQuaternionErrorStd.entrySet().toString());
		logger.info(stateSpaceStd.entrySet().toString());
		
		Plotter.plot2DLine(vectorialQuaternionErrorStd, "Statistics of Norm of Vectorial part of Quaternion Error");
		Plotter.plot2DLine(angularVelocityStd, "Statistics of Norm of Angular Velocity");
		Plotter.plot2DLine(stateSpaceStd, "Statistics of L2 Norm of State Space");
	}
}