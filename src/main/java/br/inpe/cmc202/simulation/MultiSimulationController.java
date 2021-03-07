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
	
	// FOR STORING
	final List<SimulationController> listSimulations = new ArrayList<SimulationController>();
	final Map<String, List<SimulationController>> mapSimulations = new HashMap<String, List<SimulationController>>();

	// FOR STORING INITIAL CONDITIONS
	final Map<String, Map<Double, double[]>> initialAngles = new TreeMap<String, Map<Double, double[]>>();
	final Map<String, Map<Double, double[]>> initialAngularVelocities = new TreeMap<String, Map<Double, double[]>>();
	final Map<String, Map<Double, double[]>> initialAnglesForVisualization = new TreeMap<String, Map<Double, double[]>>();

	
	MultiSimulationController(){}

	/**
	 * Constructor for the set of simulations.
	 * 
	 * @param numberOfSimulations
	 * @param approach for the definition of initial conditions (0 - NORNAL, 1 - UNIFORM, 2 - STATE SPACE EXPLORATION)
	 * @throws OrekitException
	 */
	public MultiSimulationController(int numberOfSimulations, int approach) throws OrekitException {
		logger.info("Configuring multi simulation... NumberOfTrials: {}", numberOfSimulations);
		
		computeInitialConditions(numberOfSimulations, approach);
		
		// getting the computed initial conditions
		final Map<Double, double[]> initialAnglesComputed = initialAngles.get("initialAngles");
		final Map<Double, double[]> initialAngularVelocitiesComputed = initialAngularVelocities.get("initialAngularVelocities");

		for (int i = 1; i <= initialAnglesComputed.size(); i++) {

			final double[] initialAttitudeEulerAngles = initialAnglesComputed.get((double)i);

			final double[] initialAngularVelocity = initialAngularVelocitiesComputed.get((double)i);

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

		int numberOfSimulations = 0;
		int approach = 1;
		if (args.length > 0) {
			numberOfSimulations = Integer.parseInt(args[0]);
			if (args.length > 1) {
				approach = Integer.parseInt(args[1]);
			}
		} else {
			throw new RuntimeException("It should be informed the number of trials!");
		}

		new MultiSimulationController(numberOfSimulations, approach).run();
	}
	
	//****
	/**
	 * For plotting simulations.
	 * 
	 */
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
		Plotter.plot3DScatterStateSpace(initialAnglesForVisualization, "initial Euler Angles (n = " + initialAnglesForVisualization.get("initialAnglesForVisualization").size() + ") for the unit vector");
		Plotter.plot3DScatterStateSpace(initialAngularVelocities, "initial Angular Velocities (n = " + initialAngularVelocities.get("initialAngularVelocities").size() + ")");
	}

	/**
	 * For running simulations.
	 * 
	 */
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

	
	/**
	 * For calculating statistics.
	 * 
	 */
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
					final RealVector quaternion = new ArrayRealVector(new double[] { 
							quartenionError[0],
							quartenionError[1],
							quartenionError[2], 
							1-FastMath.abs(quartenionError[3])}); // adjusting to origin 0
					logger.debug("Norm of Vectorial Part of Quaternion Error {} {}",controller, quaternion.getNorm());
					normVectorialQuaternionError.addValue(quaternion.getNorm());
					if (t > tToTestConvergence && quaternion.getNorm() > 1.E-2) {
						convergenceQuat = false;
					}

					// calculate statistics of the norm of angular velocity
					final double[] angularVelocityError = s.stepHandler.angularVelocityBody.get(t);
					final RealVector angularVelocity = new ArrayRealVector(new double[] { 
							angularVelocityError[0],
							angularVelocityError[1],
							angularVelocityError[2]});
					logger.debug("Norm of Angular Velocity {} {}",controller, angularVelocity.getNorm());
					normAngularVelocity.addValue(angularVelocity.getNorm());
					if (t > tToTestConvergence && angularVelocity.getNorm() > 1.E-3) {
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
							angularVelocityError[0],
							angularVelocityError[1],
							angularVelocityError[2]});
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
	
	//****
	/**
	 * For computing initial conditions for simulations.
	 * 
	 * @param numberOfSimulations
	 * @param approach for the definition of initial conditions (0 - NORNAL, 1 - UNIFORM, 2 - STATE SPACE EXPLORATION)
	 */
	protected void computeInitialConditions(int numberOfSimulations, int approach) throws OrekitException {
		
		logger.info("**********************************");
		logger.info("Computing initial conditions for {} number of simulations and approach {} (0 - NORNAL, 1 - UNIFORM, 2 - STATE SPACE EXPLORATION)...", numberOfSimulations, approach);

		initialAngles.put("initialAngles", new TreeMap<Double, double[]>());
		initialAngularVelocities.put("initialAngularVelocities", new TreeMap<Double, double[]>());
		initialAnglesForVisualization.put("initialAnglesForVisualization", new TreeMap<Double, double[]>());

		// for checking external boundaries of attractor
		final SimulationController ss = new SimulationController("NopeController", new double[] {0,0,0}, new double[] {0,0,0});
		
		//*********************************
		// INITIAL CONDITIONS 
		// MONTE CARLO PARAMETERS
		final double MEAN_ANGLE = 0d;
		// SPACEOPS/ICEDYN
		// private static final double BASE_ANGULAR_VELOCITY = .11d; // IACLAW // 0.01d SPACEOPS
		final double MEAN_ANGULAR_VELOCITY = 0d; // .087d; // SPIE

		// standard deviation
		final double STD_ANGLE = 0.001d;
		final double STD_ANGULAR_VELOCITY = 0.009d;

		// MONTE CARLO PARAMETERS - UNIFORM
		// CHANGED STRATEGY FOR MONTE CARLO FROM NORMAL TO UNIFORM
		// IAALACW - 2020 - CUBESAT
		//private static final double LOWER_ANGLE = -180d;
		//private static final double UPPER_ANGLE = 180d;
		final double LOWER_ANGLE = -180d; //-1E-9d;
		final double UPPER_ANGLE = 180d; //1E-9d;
		// IAALACW - 2020 - CUBESAT
		// private static final double LOWER_ANGULAR_VELOCITY = -0.15d;
		// private static final double UPPER_ANGULAR_VELOCITY = 0.15d;
		// CILAMCE - 2020 - AMAZONIA1
		//private static final double LOWER_ANGULAR_VELOCITY = -0.02d;
		//private static final double UPPER_ANGULAR_VELOCITY = 0.02d;

		final double LOWER_ANGULAR_VELOCITY = -1E-2d; //GIBBS and GIBBS H-Infinity require the min angular velocity as 1E-4d while others not: LQR AND MRP
		final double UPPER_ANGULAR_VELOCITY = 1E-2d; //1E-1 // too big

		// INITIAL CONDITIONS 
		//*********************************

		
		// NORMAL or UNIFORM
		if (approach <= 1) {
			
			final RandomDataGenerator gaussianAnglesX = new RandomDataGenerator();
			final RandomDataGenerator gaussianAnglesY = new RandomDataGenerator();
			final RandomDataGenerator gaussianAnglesZ = new RandomDataGenerator();

			final RandomDataGenerator gaussianVelocityX = new RandomDataGenerator();
			final RandomDataGenerator gaussianVelocityY = new RandomDataGenerator();
			final RandomDataGenerator gaussianVelocityZ = new RandomDataGenerator();
			
			for (int i = 1; i <= numberOfSimulations; i++) {
	
				final double[] initialAttitudeEulerAngles = new double[] {0,0,0};
				final double[] initialAngularVelocity = new double[] {0,0,0};
				
				if (approach == 0 ) {
					// NORMAL
					initialAttitudeEulerAngles[0] = gaussianAnglesX.nextNormal(MEAN_ANGLE, STD_ANGLE);
					initialAttitudeEulerAngles[1] = gaussianAnglesY.nextNormal(MEAN_ANGLE, STD_ANGLE);
					initialAttitudeEulerAngles[2] = gaussianAnglesZ.nextNormal(MEAN_ANGLE, STD_ANGLE);
							
					initialAngularVelocity[0] = gaussianVelocityX.nextNormal(MEAN_ANGULAR_VELOCITY, STD_ANGULAR_VELOCITY);
					initialAngularVelocity[1] = gaussianVelocityY.nextNormal(MEAN_ANGULAR_VELOCITY, STD_ANGULAR_VELOCITY);
					initialAngularVelocity[2] = gaussianVelocityZ.nextNormal(MEAN_ANGULAR_VELOCITY, STD_ANGULAR_VELOCITY);
					
				} else {
					// UNIFORM
					initialAttitudeEulerAngles[0] = gaussianAnglesX.nextUniform(LOWER_ANGLE, UPPER_ANGLE);
					initialAttitudeEulerAngles[1] = gaussianAnglesY.nextUniform(LOWER_ANGLE, UPPER_ANGLE);
					initialAttitudeEulerAngles[2] = gaussianAnglesZ.nextUniform(LOWER_ANGLE, UPPER_ANGLE);
							
					initialAngularVelocity[0] = gaussianVelocityX.nextUniform(LOWER_ANGULAR_VELOCITY, UPPER_ANGULAR_VELOCITY);
					initialAngularVelocity[1] = gaussianVelocityY.nextUniform(LOWER_ANGULAR_VELOCITY, UPPER_ANGULAR_VELOCITY);
					initialAngularVelocity[2] = gaussianVelocityZ.nextUniform(LOWER_ANGULAR_VELOCITY, UPPER_ANGULAR_VELOCITY);

				}
	
				// CHECKING BOUNDARIES for angular velocity
				final RealVector angVelocity = new ArrayRealVector(initialAngularVelocity);
				RealVector max = ss.satellite.getMaximumAngularVelocityControllableByReactionWheels();
				if (angVelocity.getEntry(0) > max.getEntry(0) ||
					angVelocity.getEntry(1) > max.getEntry(1) ||
					angVelocity.getEntry(2) > max.getEntry(2)) {
					logger.warn("Monte Carlo iteration - Angular Velocity: {} {} {} OUT OF THE EXTERNAL BOUNDARIES OF THE DOMAIN OF ATTRACTION", initialAngularVelocity[0],
							initialAngularVelocity[1], initialAngularVelocity[2]);
				} else {
					logger.info("Monte Carlo iteration - {} - Euler Angles (X,Y,Z): {} {} {} \t- Angular Velocity (X,Y,Z): {} {} {}", String.format("%4d", i), 
							String.format("%+3.3f", initialAttitudeEulerAngles[0]),
							String.format("%+3.3f", initialAttitudeEulerAngles[1]), 
							String.format("%+3.3f", initialAttitudeEulerAngles[2]), 
							String.format("%+.3f", initialAngularVelocity[0]),
							String.format("%+.3f", initialAngularVelocity[1]),
							String.format("%+.3f", initialAngularVelocity[2])
							);
					
					// INITIAL CONDITIONS
					initialAngles.get("initialAngles").put((double)i, initialAttitudeEulerAngles);
					// angular velocities
					initialAngularVelocities.get("initialAngularVelocities").put((double)i, initialAngularVelocity);
					// showing initial Euler angles as rotations of the unit vector
					final Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.FRAME_TRANSFORM, initialAttitudeEulerAngles[0], 
							initialAttitudeEulerAngles[1], 
							initialAttitudeEulerAngles[2]);
					Vector3D init3d = rot.applyTo(new Vector3D(1,1,1));
					initialAnglesForVisualization.get("initialAnglesForVisualization").put((double)i, init3d.toArray());
				}
	
			}
		} else {
			// EXPLORE STATE SPACE
			final long t = FastMath.round(FastMath.pow(numberOfSimulations, 1d/6d)); // 6 fors - 3 angles and 3 angular velocities
			if (t <= 1l) {
				throw new RuntimeException("Number of Simulations must be bigger than 64 for state space exploration (at least the boundaries)!");
			}
			int i = 0;
			for (int k = 0; k < t; k++) { // ANGLE X
								
				final double[] initialAttitudeEulerAngles = new double[] {0,0,0};
				final double[] initialAngularVelocity = new double[] {0,0,0};

				// computing
				final double range = FastMath.abs(LOWER_ANGLE) + FastMath.abs(UPPER_ANGLE);
				final double eulerAngleX = LOWER_ANGLE + k*range/(t-1);
				initialAttitudeEulerAngles[0] = eulerAngleX;

				for (int l = 0; l < t; l++) {  // ANGLE Y
					// computing
					final double eulerAngleY = LOWER_ANGLE + l*range/(t-1);
					initialAttitudeEulerAngles[1] = eulerAngleY;

					for (int m = 0; m < t; m++) {  // ANGLE Z
						// computing
						final double eulerAngleZ = LOWER_ANGLE + m*range/(t-1);
						initialAttitudeEulerAngles[2] = eulerAngleZ;

						for (int n = 0; n < t; n++) { // VELOCITY X
							
							// computing
							final double rangeVelocity = FastMath.abs(LOWER_ANGULAR_VELOCITY) + FastMath.abs(UPPER_ANGULAR_VELOCITY);
							final double angularVelocityX = LOWER_ANGULAR_VELOCITY + n*rangeVelocity/(t-1);
							initialAngularVelocity[0] = angularVelocityX;

							for (int o = 0; o < t; o++) { // VELOCITY Y

								// computing
								final double angularVelocityY = LOWER_ANGULAR_VELOCITY + o*rangeVelocity/(t-1);
								initialAngularVelocity[1] = angularVelocityY;
								
								for (int p = 0; p < t; p++) { // VELOCITY Z

									// computing
									final double angularVelocityZ = LOWER_ANGULAR_VELOCITY + p*rangeVelocity/(t-1);
									initialAngularVelocity[2] = angularVelocityZ;

									i++;
									
									// CHECKING BOUNDARIES for angular velocity
									final RealVector angVelocity = new ArrayRealVector(initialAngularVelocity);
									RealVector max = ss.satellite.getMaximumAngularVelocityControllableByReactionWheels();
									if (angVelocity.getEntry(0) > max.getEntry(0) ||
										angVelocity.getEntry(1) > max.getEntry(1) ||
										angVelocity.getEntry(2) > max.getEntry(2)) {
										logger.warn("State Space Exploration iteration - Angular Velocity: {} {} {} OUT OF THE EXTERNAL BOUNDARIES OF THE DOMAIN OF ATTRACTION", initialAngularVelocity[0],
												initialAngularVelocity[1], initialAngularVelocity[2]);
									} else {

										logger.info("State Space Exploration iteration - {} - Euler Angles (X,Y,Z): {} {} {} \t- Angular Velocity (X,Y,Z): {} {} {}", String.format("%4d", i), 
												String.format("%+3.3f", initialAttitudeEulerAngles[0]),
												String.format("%+3.3f", initialAttitudeEulerAngles[1]), 
												String.format("%+3.3f", initialAttitudeEulerAngles[2]), 
												String.format("%+.3f", initialAngularVelocity[0]),
												String.format("%+.3f", initialAngularVelocity[1]),
												String.format("%+.3f", initialAngularVelocity[2])
												);
	
										// INITIAL CONDITIONS
										initialAngles.get("initialAngles").put((double)i, initialAttitudeEulerAngles);
										// angular velocities
										initialAngularVelocities.get("initialAngularVelocities").put((double)i, initialAngularVelocity);
										// showing initial Euler angles as rotations of the unit vector
										final Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.FRAME_TRANSFORM, initialAttitudeEulerAngles[0], 
												initialAttitudeEulerAngles[1], 
												initialAttitudeEulerAngles[2]);
										Vector3D init3d = rot.applyTo(new Vector3D(1,1,1));
										initialAnglesForVisualization.get("initialAnglesForVisualization").put((double)i, init3d.toArray());
									}
								}
							}
						}
					}
				}
			}
		}
		logger.info("Initial conditions computed!");
	}

}