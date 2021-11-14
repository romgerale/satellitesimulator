package br.inpe.cmc202.simulation;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import org.hipparchus.exception.MathIllegalStateException;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.RotationOrder;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.geometry.euclidean.twod.PolygonsSet;
import org.hipparchus.geometry.euclidean.twod.Vector2D;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.hipparchus.random.RandomDataGenerator;
import org.hipparchus.stat.descriptive.DescriptiveStatistics;
import org.math.plot.utils.FastMath;
import org.orekit.errors.OrekitException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.satellite.Satellite;
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
					"ProportionalNonLinearQuaternionFullSDREHInfinityController"));
					//"ProportionalNonLinearMRPSDREController_FIRST"));
					//"ProportionalNonLinearMRPSDREHInfinityController"));
					//"NopeController"));

	static final private Logger logger = LoggerFactory.getLogger(MultiSimulationController.class);
	
	// FOR STORING
	final List<Runnable> listSimulations = new ArrayList<Runnable>();
	final Map<String, List<SimulationController>> mapSimulations = new HashMap<String, List<SimulationController>>();
	final Map<String, List<SimulationController>> mapSimulationsNotConverged = new HashMap<String, List<SimulationController>>();

	// FOR STORING RUNNABLE QUEUE
	// queue
	ThreadPoolExecutor executorPool;

	// FOR STORING INITIAL CONDITIONS
	final Map<String, Map<Double, double[]>> initialAngles = new TreeMap<String, Map<Double, double[]>>();
	final Map<String, Map<Double, double[]>> initialAngularVelocities = new TreeMap<String, Map<Double, double[]>>();
	
	protected int headless = 0;
	
	private final static String FORMAT = "#0.0000";
	
	/**
	 * Default constructor.
	 */
	MultiSimulationController(){}

	/**
	 * Constructor for the set of simulations.
	 * 
	 * @param numberOfSimulations
	 * @param approach for the definition of initial conditions (0 - NORNAL, 1 - UNIFORM, 2 - STATE SPACE EXPLORATION)
	 * @throws OrekitException
	 */
	public MultiSimulationController(int numberOfSimulations, int approach, int headless) throws OrekitException {		
		logger.info("----------------------------");
		logger.info("Configuring multi simulation (headless:{})... number of simulations: {}", headless, numberOfSimulations);
		this.headless = headless;

		computeInitialConditions(numberOfSimulations, approach);
		// getting the computed initial conditions
		final Map<Double, double[]> initialAnglesComputed = initialAngles.get("initialAngles");
		final Map<Double, double[]> initialAngularVelocitiesComputed = initialAngularVelocities.get("initialAngularVelocities");

		for (int i = 1; i <= initialAnglesComputed.size(); i++) {

			final double[] initialAttitudeEulerAngles = initialAnglesComputed.get((double)i);

			final double[] initialAngularVelocity = initialAngularVelocitiesComputed.get((double)i);

			for (final String controller : CONTROLLERS) {
				
				final SimulationController s = new SimulationController(controller, initialAttitudeEulerAngles,
						initialAngularVelocity);
				
				Runnable s2 = new SimulationControllerRunnable(s, mapSimulations, mapSimulationsNotConverged);
						
				listSimulations.add(s2);
			}

		}
		logger.info("{} simulations configured!", listSimulations.size());
		logger.info("----------------------------");
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Runnable#run()
	 */
	public void run() {
		runSimulations();
		
		computeResults(mapSimulations, mapSimulationsNotConverged, 0d, true);

		plotSimulations(mapSimulations);
		
		plotDomainOfAttraction(mapSimulations, "CONVERGED", false);
		plotDomainOfAttraction(mapSimulationsNotConverged, "NOT CONVERGED", false);
		
		try {
			if (headless == 1) {
				logger.info("Headless configured! Exiting...");
				Thread.sleep(10000);
				System.exit(0);
			}
		} catch (InterruptedException ie) {
			// ignore
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

		int numberOfSimulations = 0;
		int approach = 1;
		int headless = 0;
		if (args.length > 0) {
			numberOfSimulations = Integer.parseInt(args[0]);
			if (args.length > 1) {
				approach = Integer.parseInt(args[1]);
			}
			if (args.length > 2) {
				headless = Integer.parseInt(args[2]);
			}
		} else {
			throw new RuntimeException("It must be informed the number of trials!");
		}

		new MultiSimulationController(numberOfSimulations, approach, headless).run();
	}
	
	//****
	/**
	 * For plotting simulations.
	 * 
	 */
	protected void plotSimulations(Map<String, List<SimulationController>> mapSimulations) {
		logger.info("----------------------------");
		logger.info("Plotting results...");
		
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
			// logger.info("**** ");
			// logger.info(details);
			Plotter.plot2DLine(quaternionError, "Simulations - " + key + " quaternion error", "quaternion error", details);
			Plotter.plot2DLine(angularVelocity, "Simulations - " + key + " angular velocity", "angular velocity", details);
			Plotter.plot2DLine(detControllability, "Simulations - " + key + " detControllability", false);
			Plotter.plot2DLine(reactionWheelAngularVelocity, "Simulations - " + key + " reaction wheel angular velocity", " reaction wheel angular velocity", details);
			Plotter.plot2DLine(reactionWheelNormAngularMomentum, "Simulations - " + key + " reactionWheelAngularMomentum", false);

			// STATE SPACE
			//Plotter.plot3DScatterStateSpace(vetQuaternionError, "Simulations - " + key + " state space quaternion - BODY", false);
			//Plotter.plot3DScatterStateSpace(vetAngularVelocity, "Simulations - " + key + " state space velocity - BODY", false);

			// H-INFINITY
			Plotter.plot2DLine(gama, "Simulations - " + key + " gamma", false);
			// Track numerical Errors
			Plotter.plot2DLine(conditionNumberA, "Simulations - " + key + " conditionNumberA", false);
			Plotter.plot2DLine(countNumericalErrors, "Simulations - " + key + " countNumericalErrors", false);

		}
		logger.info("Results plotted!");
		logger.info("----------------------------");
	}

	/**
	 * For running simulations.
	 * 
	 */
	protected void runSimulations() {
		
		logger.info("----------------------------");
		logger.info("Starting simulations {}...", listSimulations.size());

		final long start = System.currentTimeMillis();
		final int numberOfProcessors = Runtime.getRuntime()
				.availableProcessors();

		// Get the ThreadFactory implementation to use
		ThreadFactory threadFactory = Executors.defaultThreadFactory();
		// creating the ThreadPoolExecutor
		executorPool = new ThreadPoolExecutor(numberOfProcessors, numberOfProcessors, 0, TimeUnit.SECONDS,
				new LinkedBlockingQueue<Runnable>(), threadFactory);

		// starting threads
		for (Runnable s : listSimulations) {
			executorPool.execute(s);
		}

		logger.info("Total of {} simulations are scheduled to run in {} threads", listSimulations.size(),
				executorPool.getActiveCount());

		// joinning all threads
		executorPool.shutdown();
		while (!executorPool.isTerminated()) {
			try {
				Thread.sleep(10000);
				logger.info("{} simulations concluded from the total of {} in {} s (queued: {})",
						executorPool.getCompletedTaskCount(), listSimulations.size(),
						(System.currentTimeMillis() - start) / 1000d,
						executorPool.getQueue().size());
			} catch (InterruptedException e) {
				throw new RuntimeException("Simulation was interrupted", e);
			}
		}

		logger.info("----------------------------");
		logger.info("{} simulations concluded from the total of {} in {} s", executorPool.getCompletedTaskCount(),
				listSimulations.size(), (System.currentTimeMillis() - start) / 1000d);
		logger.info("----------------------------");
	}

	
	/**
	 * For calculating statistics.
	 * 
	 */
	protected void computeResults(Map<String, List<SimulationController>> mapSimulations, 
			Map<String, List<SimulationController>> mapSimulationsNotConverged, Double p,
			boolean plotStatistics) {
		logger.info("----------------------------");
		logger.info("Computing results...");
			
		boolean someSimulationPolygon = false;

		// ENFORCING polygon for norm FOR NON CONVERGENT (consequently, conservative results)
		// NOTE BETWEEN CONVERGENT RESULTS CAN EMERGE "HOLES"
		// iterating over not converged 
		// for each controller
		boolean done = false;
		while (!done) {
			done = true;
			main:
			for (String controller : new TreeSet<String>(mapSimulationsNotConverged.keySet())) {
	
				List<SimulationController> l = mapSimulations.get(controller);
				if (l == null) {
					l = new ArrayList<SimulationController>();
					mapSimulations.put(controller, l);
				}
	
				List<SimulationController> lnc = mapSimulationsNotConverged.get(controller);
				if (lnc == null) {
					lnc = new ArrayList<SimulationController>();
					mapSimulationsNotConverged.put(controller, lnc);
				}
	
				// for each simulation for a given controller
				List<SimulationController> lnc2check = new ArrayList<SimulationController>(lnc);
				for (SimulationController s : lnc2check) {
					
					final double normEulerAngles = s.getEulerAnglesOfInitialCondition().getNorm();
					final double normAngVelocity = s.getAngularVelocityOfInitialCondition().getNorm();
					final Satellite satelliteNC = s.satellite;
	
					// searching in convergent to move to NOT CONVERGED if it does not satisfy the condition (polygon)
					List<SimulationController> l2check = new ArrayList<SimulationController>(l);
					for (SimulationController s2check : l2check) {
						// it ran
						if (s2check.ran()) {
							//final String controllerNameToRun = s2check.satellite.getReactionWheelControllerName();
							final Satellite satelliteC = s2check.satellite;
							final double normEulerAnglesToRun = s2check.getEulerAnglesOfInitialCondition().getNorm();
							final double normAngVelocityToRun = s2check.getAngularVelocityOfInitialCondition().getNorm();
							if (satelliteC.equalsStructurallly(satelliteNC) &&
									normEulerAnglesToRun > normEulerAngles 
									&& normAngVelocityToRun > normAngVelocity) {
								logger.info("Found a simulation that is CONVERGENT but it does not satisfy POLYGON for NORM: {} Norm Euler Angles {} Norm Angular Velocity {} - REFERENCE Norm Euler Angles {} Norm Angular Velocity {}",
										satelliteC, normEulerAnglesToRun, normAngVelocityToRun, normEulerAngles, normAngVelocity);
								// adding to not converged
								lnc.add(s2check);
								// removing from converged
								l.remove(s2check);
								done = false;
								someSimulationPolygon = true;
								break main; // start again
							}
						} else {
							logger.error("It is not possible to have a convergent simulation that did not run!");
						}
					}
				}
			}
		}
		
		// COMPUTING measures about norm of state space and norm of reaction wheel angular momentum
		{
			boolean someValueComputed = false;
			
			final Map<String, Map<Double, double[]>> stateSpaceStats = new TreeMap<String, Map<Double, double[]>>();
			final Map<String, Map<Double, double[]>> reactionWheelStats = new TreeMap<String, Map<Double, double[]>>();
	
			// for each controller
			for (String controller : mapSimulations.keySet()) {
				double i = 0d;
				stateSpaceStats.put(controller, new TreeMap<Double, double[]>());
				reactionWheelStats.put(controller, new TreeMap<Double, double[]>());
	
				// for each simulation for a given controller
				for (SimulationController s : mapSimulations.get(controller)) {
					final DescriptiveStatistics statStateSpace = new DescriptiveStatistics();
					final DescriptiveStatistics statReactionWheel = new DescriptiveStatistics();
					
					for (Double t : s.stepHandler.quaternionError.keySet()) {
						final double[] quartenionError = s.stepHandler.quaternionError.get(t);
						final double[] angularVelocityError = s.stepHandler.angularVelocityBody.get(t);
						final double reactionWheelAngularMomentum = s.stepHandler.reactionWheelNormAngularMomentum.get(t);
						
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
						logger.debug("Norm of StateSpace {} {}",controller, stateSpace.getNorm());
						statStateSpace.addValue(stateSpace.getNorm());
						
						statReactionWheel.addValue(reactionWheelAngularMomentum);
						
						someValueComputed = true;
					}
													
					stateSpaceStats.get(controller).put(++i, new double[] {
							statStateSpace.getMean(),
							statStateSpace.getStandardDeviation()});
	
					reactionWheelStats.get(controller).put(++i, new double[] {
							statReactionWheel.getMean(),
							statReactionWheel.getStandardDeviation()});
				}
	
			}
	
			if (someValueComputed && plotStatistics) {
				// computing mean and standard deviation of overall mean by each controller
				// for each controller
				final Map<String, Map<Double, double[]>> stateSpaceStatsForGraph = new TreeMap<String, Map<Double, double[]>>();
				final Map<String, Map<Double, double[]>> reactionWheelStatsForGraph = new TreeMap<String, Map<Double, double[]>>();
	
				for (String controller : stateSpaceStats.keySet()) {
					final DescriptiveStatistics statStateSpaceOverall = new DescriptiveStatistics();
					final DescriptiveStatistics statReactionWheelOverall = new DescriptiveStatistics();
					for (final double i: stateSpaceStats.get(controller).keySet()) {
						final double[] meanStd = stateSpaceStats.get(controller).get(i);
						statStateSpaceOverall.addValue(meanStd[0]);
					}
					for (final double i: reactionWheelStats.get(controller).keySet()) {
						final double[] meanStd = reactionWheelStats.get(controller).get(i);
						statReactionWheelOverall.addValue(meanStd[0]);
					}
					stateSpaceStatsForGraph.put(controller + " (Mean=" + statStateSpaceOverall.getMean() + 
							                                 ",Std=" + statStateSpaceOverall.getStandardDeviation() + 
							                                 ",Samples="+statStateSpaceOverall.getN()+")", 
							                                 stateSpaceStats.get(controller));
					reactionWheelStatsForGraph.put(controller + " (Mean="+statReactionWheelOverall.getMean() + 
															 ",Std=" + statReactionWheelOverall.getStandardDeviation() + 
															 ",Samples="+statReactionWheelOverall.getN()+")", 
							                                 reactionWheelStats.get(controller));
				}
	
				//Plotter.plot2DScatter(stateSpaceStatsForGraph, "Statistics of L2 Norm of State Space - CONVERGED " + p,
				//		new String[] {"mean of Norm", "standard deviation of Norm"});
				//Plotter.plot2DScatter(reactionWheelStatsForGraph, "Statistics of L2 Norm of Reaction Wheel Angular Momentum - CONVERGED " + p,
				//		new String[] {"mean of Norm", "standard deviation of Norm"});
			}
			logger.info("Results computed {} about norm of state space and norm of reaction wheel angular momentum!", someValueComputed);
		}
		
		// COMPUTING measures about 
		// 1) sum of norm of state space and sum of norm of control torque (reaction wheel control torque)
		// 2) sum of norm of actual x ideal control
		// 3) gamma for H-infinity (min and max)
		{
			boolean someValueComputed = false;
			
			final Map<String, Map<Double, double[]>> optimalStateControl = new TreeMap<String, Map<Double, double[]>>();
			final Map<String, Map<Double, double[]>> optimalIdealActualTorque = new TreeMap<String, Map<Double, double[]>>();
			final Map<String, Map<Double, double[]>> minMaxGamma = new TreeMap<String, Map<Double, double[]>>();
	
			// for each controller
			for (String controller : mapSimulations.keySet()) {
				double i = 0d;
				optimalStateControl.put(controller, new TreeMap<Double, double[]>());
				optimalIdealActualTorque.put(controller, new TreeMap<Double, double[]>());
				minMaxGamma.put(controller, new TreeMap<Double, double[]>());
				
				// for each simulation for a given controller
				for (SimulationController s : mapSimulations.get(controller)) {
					double sumNormStateSpace = 0d;
					double sumNormActualControl = 0d;
					double sumNormIdealControl = 0d;
					double minGamma = Double.MAX_VALUE;
					double maxGamma = 0d;
					
					for (Double t : s.stepHandler.quaternionError.keySet()) {
						final double[] quartenionError = s.stepHandler.quaternionError.get(t);
						final double[] angularVelocityError = s.stepHandler.angularVelocityBody.get(t);
						final double[] actualControl = s.stepHandler.reactionWheelTorque.get(t);
						final double[] idealControl = s.stepHandler.reactionWheelDesiredTorque.get(t);
						final double gamma = s.stepHandler.gama.get(t);
						
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
						logger.debug("Norm of StateSpace {} {}",controller, stateSpace.getNorm());

						final RealVector actualControlV = new ArrayRealVector(actualControl); 
						final RealVector idealControlV = new ArrayRealVector(idealControl); 

						// computing
						sumNormStateSpace += stateSpace.getNorm();
						sumNormActualControl += actualControlV.getNorm();
						sumNormIdealControl += idealControlV.getNorm();
						
						// computing gamma
						if (gamma <= minGamma) {
							minGamma = gamma;
						}
						if (gamma >= maxGamma) {
							maxGamma = gamma;
						}

						someValueComputed = true;
					}
													
					optimalStateControl.get(controller).put(++i, new double[] {
							sumNormStateSpace,
							sumNormActualControl});
	
					optimalIdealActualTorque.get(controller).put(++i, new double[] {
							sumNormActualControl,
							sumNormIdealControl});

					if (minGamma != 0d || maxGamma != 0d || minMaxGamma.get(controller).size() == 0) {
						minMaxGamma.get(controller).put(++i, new double[] {
							minGamma,
							maxGamma});
					}
				}
	
			}
	
			if (someValueComputed && plotStatistics) {
				final DecimalFormat df = new DecimalFormat(FORMAT);

				// computing mean and standard deviation of overall sum by each controller
				// for each controller
				final Map<String, Map<Double, double[]>> optimalStateControlForGraph = new TreeMap<String, Map<Double, double[]>>();
				final Map<String, Map<Double, double[]>> minMaxGammaForGraph = new TreeMap<String, Map<Double, double[]>>();
	
				for (String controller : optimalStateControl.keySet()) {
					final DescriptiveStatistics statStateSpaceOverall = new DescriptiveStatistics();
					for (final double i: optimalStateControl.get(controller).keySet()) {
						final double[] xy = optimalStateControl.get(controller).get(i);
						statStateSpaceOverall.addValue(xy[0] + xy[1]);
					}
					
					double minGamma = Double.MAX_VALUE;
					double maxGamma = 0d;
					for (final double i: minMaxGamma.get(controller).keySet()) {
						final double[] gamma = minMaxGamma.get(controller).get(i);
						
						// computing gamma
						if (gamma[0] <= minGamma) {
							minGamma = gamma[0];
						}
						if (gamma[1] >= maxGamma) {
							maxGamma = gamma[1];
						}
					}
					
					optimalStateControlForGraph.put(controller + " (SUM - Mean=" + statStateSpaceOverall.getMean() + 
							                                 ",Std=" + statStateSpaceOverall.getStandardDeviation() + 
							                                 ",Samples="+statStateSpaceOverall.getN()+")", 
							                                 optimalStateControl.get(controller));

					minMaxGammaForGraph.put(controller + " (Min=" + df.format(minGamma) + 
                            ",Max=" + df.format(maxGamma) + ")", 
                            minMaxGamma.get(controller));
				}
	
				//Plotter.plot2DScatter(optimalStateControlForGraph, "Statistics of Optimality - CONVERGED " + p,
				//		new String[] {"sum of Norm of state", "sum of Norm of control"});
				Plotter.plot2DScatter(minMaxGammaForGraph, "Results - Min Gamma vs Max Gamma - CONVERGED " + p,
						new String[] {"min Gamma", "max Gamma"});
				//Plotter.plot2DScatter(optimalIdealActualTorque, "Norm of Actual Control vs Norm of Computed Control - CONVERGED " + p,
				//		new String[] {"sum of Norm of Actual Control", "sum of Norm of Computed Control"});
			}
			logger.info("Results computed {} about sum of norm of state space and sum of norm of control torque (reaction wheel control torque)!", someValueComputed);
		}
		
		// COMPUTING measures about 
		// 1)  \int x^TQx + u^TRu
		// 2)  \int u^TRu (computed/actual)
		{
			boolean someValueComputed = false;
			
			final RealMatrix Q = MatrixUtils.createRealIdentityMatrix(7);
			final RealMatrix R = MatrixUtils.createRealIdentityMatrix(3);
			
			final Map<String, Map<Double, double[]>> optimalStateControl = new TreeMap<String, Map<Double, double[]>>();
			final Map<String, Map<Double, double[]>> optimalIdealActualTorque = new TreeMap<String, Map<Double, double[]>>();
	
			double maxIntControl = 0;
			
			// for each controller
			for (String controller : mapSimulations.keySet()) {
				double i = 0d;
				optimalStateControl.put(controller, new TreeMap<Double, double[]>());
				optimalIdealActualTorque.put(controller, new TreeMap<Double, double[]>());
				
				// for each simulation for a given controller
				for (SimulationController s : mapSimulations.get(controller)) {
					double intStateSpace = 0d;
					double intActualControl = 0d;
					double intIdealControl = 0d;
					
					for (Double t : s.stepHandler.quaternionError.keySet()) {
						final double[] quartenionError = s.stepHandler.quaternionError.get(t);
						final double[] angularVelocityError = s.stepHandler.angularVelocityBody.get(t);
						final double[] actualControl = s.stepHandler.reactionWheelTorque.get(t);
						final double[] idealControl = s.stepHandler.reactionWheelDesiredTorque.get(t);
						final double step = s.step;
						
						// state space: 
						// quaternion (last entry as scalar and adjusted to origin) 
						// and angular velocity
						final RealMatrix stateSpace = MatrixUtils.createColumnRealMatrix(new double[] { 
								quartenionError[0],
								quartenionError[1],
								quartenionError[2], 
								1-FastMath.abs(quartenionError[3]), // adjusting to origin 0
								angularVelocityError[0],
								angularVelocityError[1],
								angularVelocityError[2]});

						final RealMatrix actualControlV = MatrixUtils.createColumnRealMatrix(actualControl); 
						final RealMatrix idealControlV = MatrixUtils.createColumnRealMatrix(idealControl); 

						// integrating
						intStateSpace += stateSpace.transpose().multiply(Q).multiply(stateSpace).scalarMultiply(step).getEntry(0, 0);
						intActualControl += actualControlV.transpose().multiply(R).multiply(actualControlV).scalarMultiply(step).getEntry(0, 0);
						intIdealControl += idealControlV.transpose().multiply(R).multiply(idealControlV).scalarMultiply(step).getEntry(0, 0);
												
						someValueComputed = true;
					}
													
					optimalStateControl.get(controller).put(++i, new double[] {
							intStateSpace,
							intActualControl});
	
					optimalIdealActualTorque.get(controller).put(++i, new double[] {
							intIdealControl,
							intActualControl});
					
					if (intIdealControl > maxIntControl) {
						maxIntControl = intIdealControl;
					}
					if (intActualControl > maxIntControl) {
						maxIntControl = intActualControl;
					}

				}
	
			}
	
			if (someValueComputed && plotStatistics) {
				final DecimalFormat df = new DecimalFormat(FORMAT);
				// computing mean and standard deviation of overall integral by each controller
				final Map<String, Map<Double, double[]>> optimalStateControlForGraph = new TreeMap<String, Map<Double, double[]>>();
				for (String controller : optimalStateControl.keySet()) {
					final DescriptiveStatistics statStateSpaceOverall = new DescriptiveStatistics();
					for (final double i: optimalStateControl.get(controller).keySet()) {
						final double[] xy = optimalStateControl.get(controller).get(i);
						
						// 1/2 *  \int x^TQx + u^TRu
						statStateSpaceOverall.addValue(.5d * (xy[0] + xy[1]));
					}

					optimalStateControlForGraph.put(controller + " (1/2 SUM - Mean=" + df.format(statStateSpaceOverall.getMean()) + 
							                                 ",Std=" + df.format(statStateSpaceOverall.getStandardDeviation()) + 
							                                 ",Samples="+statStateSpaceOverall.getN()+")", 
							                                 optimalStateControl.get(controller));
				}

				//creating the bisectrix for the actual/computed control
				TreeMap<Double, double[]> bisectrix = new TreeMap<Double, double[]>();
				optimalIdealActualTorque.put("bisectrix", bisectrix);
				for (double delta = 0; delta < maxIntControl;  delta += (maxIntControl/30d) ) {
					bisectrix.put(delta, new double[] { delta, delta});
				}

				Plotter.plot2DScatter(optimalStateControlForGraph, "Results - Statistics of Optimality (Integral) - CONVERGED " + p,
						new String[] {"integral statespace", "integral actual control"});
				Plotter.plot2DScatter(optimalIdealActualTorque, "Results - Integral of Actual Control vs Integral of Computed Control - CONVERGED " + p,
						new String[] {"integral computed control", "integral actual control"});
			}
			logger.info("Results computed {} about integral of state space and integral of control torque (reaction wheel control torque)!", someValueComputed);
		}
		
		logger.info("Polygon enforced {}!", someSimulationPolygon);
		logger.info("----------------------------");
	}
	
	
	/**
	 * For plotting the domain of attraction.
	 * 
	 */
	protected Map<String, Map<Double, Double>> plotDomainOfAttraction(Map<String, List<SimulationController>> simulations, 
			String label, 
			boolean plotAttitudeAndAngularVelocity) {
		logger.info("----------------------------");
		logger.info("Plotting Domain of Attraction ({})...", label);
		
		boolean hasSomeDomainOfAttraction = false;

		final Map<String, Map<Double, double[]>> domainOfAttractionNorm = new TreeMap<String, Map<Double, double[]>>();
		final Map<String, Map<Double, double[]>> domainOfAttractionAttitude = new TreeMap<String, Map<Double, double[]>>();
		final Map<String, Map<Double, double[]>> domainOfAttractionAngularVelocity = new TreeMap<String, Map<Double, double[]>>();
		final Map<String, Map<Double, Double>> domainOfAttractionNormShape = new TreeMap<String, Map<Double, Double>>();
		final Map<String, Map<Double, Double>> domainOfAttractionNormShapeForGraph = new TreeMap<String, Map<Double, Double>>();

		// for each controller
		for (String controller : simulations.keySet()) {
			double i = 0d;

			domainOfAttractionNorm.put(controller, new TreeMap<Double, double[]>());
			domainOfAttractionAttitude.put(controller, new TreeMap<Double, double[]>());
			domainOfAttractionAngularVelocity.put(controller, new TreeMap<Double, double[]>());

			// for each simulation for a given controller
			for (SimulationController s : simulations.get(controller)) {

				final RealVector initialConditionEulerAnglesV = s.getEulerAnglesOfInitialCondition();
				final RealVector initialConditionAngularVelocity = s.getAngularVelocityOfInitialCondition();
				
				logger.info("Inside domain of attraction ({})! Run: {}, Converged: {}, Controller: {}, Initial Euler Angles: {}, Norm - Initial Euler Angles: {}, Initial Angular Velocity: {}, Norm - Initial Angular Velocity: {}", 
						label, 
						s.ran(),
						s.ran() ? s.checkConvergence() : false, 
						controller,
						initialConditionEulerAnglesV, 
						initialConditionEulerAnglesV.getNorm(), 
						initialConditionAngularVelocity,
						initialConditionAngularVelocity.getNorm());
				
				hasSomeDomainOfAttraction = true;
				domainOfAttractionNorm.get(controller).put(++i, new double[] {
						initialConditionEulerAnglesV.getNorm(),
						initialConditionAngularVelocity.getNorm()});
				domainOfAttractionAttitude.get(controller).put(i, initialConditionEulerAnglesV.toArray());
				domainOfAttractionAngularVelocity.get(controller).put(i, 
						initialConditionAngularVelocity.toArray());

			}
		}

		if (hasSomeDomainOfAttraction) {
			// logging the results and trying to compute a polygon for the domain of attraction 
			for (String controller : simulations.keySet()) {
				
				int ran = 0;
				int notRan = 0;
				double maxAbsEulerAngleX = 0d;
				double maxAbsEulerAngleY = 0d;
				double maxAbsEulerAngleZ = 0d;
				double maxAbsAngVelX = 0d;
				double maxAbsAngVelY = 0d;
				double maxAbsAngVelZ = 0d;
				
				// for each simulation for a given controller
				for (SimulationController s : simulations.get(controller)) {
					if (s.ran()) {
						ran++;
					} else {
						notRan ++;
					}
					
					final double[] initialConditionEulerAnglesV = s.getEulerAnglesOfInitialCondition().toArray();
					final double[] initialConditionAngularVelocity = s.getAngularVelocityOfInitialCondition().toArray();

					if (maxAbsEulerAngleX < FastMath.abs(initialConditionEulerAnglesV[0])) {
						maxAbsEulerAngleX = FastMath.abs(initialConditionEulerAnglesV[0]);
					}
					if (maxAbsEulerAngleY < FastMath.abs(initialConditionEulerAnglesV[1])) {
						maxAbsEulerAngleY = FastMath.abs(initialConditionEulerAnglesV[1]);
					}
					if (maxAbsEulerAngleZ < FastMath.abs(initialConditionEulerAnglesV[2])) {
						maxAbsEulerAngleZ = FastMath.abs(initialConditionEulerAnglesV[2]);
					}
					if (maxAbsAngVelX < FastMath.abs(initialConditionAngularVelocity[0])) {
						maxAbsAngVelX = FastMath.abs(initialConditionAngularVelocity[0]);
					}
					if (maxAbsAngVelY < FastMath.abs(initialConditionAngularVelocity[1])) {
						maxAbsAngVelY = FastMath.abs(initialConditionAngularVelocity[1]);
					}
					if (maxAbsAngVelZ < FastMath.abs(initialConditionAngularVelocity[2])) {
						maxAbsAngVelZ = FastMath.abs(initialConditionAngularVelocity[2]);
					}
				}
				
				logger.info("DOMAIN OF ATTRACTION - {} - Controller {} - Samples: {}, Ran: {}, Saved (not ran): {}, Max Euler Angles (XYZ): {},{},{}, Max Angular Velocities (XYZ): {},{},{} ",
						label,
						controller, 
						simulations.get(controller).size(),
						ran,
						notRan,
						maxAbsEulerAngleX,
						maxAbsEulerAngleY,
						maxAbsEulerAngleZ,
						maxAbsAngVelX,
						maxAbsAngVelY,
						maxAbsAngVelZ);
				
				// trying to figure out the shape of the domain of attraction
				if (label != null && label.startsWith("CONVERGED")) {
					Map<Double, Double> data = new TreeMap<Double, Double>();
					for (double i : domainOfAttractionNorm.get(controller).keySet()) {
						final double x = domainOfAttractionNorm.get(controller).get(i)[0];
						final double y = domainOfAttractionNorm.get(controller).get(i)[1];
						// checking presence of same data
						final Double xPresent = data.get(x);
						if (xPresent == null || y > xPresent.doubleValue()) {
							data.put(x, y);
						} else {
							logger.info("Found similar data x{} y{}!", x, y);
						}
					}
					// filtering results
					if (data.size() > 0) {
						Map<Double, Double> filteredData = new TreeMap<Double, Double>();
						computePolygon(controller, data, filteredData);
						double area = computeArea(controller, filteredData);
						
						final DecimalFormat df = new DecimalFormat(FORMAT);
						
						domainOfAttractionNormShape.put(controller, filteredData);
						domainOfAttractionNormShapeForGraph.put(controller + " (area= "+df.format(area)+", samples= "+(filteredData.size()-2)+")", filteredData);
					} else {
						logger.info("No data available for computing the shape of {}!", controller);
					}
				}

			}
			
			// plotting
			Plotter.plot2DScatter(domainOfAttractionNorm, "Results - Domain of Attraction - Norm - " + label,
					new String[] {"norm of Euler angles", "norm of angular velocity"});
			if (plotAttitudeAndAngularVelocity) {
				Plotter.plot3DScatterStateSpace(domainOfAttractionAttitude, "Results - Domain of Attraction - Attitude " + label);
				Plotter.plot3DScatterStateSpace(domainOfAttractionAngularVelocity, "Results - Domain of Attraction - AngularVelocity " + label);
			}
			Plotter.plot2DLine(domainOfAttractionNormShapeForGraph, "Results - Domain of Attraction - Norm - Shape - " + label,
					true, new String[] {"norm of Euler angles", "norm of angular velocity"});
		}
		
		logger.info("Domain of Attraction ({}) plotted {}!", label, hasSomeDomainOfAttraction);
		logger.info("----------------------------");
		
		return domainOfAttractionNormShape;
	}

	protected void computePolygon(final String controller, final Map<Double, Double> data, final Map<Double, Double> filteredData) {
		final Double[] x = data.keySet().toArray(new Double[data.keySet().size()]);
		//final int numberOfIntervals = x.length > 100 ? 100 : x.length * 2;
		final double start = x[0];
		final double end = x[x.length-1];
		//final double lengthIntervalToAggregate = (end - start) / ((double)numberOfIntervals);
		
		// first point "FAKE" reusing y and x=0
		filteredData.put(0d, data.get(start));
		
		//int j = 0;
		for (int k = 0; k < x.length; ) {
			//final double startInterval = start + lengthIntervalToAggregate * j;
			//final double endInterval = startInterval + (lengthIntervalToAggregate * (j+1));
			final double endInterval = x[k];
			final double initialX = x[k];
			double currentX = initialX;
			double maxY = 0d;
			while (currentX <= endInterval) {
				if (data.get(currentX) > maxY) {
					maxY = data.get(currentX);
				}
				k++;
				if (k > x.length - 1) break;
				currentX = x[k];
			}
			//j++;
			if (maxY > 0 ) filteredData.put(x[k-1], maxY);
		}

		// last point "FAKE" reusing x and y=0
		filteredData.put(end+1, 0d); //+1E-10
	}

	protected double computeArea(final String controller, final Map<Double, Double> filteredData) {
		double area = 0d;
		try {
			// trying to calculate the area of the polygon
			// in the reverse order due to the convention of PolygonsSet
			// "The interior part of the region is on the left side of this path and the exterior is on its right side.
			Vector2D[] vec = new Vector2D[filteredData.size()+1];
			int count = filteredData.size();
			for (double xx : filteredData.keySet()) {
				vec[count--] = new Vector2D(xx, filteredData.get(xx));	
			}
			vec[0] = new Vector2D(0d, 0d);
			logger.info("Controller {}  polygon = {} ", controller, vec);
			PolygonsSet polygon = new PolygonsSet(1E-10d, vec); //1E-5d
			area = polygon.getSize();
			logger.info("Area controller {} = {} ", controller, area);
		} catch (Exception e) {
			// numerical problems is possible
			logger.error("Numerical problems calculating area of the polygon for controller " + controller, e);
		}
		return area;
	}

	//****
	/**
	 * For computing initial conditions for simulations.
	 * 
	 * @param numberOfSimulations
	 * @param approach for the definition of initial conditions (0 - NORNAL, 1 - UNIFORM, 2 - STATE SPACE EXPLORATION)
	 */
	protected void computeInitialConditions(int numberOfSimulations, int approach) throws OrekitException {
		
		logger.info("----------------------------");
		logger.info("Computing initial conditions for {} number of simulations and approach {} (0 - NORNAL, 1 - UNIFORM, 2 - STATE SPACE EXPLORATION)...", numberOfSimulations, approach);

		final Map<String, Map<Double, double[]>> initialAnglesForVisualization = new TreeMap<String, Map<Double, double[]>>();
		final Map<String, Map<Double, double[]>> initialNorm = new TreeMap<String, Map<Double, double[]>>();

		initialAngles.put("initialAngles", new TreeMap<Double, double[]>());
		initialAngularVelocities.put("initialAngularVelocities", new TreeMap<Double, double[]>());
		initialAnglesForVisualization.put("initialAnglesForVisualization", new TreeMap<Double, double[]>());
		initialNorm.put("initialNorm", new TreeMap<Double, double[]>());

		// for checking external boundaries of attractor
		final SimulationController ss = new SimulationController("NopeController", new double[] {0,0,0}, new double[] {0,0,0});
		final RealVector max = ss.satellite.getMaximumAngularVelocityControllableByReactionWheels();
		final double maxControllableAngularVelocity = max.getLInfNorm(); //max.getMinValue()
		logger.info("Maximum Angular Velocity Controllable by reaction wheels: {} NORM: {}. For simulation: {}!", max, max.getNorm(), maxControllableAngularVelocity);

		//*********************************
		// INITIAL CONDITIONS 
		// MONTE CARLO PARAMETERS
		final double MEAN_ANGLE = 0d;
		// SPACEOPS/ICEDYN
		// private static final double BASE_ANGULAR_VELOCITY = .11d; // IACLAW // 0.01d SPACEOPS
		final double MEAN_ANGULAR_VELOCITY = 0d; // .087d; // SPIE

		// standard deviation (3 sigmas)
		final double STD_ANGLE = 180d/3d;
		final double STD_ANGULAR_VELOCITY = maxControllableAngularVelocity/3d;

		// MONTE CARLO PARAMETERS - UNIFORM
		// CHANGED STRATEGY FOR MONTE CARLO FROM NORMAL TO UNIFORM
		// IAALACW - 2020 - CUBESAT
		//private static final double LOWER_ANGLE = -180d;
		//private static final double UPPER_ANGLE = 180d;
		final double LOWER_ANGLE = -180d; //-1E-9d; // -180d; 
		final double UPPER_ANGLE = 180d; //1E-9d; //180d; 
		// IAALACW - 2020 - CUBESAT
		// private static final double LOWER_ANGULAR_VELOCITY = -0.15d;
		// private static final double UPPER_ANGULAR_VELOCITY = 0.15d;
		// CILAMCE - 2020 - AMAZONIA1
		//private static final double LOWER_ANGULAR_VELOCITY = -0.02d;
		//private static final double UPPER_ANGULAR_VELOCITY = 0.02d;

		final double LOWER_ANGULAR_VELOCITY = -1 * maxControllableAngularVelocity;//-15E-3d; //GIBBS and GIBBS H-Infinity require the min angular velocity as 1E-4d while others not: LQR AND MRP
		final double UPPER_ANGULAR_VELOCITY = maxControllableAngularVelocity;//15E-3d; //1E-1d // too big
		
		if (approach == 0 ) {
			// NORMAL
			logger.info("NORMAL - Euler Angles X: N({},{}) Y: N({},{}) Z: N({},{})", MEAN_ANGLE, STD_ANGLE, MEAN_ANGLE, STD_ANGLE/3d, MEAN_ANGLE, STD_ANGLE/3d);
			logger.info("NORMAL - Angular Velocities X: N({},{}) Y: N({},{}) Z: N({},{})", MEAN_ANGULAR_VELOCITY, STD_ANGULAR_VELOCITY, MEAN_ANGULAR_VELOCITY, STD_ANGULAR_VELOCITY, MEAN_ANGULAR_VELOCITY, STD_ANGULAR_VELOCITY);
		} else {
			// UNIFORM
			logger.info("UNIFORM - Euler Angles X: U({},{}) Y: U({},{}) Z: U({},{})", LOWER_ANGLE, UPPER_ANGLE, LOWER_ANGLE/2d, UPPER_ANGLE/2d, LOWER_ANGLE, UPPER_ANGLE);
			logger.info("UNIFORM - Angular Velocities X: U({},{}) Y: U({},{}) Z: U({},{})", LOWER_ANGULAR_VELOCITY, UPPER_ANGULAR_VELOCITY, LOWER_ANGULAR_VELOCITY, UPPER_ANGULAR_VELOCITY, LOWER_ANGULAR_VELOCITY, UPPER_ANGULAR_VELOCITY);
		}

		// max value for the norm
		final RealVector maxNormEulerAngles = new ArrayRealVector(new Double[] {UPPER_ANGLE, UPPER_ANGLE/2, UPPER_ANGLE});
		final RealVector maxNormAngularVelocity = new ArrayRealVector(new Double[] {UPPER_ANGULAR_VELOCITY, UPPER_ANGULAR_VELOCITY, UPPER_ANGULAR_VELOCITY});
		logger.info("MAX NORM: Euler Angles: {} Angular Velocities: {}", maxNormEulerAngles.getNorm(), maxNormAngularVelocity.getNorm());
		
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
					initialAttitudeEulerAngles[1] = gaussianAnglesY.nextNormal(MEAN_ANGLE, STD_ANGLE/3d); // intermediary axis has a "smaller range" pi/2
					initialAttitudeEulerAngles[2] = gaussianAnglesZ.nextNormal(MEAN_ANGLE, STD_ANGLE);
							
					initialAngularVelocity[0] = gaussianVelocityX.nextNormal(MEAN_ANGULAR_VELOCITY, STD_ANGULAR_VELOCITY);
					initialAngularVelocity[1] = gaussianVelocityY.nextNormal(MEAN_ANGULAR_VELOCITY, STD_ANGULAR_VELOCITY);
					initialAngularVelocity[2] = gaussianVelocityZ.nextNormal(MEAN_ANGULAR_VELOCITY, STD_ANGULAR_VELOCITY);
					
				} else {
					// UNIFORM
					initialAttitudeEulerAngles[0] = gaussianAnglesX.nextUniform(LOWER_ANGLE, UPPER_ANGLE);
					initialAttitudeEulerAngles[1] = gaussianAnglesY.nextUniform(LOWER_ANGLE/2d, UPPER_ANGLE/2d); // intermediary axis has a "smaller range" pi/2
					initialAttitudeEulerAngles[2] = gaussianAnglesZ.nextUniform(LOWER_ANGLE, UPPER_ANGLE);
							
					initialAngularVelocity[0] = gaussianVelocityX.nextUniform(LOWER_ANGULAR_VELOCITY, UPPER_ANGULAR_VELOCITY);
					initialAngularVelocity[1] = gaussianVelocityY.nextUniform(LOWER_ANGULAR_VELOCITY, UPPER_ANGULAR_VELOCITY);
					initialAngularVelocity[2] = gaussianVelocityZ.nextUniform(LOWER_ANGULAR_VELOCITY, UPPER_ANGULAR_VELOCITY);

				}
	
				// CHECKING BOUNDARIES for angular velocity
				// EXTERNAL BOUNDARIES OF THE DOMAIN OF ATTRACTION - AMAZONIA-1: 0.03854253829546482 0.03325811543822333 0.022574544749001883
				final RealVector angVelocity = new ArrayRealVector(initialAngularVelocity);
				if (angVelocity.getEntry(0) > max.getEntry(0) ||
					angVelocity.getEntry(1) > max.getEntry(1) ||
					angVelocity.getEntry(2) > max.getEntry(2)) {
					logger.debug("Monte Carlo iteration - Angular Velocity: {} {} {} OUT OF THE EXTERNAL BOUNDARIES OF THE DOMAIN OF ATTRACTION: {} {} {}", 
							initialAngularVelocity[0],
							initialAngularVelocity[1], 
							initialAngularVelocity[2],
							max.getEntry(0),
							max.getEntry(1),
							max.getEntry(2));
				}  
				try {	
					// INITIAL CONDITIONS
					// showing initial Euler angles as rotations of the unit vector
					final Rotation rot = new Rotation(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR, 
							FastMath.toRadians(initialAttitudeEulerAngles[0]), 
							FastMath.toRadians(initialAttitudeEulerAngles[1]), 
							FastMath.toRadians(initialAttitudeEulerAngles[2]));
					Vector3D init3d = rot.applyTo(new Vector3D(1,1,1));
					initialAnglesForVisualization.get("initialAnglesForVisualization").put((double)i, init3d.toArray());
					// showing norm of initial conditions
					final RealVector initialConditionEulerAnglesV = new ArrayRealVector(new double[] {
							FastMath.toDegrees(rot.getAngles(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR)[0] ),
							FastMath.toDegrees(rot.getAngles(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR)[1] ), 
							FastMath.toDegrees(rot.getAngles(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR)[2] )
					});
					final RealVector initialConditionAngularVelocityV = new ArrayRealVector(initialAngularVelocity);
					// angles
					initialAngles.get("initialAngles").put((double)i, initialConditionEulerAnglesV.toArray());
					// norm
					initialNorm.get("initialNorm").put((double)i, new double[] { initialConditionEulerAnglesV.getNorm(),
																				 initialConditionAngularVelocityV.getNorm()});
					// angular velocities
					initialAngularVelocities.get("initialAngularVelocities").put((double)i, initialAngularVelocity.clone());

					logger.info("Monte Carlo iteration - {} - Euler Angles (X,Y,Z): {} {} {} NORM: {} \t- Angular Velocity (X,Y,Z): {} {} {} NORM: {}", String.format("%4d", i), 
							String.format("%+3.3f", initialConditionEulerAnglesV.toArray()[0]),
							String.format("%+3.3f", initialConditionEulerAnglesV.toArray()[1]), 
							String.format("%+3.3f", initialConditionEulerAnglesV.toArray()[2]), 
							String.format("%+3.3f", initialConditionEulerAnglesV.getNorm()),
							String.format("%+.3f", initialAngularVelocity[0]),
							String.format("%+.3f", initialAngularVelocity[1]),
							String.format("%+.3f", initialAngularVelocity[2]),
							String.format("%+3.3f", initialConditionAngularVelocityV.getNorm())
							);
				} catch(MathIllegalStateException me) {
					logger.debug("Cardans angles singularity", me);
				}
	
			}
		} else {
			// EXPLORE STATE SPACE
			// 2^6 = 64
			// 3^6 = 729
			// 4^6 = 4096
			// 5^6 = 15625
			// 6^6 = 46656
			// 7ˆ6 = 117649
			final long t = FastMath.round(FastMath.pow(numberOfSimulations, 1d/6d)); // 6 fors - 3 angles and 3 angular velocities
			if (t <= 1l) {
				throw new RuntimeException("In the state space exploration, the number of simulations is just a hint for the determination of such final number (power(numberOfSimulation,1/6)), which must be greater than 2 but it is " + t + "!");
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
									if (angVelocity.getEntry(0) > max.getEntry(0) ||
										angVelocity.getEntry(1) > max.getEntry(1) ||
										angVelocity.getEntry(2) > max.getEntry(2)) {
										logger.debug("State Space Exploration - Angular Velocity: {} {} {} OUT OF THE EXTERNAL BOUNDARIES OF THE DOMAIN OF ATTRACTION: {} {} {}", 
												initialAngularVelocity[0],
												initialAngularVelocity[1], 
												initialAngularVelocity[2],
												max.getEntry(0),
												max.getEntry(1),
												max.getEntry(2));
									} 
									try {	
										// INITIAL CONDITIONS
										// showing initial Euler angles as rotations of the unit vector
										final Rotation rot = new Rotation(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR, 
												FastMath.toRadians(initialAttitudeEulerAngles[0]), 
												FastMath.toRadians(initialAttitudeEulerAngles[1]), 
												FastMath.toRadians(initialAttitudeEulerAngles[2]));
										Vector3D init3d = rot.applyTo(new Vector3D(1,1,1));
										initialAnglesForVisualization.get("initialAnglesForVisualization").put((double)i, init3d.toArray());
										// showing norm of initial conditions
										final RealVector initialConditionEulerAnglesV = new ArrayRealVector(new double[] {
												FastMath.toDegrees(rot.getAngles(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR)[0] ),
												FastMath.toDegrees(rot.getAngles(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR)[1] ), 
												FastMath.toDegrees(rot.getAngles(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR)[2] )
										});
										final RealVector initialConditionAngularVelocityV = new ArrayRealVector(initialAngularVelocity);
										// angles
										initialAngles.get("initialAngles").put((double)i, initialConditionEulerAnglesV.toArray());
										// norm
										initialNorm.get("initialNorm").put((double)i, new double[] { initialConditionEulerAnglesV.getNorm(),
																									 initialConditionAngularVelocityV.getNorm()});
										// angular velocities
										initialAngularVelocities.get("initialAngularVelocities").put((double)i, initialAngularVelocity.clone());

										logger.info("State Space Exploration - {} - Euler Angles (X,Y,Z): {} {} {} NORM: {} \t- Angular Velocity (X,Y,Z): {} {} {} NORM: {}", String.format("%4d", i), 
												String.format("%+3.3f", initialConditionEulerAnglesV.toArray()[0]),
												String.format("%+3.3f", initialConditionEulerAnglesV.toArray()[1]), 
												String.format("%+3.3f", initialConditionEulerAnglesV.toArray()[2]), 
												String.format("%+3.3f", initialConditionEulerAnglesV.getNorm()),
												String.format("%+.3f", initialAngularVelocity[0]),
												String.format("%+.3f", initialAngularVelocity[1]),
												String.format("%+.3f", initialAngularVelocity[2]),
												String.format("%+3.3f", initialConditionAngularVelocityV.getNorm())
												);
									} catch(MathIllegalStateException me) {
										logger.debug("Cardans angles singularity", me);
									}
								}
							}
						}
					}
				}
			}
		}
		
		Plotter.plot3DScatterStateSpace(initialAngles, "Initial Conditions - Euler Angles (n = " + initialAngles.get("initialAngles").size() + ")");
		Plotter.plot3DScatterStateSpace(initialAnglesForVisualization, "Initial Conditions - Euler Angles (n = " + initialAnglesForVisualization.get("initialAnglesForVisualization").size() + ") for the unit vector");
		Plotter.plot3DScatterStateSpace(initialAngularVelocities, "Initial Conditions - Angular Velocities (n = " + initialAngularVelocities.get("initialAngularVelocities").size() + ")");
		Plotter.plot2DScatter(initialNorm, "Initial Conditions - Norm (n = " + initialNorm.get("initialNorm").size() + ")",
				new String[] {"norm of Euler angles", "norm of angular velocity"});

		logger.info("{} initial conditions computed!", initialAngles.get("initialAngles").size());
		logger.info("----------------------------");
				
	}

	//**********
	
	// to check convergence and to clean the list of simulations
	class SimulationControllerRunnable implements Runnable {
		private final SimulationController storedSimulationController;
		private final Map<String, List<SimulationController>> storedMapSimulations;
		private final Map<String, List<SimulationController>> storedMapSimulationsNotConverged;
		
		SimulationControllerRunnable(final SimulationController s,
				final Map<String, List<SimulationController>> mapSimulationsP,
				final Map<String, List<SimulationController>> mapSimulationsNotConvergedP){
			this.storedSimulationController = s;
			this.storedMapSimulations = mapSimulationsP;
			this.storedMapSimulationsNotConverged = mapSimulationsNotConvergedP;
		}
		
		@Override
		public void run() {
			// running
			storedSimulationController.run();
			
			// checking convergence
			final boolean convergence = storedSimulationController.checkConvergence();
			//final String controllerName =  storedSimulationController.satellite.getReactionWheelControllerName();
			final Satellite satelliteRan = storedSimulationController.satellite;
			if (!convergence) {
				// adding to not converged
				List<SimulationController> lnc = storedMapSimulationsNotConverged.get(satelliteRan.getReactionWheelControllerName());
				if (lnc == null) {
					lnc = new ArrayList<SimulationController>();
					storedMapSimulationsNotConverged.put(satelliteRan.getReactionWheelControllerName(), lnc);
				}
				lnc.add(storedSimulationController);
				
				// searching in the next simulations - IN THE QUEUE
				final double normEulerAngles = storedSimulationController.getEulerAnglesOfInitialCondition().getNorm();
				final double normAngVelocity = storedSimulationController.getAngularVelocityOfInitialCondition().getNorm();
				for (Runnable toRun : executorPool.getQueue()) {
					SimulationController toRunS = ((SimulationControllerRunnable) toRun).storedSimulationController;
					//final String controllerNameToRun = toRunS.satellite.getReactionWheelControllerName();
					final Satellite satelliteToRun = toRunS.satellite;
					final double normEulerAnglesToRun = toRunS.getEulerAnglesOfInitialCondition().getNorm();
					final double normAngVelocityToRun = toRunS.getAngularVelocityOfInitialCondition().getNorm();
					if (satelliteToRun.equalsStructurallly(satelliteRan) &&
							normEulerAnglesToRun > normEulerAngles 
							&& normAngVelocityToRun > normAngVelocity) {
						logger.info("Found a simulation that should NOT RUN: {} Norm Euler Angles {} Norm Angular Velocity {} - REFERENCE Norm Euler Angles {} Norm Angular Velocity {}",
								satelliteToRun, normEulerAnglesToRun, normAngVelocityToRun, normEulerAngles, normAngVelocity);
						// remove from EXECUTOR
						logger.info("Removing from executor, success: {}!", executorPool.remove(toRun));
						// adding to not converged
						lnc.add(toRunS);
					}
				}
				
			} else {
				// adding to converged
				List<SimulationController> l = storedMapSimulations.get(satelliteRan.getReactionWheelControllerName());
				if (l == null) {
					l = new ArrayList<SimulationController>();
					storedMapSimulations.put(satelliteRan.getReactionWheelControllerName(), l);
				}
				l.add(storedSimulationController);
			}
		}
		
	};

}