package br.inpe.cmc202.simulation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;

import org.math.plot.utils.FastMath;
import org.orekit.errors.OrekitException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * 
 * Main class for the comparison of the controller for a given Monte Carlo
 * perturbation model, taking into account uncertainty
 * 
 * @author alessandro.g.romero
 * 
 */
public class MultiSimulationUncertaintyController extends MultiSimulationController {
	
	static final private Logger logger = LoggerFactory
			.getLogger(MultiSimulationUncertaintyController.class);
	
	private static final List<String> CONTROLLERS = new ArrayList<String>(
			Arrays.asList("ProportionalLinearQuaternionPartialLQRController",
					"ProportionalNonLinearQuaternionSDREController_GIBBS"));
					//"ProportionalNonLinearQuaternionFullSDREHInfinityController"));
	
	final double LOWER_DEVIATION = -3E-1d; 
	final double UPPER_DEVIATION = 3E-1d;  
	final int NUMBER_OF_DEVIATIONS = 2;  

	final private Map<Double, Map<String, List<SimulationController>>> mapSimulationsU = new HashMap<Double, Map<String, List<SimulationController>>>();
	final private Map<Double, Map<String, List<SimulationController>>> mapSimulationsNotConvergedU = new HashMap<Double, Map<String, List<SimulationController>>>();

	public MultiSimulationUncertaintyController(int numberOfSimulations, int approach, int perturbationType) throws OrekitException {
		super();
		logger.info("----------------------------");
		logger.info("Configuring multi simulation... number of simulations: {}", numberOfSimulations);

		computeInitialConditions(numberOfSimulations, approach);
		// getting the computed initial conditions
		final Map<Double, double[]> initialAnglesComputed = initialAngles.get("initialAngles");
		final Map<Double, double[]> initialAngularVelocitiesComputed = initialAngularVelocities.get("initialAngularVelocities");

		//computing perturbation
		double stepPerturbation = (FastMath.abs(LOWER_DEVIATION)+FastMath.abs(UPPER_DEVIATION)) / (double) NUMBER_OF_DEVIATIONS;
		logger.info("Configuring perturbation... lower: {} upper: {} step: {}", LOWER_DEVIATION, UPPER_DEVIATION, stepPerturbation);
		
		// PERTURBATION
		for(double p = LOWER_DEVIATION; p <= UPPER_DEVIATION; p+=stepPerturbation) {			
			final Map<String, List<SimulationController>> mapSimulationsUP = new HashMap<String, List<SimulationController>>();
			final Map<String, List<SimulationController>> mapSimulationsNotConvergedUP = new HashMap<String, List<SimulationController>>();
			
			mapSimulationsU.put(p, mapSimulationsUP);
			mapSimulationsNotConvergedU.put(p,  mapSimulationsNotConvergedUP);

			// INITIAL CONDITIONS
			for (int i = 1; i <= initialAnglesComputed.size(); i++) {
	
				final double[] initialAttitudeEulerAngles = initialAnglesComputed.get((double)i);
	
				final double[] initialAngularVelocity = initialAngularVelocitiesComputed.get((double)i);
	
				// CONTROLLERS
				for (final String controller : CONTROLLERS) {
					
					final SimulationController s = new SimulationController(controller, calculateInertiaTensor(p), initialAttitudeEulerAngles,
							initialAngularVelocity);
					
					final Runnable s2 = new SimulationControllerRunnable(s, mapSimulationsUP, mapSimulationsNotConvergedUP);
							
					listSimulations.add(s2);
				}
	
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
		
		for (Double p: mapSimulationsU.keySet()) {
			final Map<String, List<SimulationController>> mapSimulationsUP = mapSimulationsU.get(p);
			final Map<String, List<SimulationController>> mapSimulationsNotConvergedUP = mapSimulationsNotConvergedU.get(p);

			logger.info("Computing PERTURBATION {}, CONVERGED {} NOT CONVERGED {}...", p, mapSimulationsUP.size(), mapSimulationsNotConvergedUP.size());
			computeResults(mapSimulationsUP, mapSimulationsNotConvergedUP, false);
			logger.info("Computed PERTURBATION {}, CONVERGED {} NOT CONVERGED {}!", p, mapSimulationsUP.size(), mapSimulationsNotConvergedUP.size());
			
			plotDomainOfAttraction(mapSimulationsUP, "CONVERGED "+p, false);
			plotDomainOfAttraction(mapSimulationsNotConvergedUP, "NOT CONVERGED "+p, false);
			
			// to check
			plotSimulations(mapSimulationsNotConvergedUP);

		}
		
	}
	
	/**
	 * Calculate the inertiaTensor - the parametric uncertainty.
	 * 
	 * @param variation - percent
	 * @return
	 * @throws OrekitException 
	 */
	private Properties calculateInertiaTensor(double variation) throws OrekitException {
		// loading and copying satellite configuration
		final SimulationController ss = new SimulationController("NopeController", new double[] {0,0,0}, new double[] {0,0,0});
		final Properties inertiaTensor = new Properties();
		for (final String key : ss.satelliteConfiguration.stringPropertyNames()) {
			if (key.startsWith("inertiaMoment")) {
				inertiaTensor.put(key, ss.satelliteConfiguration.get(key));
			}
		}
		
		// it changes the diagonal elements
		final double v11 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.1.1"));
		final double v22 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.2.2"));
		final double v33 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.3.3"));
		inertiaTensor.put("inertiaMoment.1.1", Double.toString(v11 + v11 * variation));
		inertiaTensor.put("inertiaMoment.2.2", Double.toString(v22 + v22 * variation));
		inertiaTensor.put("inertiaMoment.3.3", Double.toString(v33 + v33 * variation));
		
		// it changes other elements
		double v12 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.1.2"));
		double v13 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.1.3"));
		double v23 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.2.3"));
		inertiaTensor.put("inertiaMoment.1.2", Double.toString(v12 + v12 * variation));
		inertiaTensor.put("inertiaMoment.1.3", Double.toString(v13 + v13 * variation));
		inertiaTensor.put("inertiaMoment.2.1", Double.toString(v12 + v12 * variation));
		inertiaTensor.put("inertiaMoment.2.3", Double.toString(v23 + v23 * variation));
		inertiaTensor.put("inertiaMoment.3.1", Double.toString(v13 + v13 * variation));
		inertiaTensor.put("inertiaMoment.3.2", Double.toString(v23 + v23 * variation));

		logger.info("Inertia Tensor: {} ", inertiaTensor);
		return inertiaTensor;
	}

	/**
	 * Entry point for the simulation.
	 * 
	 * @param args
	 * @throws OrekitException
	 */
	public static void main(String[] args) throws OrekitException {
		logger.info("**********************************");
		logger.info("Satellite Multi Simulation Uncertainty " + (MultiSimulationUncertaintyController.class
				.getPackage().getImplementationVersion() == null ? ""
						: MultiSimulationUncertaintyController.class.getPackage()
								.getImplementationVersion()));
		logger.info("**********************************");

		int numberOfSimulations = 0;
		int approach = 1;
		int perturbationType = 0;
		if (args.length > 0) {
			numberOfSimulations = Integer.parseInt(args[0]);
			if (args.length > 1) {
				approach = Integer.parseInt(args[1]);
				if (args.length > 2) {
					perturbationType = Integer.parseInt(args[2]);
				}
			}
		} else {
			throw new RuntimeException("It must be informed the number of trials!");
		}

		new MultiSimulationUncertaintyController(numberOfSimulations, approach, perturbationType).run();

	}
}