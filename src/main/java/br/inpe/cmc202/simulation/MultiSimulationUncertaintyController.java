package br.inpe.cmc202.simulation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.TreeMap;

import org.hipparchus.util.Precision;
import org.math.plot.utils.FastMath;
import org.orekit.errors.OrekitException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.simulation.plotter.Plotter;

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
					"ProportionalNonLinearQuaternionSDREController_GIBBS",
					"ProportionalNonLinearQuaternionFullSDREHInfinityController"));
	
	// percent
	final double LOWER_DEVIATION_PARAMETRIC = -3E-1d; 
	final double UPPER_DEVIATION_PARAMETRIC = +3E-1d; 
	// it is based on a white noise, so there is no mean to use negative values for magnitude
	// percent of max torque of reaction wheel
	final double LOWER_DEVIATION_UNSTRUCTURED = 0d; 
	final double UPPER_DEVIATION_UNSTRUCTURED = +3E-1d;
			
	final int NUMBER_OF_DEVIATIONS = 3;  

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

		double LOWER_DEVIATION = 0d;
		double UPPER_DEVIATION = 0d;
		if (perturbationType == 0) {
			LOWER_DEVIATION = LOWER_DEVIATION_PARAMETRIC;
			UPPER_DEVIATION = UPPER_DEVIATION_PARAMETRIC;
		} else {
			final SimulationController ss = new SimulationController("NopeController", new double[] {0,0,0}, new double[] {0,0,0});
			final Double maxTorque = ss.satellite.getSetOfReactionWheels().getMAX_TORQ();
			LOWER_DEVIATION = maxTorque * LOWER_DEVIATION_UNSTRUCTURED;
			UPPER_DEVIATION = maxTorque * UPPER_DEVIATION_UNSTRUCTURED;
		}
		
		//computing perturbation
		double stepPerturbation = (FastMath.abs(LOWER_DEVIATION)+FastMath.abs(UPPER_DEVIATION)) / ((double) NUMBER_OF_DEVIATIONS - 1d);
		logger.info("Configuring perturbation (TYPE={}, 0 = Parametric(InertiaTensor), 1 = Unstructured(externalTorques))... lower: {} upper: {} step: {}", perturbationType, LOWER_DEVIATION, UPPER_DEVIATION, stepPerturbation);
		for(double p = LOWER_DEVIATION; p < UPPER_DEVIATION || Precision.equals(p, UPPER_DEVIATION, UPPER_DEVIATION/NUMBER_OF_DEVIATIONS); p+=stepPerturbation) {			
			logger.info("Perturbation TYPE={} VALUE={}!", perturbationType, p);
		}
		
		// PERTURBATION
		for(double p = LOWER_DEVIATION; p < UPPER_DEVIATION || Precision.equals(p, UPPER_DEVIATION, UPPER_DEVIATION/NUMBER_OF_DEVIATIONS); p+=stepPerturbation) {			
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
					
					SimulationController s = null;
					if (perturbationType == 0) {
						s = new SimulationController(controller, calculateInertiaTensor(p), initialAttitudeEulerAngles,
							initialAngularVelocity);
					} else {
						s = new SimulationController(controller, p, initialAttitudeEulerAngles,
								initialAngularVelocity);
					}
					
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
		
		Map<String, Map<Double, double[][]>> mmap = new TreeMap<String, Map<Double, double[][]>>();
				
		for (Double p: mapSimulationsU.keySet()) {
			final Map<String, List<SimulationController>> mapSimulationsUP = mapSimulationsU.get(p);
			final Map<String, List<SimulationController>> mapSimulationsNotConvergedUP = mapSimulationsNotConvergedU.get(p);

			logger.info("Computing PERTURBATION {}, CONVERGED {} NOT CONVERGED {}...", p, mapSimulationsUP.size(), mapSimulationsNotConvergedUP.size());
			computeResults(mapSimulationsUP, mapSimulationsNotConvergedUP, p, true);
			logger.info("Computed PERTURBATION {}, CONVERGED {} NOT CONVERGED {}!", p, mapSimulationsUP.size(), mapSimulationsNotConvergedUP.size());
			
			Map<String, Map<Double, Double>> domainShape = plotDomainOfAttraction(mapSimulationsUP, "CONVERGED "+p, false);
			consolidateDomainOfAttraction(mmap, p, domainShape);
			
			//plotDomainOfAttraction(mapSimulationsNotConvergedUP, "NOT CONVERGED "+p, false);
			
			// to check
			//plotSimulations(mapSimulationsNotConvergedUP);

		}
		
		if (mmap.size() > 0) {
			// plotting each domain of attraction separated
			for (String controller: mmap.keySet()) {
				Map<Double, double[][]> map = mmap.get(controller);
				Map<String, Map<Double, double[][]>> mmapP = new TreeMap<String, Map<Double, double[][]>>();
				mmapP.put(controller, map);
				
				Plotter.plot3DLinesDomainOfAttraction(mmapP, "Domain Of Attraction - Uncertainty - " + controller, false);
			}
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
	 * Consolidate the domain of attraction for all data computed.
	 * 
	 * @param mmap - map with all data for the domain of attraction
	 * @param p - perturbation value
	 * @param domainShape - shape of the domain of Attraction for a given perturbation
	 */
	private void consolidateDomainOfAttraction(Map<String, Map<Double, double[][]>> mmap, Double p,
			Map<String, Map<Double, Double>> domainShape) {
		// consolidating results
		if (domainShape.size() > 0) {
			// for each controller
			for (String controller : domainShape.keySet()) {
				// getting data
				Map<Double, Double> data = domainShape.get(controller);
				// getting controller in the consolidated data
				Map<Double, double[][]> map = mmap.get(controller);
				if (map == null) {
					map = new TreeMap<Double, double[][]>();
					mmap.put(controller, map);
				}
				// formatting data for the polygon
				double[][] xyz = new double[data.size()+2][3] ;
				int count = 0;
				for(double x: data.keySet()) {
					xyz[count][0] = x;
					xyz[count][1] = data.get(x);
					xyz[count++][2] = p;
				}
				// origin
				xyz[count][0] = 0d;
				xyz[count][1] = 0d;
				xyz[count++][2] = p;
				// closing the polygon
				xyz[count][0] = xyz[0][0];
				xyz[count][1] = xyz[0][1];
				xyz[count][2] = xyz[0][2];
				map.put(p, xyz);
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