package br.inpe.cmc202.simulation;

import java.util.Properties;

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

	public MultiSimulationUncertaintyController(int numberOfSimulations, int approach) throws OrekitException {
		super(numberOfSimulations, approach);
	}

	@Override
	protected SimulationController createSimulationController(final double[] initialAttitudeEulerAngles,
			final double[] initialAngularVelocity, final String controller) throws OrekitException {
		return new SimulationController(controller, calculateInertiaTensor(2E-1d), initialAttitudeEulerAngles,
			initialAngularVelocity);
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
		if (args.length > 0) {
			numberOfSimulations = Integer.parseInt(args[0]);
			if (args.length > 1) {
				approach = Integer.parseInt(args[1]);
			}
		} else {
			throw new RuntimeException("It must be informed the number of trials!");
		}

		new MultiSimulationUncertaintyController(numberOfSimulations, approach).run();

	}
}