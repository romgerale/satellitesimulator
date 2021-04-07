package br.inpe.cmc202.simulation;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import org.hipparchus.util.FastMath;
import org.orekit.errors.OrekitException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.simulation.plotter.Plotter;

/**
 * 
 * Main class for the comparison of a given controller and initial condition
 * with different alphas defined for a given Monte Carlo perturbation model.
 * 
 * @author alessandro.g.romero
 * @deprecated
 */
public class MultiAlphaSimulationController extends MultiSimulationController {

	private static final String CONTROLLER1 = "ProportionalNonLinearMRPSDREController_ALPHA";
	//private static final String CONTROLLER1 = "ProportionalNonLinearQuaternionSDREController_ALPHA";

	static final private Logger logger = LoggerFactory
			.getLogger(MultiAlphaSimulationController.class);

	/**
	 * @param monteCarlo
	 * @throws OrekitException
	 */
	public MultiAlphaSimulationController(int numberOfSimulations)
			throws OrekitException {
		logger.info("Configuring multi simulation... NumberOfTrials: {}",
				numberOfSimulations);
		for (int i = 1; i <= numberOfSimulations; i++) {
			double alpha1 = FastMath.random();

			// LINEAR
			SimulationController s = new SimulationController(CONTROLLER1,
					alpha1);
			listSimulations.add(s);
			List<SimulationController> l = mapSimulations.get(CONTROLLER1);
			if (l == null) {
				l = new ArrayList<SimulationController>();
				mapSimulations.put(CONTROLLER1, l);
			}
			l.add(s);
		}
		{
			// LINEAR 0
			SimulationController s = new SimulationController(CONTROLLER1, 0d);
			listSimulations.add(s);
			List<SimulationController> l = mapSimulations.get(CONTROLLER1);
			if (l == null) {
				l = new ArrayList<SimulationController>();
				mapSimulations.put(CONTROLLER1, l);
			}
			l.add(s);
		}
		{
			// LINEAR 1
			SimulationController s = new SimulationController(CONTROLLER1, 1d);
			listSimulations.add(s);
			List<SimulationController> l = mapSimulations.get(CONTROLLER1);
			if (l == null) {
				l = new ArrayList<SimulationController>();
				mapSimulations.put(CONTROLLER1, l);
			}
			l.add(s);
		}
	}

	protected void plotSimulations() {
		// plotting
		Map<String, Map<Double, Double>> detControllability = new TreeMap<String, Map<Double, Double>>();
		NumberFormat numberFormatter = new DecimalFormat("#0.00");
		for (String key : mapSimulations.keySet()) {
			for (SimulationController s : mapSimulations.get(key)) {
				detControllability.put("alpha="
						+ numberFormatter.format(s.satellite.getAlpha1()),
						s.stepHandler.detControllability);
			}
		}
		Plotter.plot2DLine(detControllability, "det controllability");
	}

	/**
	 * Entry point for the simulation.
	 * 
	 * @param args
	 * @throws OrekitException
	 */
	public static void main(String[] args) throws OrekitException {
		logger.info("**********************************");
		logger.info("Satellite Alpha Multi Simulation "
				+ (MultiAlphaSimulationController.class.getPackage()
						.getImplementationVersion() == null ? ""
						: MultiAlphaSimulationController.class.getPackage()
								.getImplementationVersion()));
		logger.info("**********************************");

		if (args.length > 0) {
			Integer numberOfSimulations = Integer.parseInt(args[0]);
			new MultiAlphaSimulationController(numberOfSimulations).run();
		} else {
			throw new RuntimeException(
					"It should be informed the number of trials!");
		}

	}
}