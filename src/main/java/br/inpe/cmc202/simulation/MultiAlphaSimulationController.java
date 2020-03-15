package br.inpe.cmc202.simulation;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

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
 * 
 */
public class MultiAlphaSimulationController implements Runnable {

	private static final String CONTROLLER1 = "ProportionalNonLinearMRPSDREController_ALPHA";
	//private static final String CONTROLLER1 = "ProportionalNonLinearQuaternionSDREController_ALPHA";

	static final private Logger logger = LoggerFactory
			.getLogger(MultiAlphaSimulationController.class);

	// FOR STORING
	final List<SimulationController> listSimulations = new ArrayList<SimulationController>();
	final Map<String, List<SimulationController>> mapSimulations = new HashMap<String, List<SimulationController>>();

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
		ThreadPoolExecutor executorPool = new ThreadPoolExecutor(3, 3, 0,
				TimeUnit.SECONDS, new LinkedBlockingQueue<Runnable>(),
				threadFactory);

		// starting threads
		for (SimulationController s : listSimulations) {
			executorPool.execute(s);
		}

		logger.info(
				"Total of {} simulations are scheduled to run in {} threads",
				listSimulations.size(), executorPool.getActiveCount());

		// joinning all threads
		executorPool.shutdown();
		while (!executorPool.isTerminated()) {
			try {
				Thread.sleep(10000);
				logger.info(
						"{} simulations concluded from the total of {} in {} s",
						executorPool.getCompletedTaskCount(),
						listSimulations.size(),
						(System.currentTimeMillis() - start) / 1000d);
			} catch (InterruptedException e) {
				throw new RuntimeException("Simulation was interrupted", e);
			}
		}

		logger.info("**********************************");
		logger.info("{} simulations concluded from the total of {} in {} s",
				executorPool.getCompletedTaskCount(), listSimulations.size(),
				(System.currentTimeMillis() - start) / 1000d);
		logger.info("**********************************");

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
			Integer numberOfSimulations = new Integer(args[0]);
			new MultiAlphaSimulationController(numberOfSimulations).run();
		} else {
			throw new RuntimeException(
					"It should be informed the number of trials!");
		}

	}
}