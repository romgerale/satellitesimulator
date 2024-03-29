package br.inpe.cmc202.simulation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.TreeMap;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.RotationOrder;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.RealVector;
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
 * AMAZONIA1
 * 
 * 
 * @author alessandro.g.romero
 * @deprecated see MultiSimulationUncertaintyController
 */
public class MultiSimulationParametricUncertaintyPlusController extends MultiSimulationController {

	// CILAMCE 2020
	// "ProportionalNonLinearQuaternionSDREController_GIBBS",
	private static final List<String> CONTROLLERS = new ArrayList<String>(
			Arrays.asList(//"ProportionalLinearQuaternionPartialLQRController",
					"ProportionalNonLinearQuaternionSDREController_GIBBS",
					"ProportionalNonLinearQuaternionFullSDREHInfinityController"));
					//"ProportionalNonLinearMRPSDREController_FIRST",
					//"ProportionalNonLinearMRPSDREHInfinityController",
					//"NopeController"));
	
	static final private Logger logger = LoggerFactory
			.getLogger(MultiSimulationParametricUncertaintyPlusController.class);

	// MONTE CARLO PARAMETERS - GAUSSIAN
	// standard deviation for INERTIA TENSOR - 3 standard deviation equals 5%
	private static final double STD = 0.016666d;
	// range for uniform distribution
	private static final double RANGE = 0.2d;

	// UNIFORM 
	// CILAMCE - 2020
	private static final double LOWER_ANGLE = -180d;
	private static final double UPPER_ANGLE = 180d;
	private static final double LOWER_ANGULAR_VELOCITY = -0.01d;
	private static final double UPPER_ANGULAR_VELOCITY = 0.01d;
	
	//private static final double LOWER_ANGULAR_VELOCITY = -1E-4d; //GIBBS and GIBBS H-Infinity require the min angular velocity as 1E-4d while others not: LQR AND MRP
	//private static final double UPPER_ANGULAR_VELOCITY = 1E-4d;

	
	/**
	 * @param monteCarlo
	 * @throws OrekitException
	 */
	public MultiSimulationParametricUncertaintyPlusController(int numberOfSimulations) throws OrekitException {
		logger.info("Configuring multi simulation... NumberOfTrials: {}", numberOfSimulations);

		initialAngles.put("initialAngles", new TreeMap<Double, double[]>());
		initialAngularVelocities.put("initialAngularVelocities", new TreeMap<Double, double[]>());

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
			 calculateInertiaTensorUsingUniform(inertiaTensorRandom);

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
				// INITIAL CONDITIONS
				// showing initial Euler angles as rotations of the unit vector
				final Rotation rot = new Rotation(RotationOrder.ZYX, RotationConvention.FRAME_TRANSFORM, initialAttitudeEulerAngles[0], 
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

	/**
	 * @param inertiaTensorRandom
	 * @return
	 */
	private Properties calculateInertiaTensorUsingNormal(RandomDataGenerator inertiaTensorRandom) {
		/*
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
		*/
		// loading and copying satellite configuration
		final Properties inertiaTensor = new Properties();
		try { 
			final SimulationController ss = new SimulationController("NopeController", new double[] {0,0,0}, new double[] {0,0,0});
			for (final String key : ss.satelliteConfiguration.stringPropertyNames()) {
				if (key.startsWith("inertiaMoment")) {
					inertiaTensor.put(key, ss.satelliteConfiguration.get(key));
				}
			}
		} catch (OrekitException ex) {
			throw new RuntimeException(ex);
		}
		
		// it changes the diagonal elements
		final double v11 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.1.1"));
		final double v22 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.2.2"));
		final double v33 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.3.3"));
		inertiaTensor.put("inertiaMoment.1.1", Double.toString(inertiaTensorRandom.nextNormal(v11, v11 * STD)));
		inertiaTensor.put("inertiaMoment.2.2", Double.toString(inertiaTensorRandom.nextNormal(v22, v22 * STD)));
		inertiaTensor.put("inertiaMoment.3.3", Double.toString(inertiaTensorRandom.nextNormal(v33, v33 * STD)));
		
		// it changes other elements
		double v12 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.1.2"));
		double v13 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.1.3"));
		double v23 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.2.3"));
		inertiaTensor.put("inertiaMoment.1.2", Double.toString(v12));
		inertiaTensor.put("inertiaMoment.1.3", Double.toString(v13));
		inertiaTensor.put("inertiaMoment.2.1", Double.toString(v12));
		inertiaTensor.put("inertiaMoment.2.3", Double.toString(v23));
		inertiaTensor.put("inertiaMoment.3.1", Double.toString(v13));
		inertiaTensor.put("inertiaMoment.3.2", Double.toString(v23));

		logger.info("Inertia Tensor: {} ", inertiaTensor);
		return inertiaTensor;
	}

	/**
	 * @param inertiaTensorRandom
	 * @return
	 */
	private Properties calculateInertiaTensorUsingUniform(RandomDataGenerator inertiaTensorRandom) {
		// loading and copying satellite configuration
		final Properties inertiaTensor = new Properties();
		try { 
			final SimulationController ss = new SimulationController("NopeController", new double[] {0,0,0}, new double[] {0,0,0});
			for (final String key : ss.satelliteConfiguration.stringPropertyNames()) {
				if (key.startsWith("inertiaMoment")) {
					inertiaTensor.put(key, ss.satelliteConfiguration.get(key));
				}
			}
		} catch (OrekitException ex) {
			throw new RuntimeException(ex);
		}
		
		// it changes the diagonal elements
		final double v11 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.1.1"));
		final double v22 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.2.2"));
		final double v33 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.3.3"));
		inertiaTensor.put("inertiaMoment.1.1", Double.toString(inertiaTensorRandom.nextUniform(v11 - (v11 * RANGE), v11 + (v11 * RANGE))));
		inertiaTensor.put("inertiaMoment.2.2", Double.toString(inertiaTensorRandom.nextUniform(v22 - (v22 * RANGE), v22 + (v22 * RANGE))));
		inertiaTensor.put("inertiaMoment.3.3", Double.toString(inertiaTensorRandom.nextUniform(v33 - (v33 * RANGE), v33 + (v33 * RANGE))));
		
		// it changes other elements
		double v12 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.1.2"));
		double v13 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.1.3"));
		double v23 = Double.valueOf(inertiaTensor.getProperty("inertiaMoment.2.3"));
		inertiaTensor.put("inertiaMoment.1.2", Double.toString(v12));
		inertiaTensor.put("inertiaMoment.1.3", Double.toString(v13));
		inertiaTensor.put("inertiaMoment.2.1", Double.toString(v12));
		inertiaTensor.put("inertiaMoment.2.3", Double.toString(v23));
		inertiaTensor.put("inertiaMoment.3.1", Double.toString(v13));
		inertiaTensor.put("inertiaMoment.3.2", Double.toString(v23));

		logger.info("Inertia Tensor: {} ", inertiaTensor);
		return inertiaTensor;
	}
	
	/*protected void plotSimulations(Map<String, List<SimulationController>> mapSimulations) {
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
	}*/

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
			Integer numberOfSimulations = Integer.parseInt(args[0]);
			new MultiSimulationParametricUncertaintyPlusController(numberOfSimulations).run();
		} else {
			throw new RuntimeException("It should be informed the number of trials!");
		}

	}
}