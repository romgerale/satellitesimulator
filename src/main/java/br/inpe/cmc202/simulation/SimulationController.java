package br.inpe.cmc202.simulation;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;
import java.util.Set;
import java.util.TreeSet;

import org.hipparchus.exception.DummyLocalizable;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.RotationOrder;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.RealVector;
import org.hipparchus.util.FastMath;
import org.orekit.attitudes.Attitude;
import org.orekit.errors.OrekitException;
import org.orekit.frames.FramesFactory;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.AngularCoordinates;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.frame.EarthCenteredEarthFixedFrame;
import br.inpe.cmc202.frame.EarthCenteredInertialFrame;
import br.inpe.cmc202.orbit.KeplerianOrbitAroundEarth;
import br.inpe.cmc202.propagation.ControllerAttitudeModifier;
import br.inpe.cmc202.propagation.KeplerianPropagator;
import br.inpe.cmc202.propagation.KinematicsAttitudeProvider;
import br.inpe.cmc202.propagation.KineticsAttitudeModifier;
import br.inpe.cmc202.propagation.stateprovider.MagnetometerStateProvider;
import br.inpe.cmc202.propagation.stateprovider.SatelliteUpdaterStateProvider;
import br.inpe.cmc202.propagation.stateprovider.SetOfSunSensorsStateProvider;
import br.inpe.cmc202.propagation.stateprovider.SunErrorStateProvider;
import br.inpe.cmc202.satellite.Satellite;

/**
 * 
 * Main class for an unique simulation.
 * 
 * @author alessandro.g.romero
 * 
 */
public class SimulationController implements Runnable {

	static final private Logger logger = LoggerFactory.getLogger(SimulationController.class);

	// frames
	EarthCenteredInertialFrame eci;
	EarthCenteredEarthFixedFrame ecef;

	// start date
	AbsoluteDate startTime;
	// orbit
	KeplerianOrbitAroundEarth orbit;

	// satellite
	Satellite satellite;

	// propagator
	KeplerianPropagator propagator;

	// stephandler
	StepHandler stepHandler;

	// SIMULATION PARAMETERS in seconds
	double step;
	double simulationTime;
	long intervalToStore;

	// INITIAL conditions
	double[] initialAttitude;
	double[] initialAngularVelocity;
	Attitude initialAttitudeS;

	// simulation configuration
	Properties simulationConfiguration = new Properties();
	Properties satelliteConfiguration = new Properties();

	/**
	 * Constructor based on configuration files.
	 * 
	 * @throws OrekitException
	 */
	public SimulationController() throws OrekitException {
		loadDefaultConfiguration();

		// setting initial attitude
		this.initialAttitude = new double[] {
				Double.valueOf(simulationConfiguration.getProperty("initial.eulerangle.x", "0")),
				Double.valueOf(simulationConfiguration.getProperty("initial.eulerangle.y", "0")),
				Double.valueOf(simulationConfiguration.getProperty("initial.eulerangle.z", "180")) };
		this.initialAngularVelocity = new double[] {
				Double.valueOf(simulationConfiguration.getProperty("initial.angularVelocity.x", "0")),
				Double.valueOf(simulationConfiguration.getProperty("initial.angularVelocity.y", "0")),
				Double.valueOf(simulationConfiguration.getProperty("initial.angularVelocity.z", "0.024d")) };

		// configuring
		configure(Double.valueOf(simulationConfiguration.getProperty("simulation.time", "700")),
				Double.valueOf(simulationConfiguration.getProperty("simulation.step", "0.01d")),
				Long.valueOf(simulationConfiguration.getProperty("simulation.intervalToStore", "1000")),
				simulationConfiguration.getProperty("simulation.reactionWheelControllerName"), true, 0d, 0d, null);
	}

	/**
	 * Constructor based on configuration files. It allows t ooverride the
	 * controller defined in the files as well as initial attitude and initial
	 * angular velocity.
	 * 
	 * @param reactionWheelControllerName
	 * @param initialAttitude
	 * @param initialAngularVelocity
	 * @throws OrekitException
	 */
	public SimulationController(String reactionWheelControllerName, double[] initialAttitude,
			double[] initialAngularVelocity) throws OrekitException {
		loadDefaultConfiguration();

		// using initial attitude and angular velocity
		this.initialAttitude = initialAttitude;
		this.initialAngularVelocity = initialAngularVelocity;

		// configuring
		configure(Double.valueOf(simulationConfiguration.getProperty("simulation.time", "700")),
				Double.valueOf(simulationConfiguration.getProperty("simulation.step", "0.01d")),
				Long.valueOf(simulationConfiguration.getProperty("simulation.intervalToStore", "1000")),
				reactionWheelControllerName, false, 0d, 0d, null);
	}

	/**
	 * Constructor based on configuration files. It allows to override the
	 * controller defined in the files as well as the alpha.
	 * 
	 * @param reactionWheelControllerName
	 * @param alpha
	 * @throws OrekitException
	 */
	public SimulationController(String reactionWheelControllerName, double alpha) throws OrekitException {
		loadDefaultConfiguration();

		// setting initial attitude
		this.initialAttitude = new double[] {
				Double.valueOf(simulationConfiguration.getProperty("initial.eulerangle.x", "0")),
				Double.valueOf(simulationConfiguration.getProperty("initial.eulerangle.y", "0")),
				Double.valueOf(simulationConfiguration.getProperty("initial.eulerangle.z", "180")) };
		this.initialAngularVelocity = new double[] {
				Double.valueOf(simulationConfiguration.getProperty("initial.angularVelocity.x", "0")),
				Double.valueOf(simulationConfiguration.getProperty("initial.angularVelocity.y", "0")),
				Double.valueOf(simulationConfiguration.getProperty("initial.angularVelocity.z", "0.024d")) };

		// configuring
		configure(Double.valueOf(simulationConfiguration.getProperty("simulation.time", "700")),
				Double.valueOf(simulationConfiguration.getProperty("simulation.step", "0.01d")),
				Long.valueOf(simulationConfiguration.getProperty("simulation.intervalToStore", "1000")),
				reactionWheelControllerName, false, alpha, 0d, null);
	}

	/**
	 * Constructor based on configuration files. It allows the overriding the
	 * controller defined in the files as well as the inertia tensor.
	 * 
	 * @param reactionWheelControllerName
	 * @param inertiaTensor
	 * @throws OrekitException
	 */
	public SimulationController(String reactionWheelControllerName, Properties inertiaTensor) throws OrekitException {
		loadDefaultConfiguration();

		// setting initial attitude
		this.initialAttitude = new double[] {
				Double.valueOf(simulationConfiguration.getProperty("initial.eulerangle.x", "0")),
				Double.valueOf(simulationConfiguration.getProperty("initial.eulerangle.y", "0")),
				Double.valueOf(simulationConfiguration.getProperty("initial.eulerangle.z", "180")) };
		this.initialAngularVelocity = new double[] {
				Double.valueOf(simulationConfiguration.getProperty("initial.angularVelocity.x", "0")),
				Double.valueOf(simulationConfiguration.getProperty("initial.angularVelocity.y", "0")),
				Double.valueOf(simulationConfiguration.getProperty("initial.angularVelocity.z", "0.024d")) };

		// configuring
		configure(Double.valueOf(simulationConfiguration.getProperty("simulation.time", "700")),
				Double.valueOf(simulationConfiguration.getProperty("simulation.step", "0.01d")),
				Long.valueOf(simulationConfiguration.getProperty("simulation.intervalToStore", "1000")),
				reactionWheelControllerName, false, 0d, 0d, inertiaTensor);
	}

	/**
	 * Constructor based on configuration files. It allows the overriding the
	 * controller defined in the files as well as the inertia tensor, the initial
	 * attitude and the initial angular velocity.
	 * 
	 * @param reactionWheelControllerName
	 * @param inertiaTensor
	 * @param initialAttitude
	 * @param initialAngularVelocity
	 * 
	 * @throws OrekitException
	 */
	public SimulationController(String reactionWheelControllerName, Properties inertiaTensor, double[] initialAttitude, double[] initialAngularVelocity) throws OrekitException {
		loadDefaultConfiguration();

		// using initial attitude and angular velocity
		this.initialAttitude = initialAttitude;
		this.initialAngularVelocity = initialAngularVelocity;

		// configuring
		configure(Double.valueOf(simulationConfiguration.getProperty("simulation.time", "700")),
				Double.valueOf(simulationConfiguration.getProperty("simulation.step", "0.01d")),
				Long.valueOf(simulationConfiguration.getProperty("simulation.intervalToStore", "1000")),
				reactionWheelControllerName, false, 0d, 0d, inertiaTensor);
	}

	/**
	 * Constructor based on configuration files. It allows the overriding the
	 * controller defined in the files as well as the inertia tensor, the initial
	 * attitude and the initial angular velocity.
	 * 
	 * @param reactionWheelControllerName
	 * @param externalTorqueMagnitude
	 * @param initialAttitude
	 * @param initialAngularVelocity
	 * 
	 * @throws OrekitException
	 */
	public SimulationController(String reactionWheelControllerName, double externalTorqueMagnitude, double[] initialAttitude, double[] initialAngularVelocity) throws OrekitException {
		loadDefaultConfiguration();

		// using initial attitude and angular velocity
		this.initialAttitude = initialAttitude;
		this.initialAngularVelocity = initialAngularVelocity;

		// configuring
		configure(Double.valueOf(simulationConfiguration.getProperty("simulation.time", "700")),
				Double.valueOf(simulationConfiguration.getProperty("simulation.step", "0.01d")),
				Long.valueOf(simulationConfiguration.getProperty("simulation.intervalToStore", "1000")),
				reactionWheelControllerName, false, 0d, externalTorqueMagnitude, null);
	}

	/**
	 * Constructor for tests.
	 * 
	 * @param simulationTime
	 * @param step
	 * @param intervalToStore
	 * @param reactionWheelControllerName
	 * @param plot
	 * @param alpha1
	 * @param externalTorque
	 * @throws OrekitException
	 */
	public SimulationController(double simulationTime, double step, long intervalToStore,
			String reactionWheelControllerName, boolean plot, double alpha1, double externalTorque)
			throws OrekitException {
		configure(simulationTime, step, intervalToStore, reactionWheelControllerName, plot, alpha1, externalTorque,
				null);
	}

	/**
	 * Constructor with initial conditions, used by MultiSimulation.
	 * 
	 * @param simulationTime
	 * @param step
	 * @param intervalToStore
	 * @param reactionWheelControllerName
	 * @param plot
	 * @param initialAttitude
	 * @param initialAngularVelocity
	 * @throws OrekitException
	 */
	public SimulationController(double simulationTime, double step, long intervalToStore,
			String reactionWheelControllerName, boolean plot, double[] initialAttitude, double[] initialAngularVelocity)
			throws OrekitException {
		this.initialAttitude = initialAttitude;
		this.initialAngularVelocity = initialAngularVelocity;
		configure(simulationTime, step, intervalToStore, reactionWheelControllerName, plot, 0d, 0d, null);
	}

	/**
	 * Load default configuration from simulation.properties and properties of the satellite.
	 * 
	 */
	private final void loadDefaultConfiguration() throws OrekitException {
		logger.info("Loading configuration...");
		try {
			logger.info("Reading \"simulationcontroller.properties\"...");
			InputStream in = ClassLoader.getSystemResourceAsStream("simulationcontroller.properties");
			if (in == null) {
				throw new OrekitException(
						new DummyLocalizable("Configuration file \"simulationcontroller.properties\" not found"));
			}
			simulationConfiguration.load(in);
			in.close();
			final String configName = simulationConfiguration.getProperty("satellite.properties");
			if (configName == null) {
				throw new OrekitException(
						new DummyLocalizable("Configuration \"satellite.properties\" is not defined"));
			}
			logger.info("Reading \"" + configName + ".properties\"...");
			in = ClassLoader.getSystemResourceAsStream(configName + ".properties");
			if (in == null) {
				throw new OrekitException(new DummyLocalizable("Configuration file \"" + configName + "\" not found"));
			}
			satelliteConfiguration.load(in);
			in.close();
		} catch (IOException io) {
			throw new OrekitException(new DummyLocalizable("problems loading simulationcontroller.properties"), io);
		}
		logger.info("Configuration loaded.");
	}

	/**
	 * Configure the simulation.
	 * 
	 * @param simulationTime
	 * @param step
	 * @param intervalToStore
	 * @param reactionWheelControllerName
	 * @param plot
	 * @param alpha1
	 * @param externalTorque
	 * @throws OrekitException
	 */
	private final void configure(double simulationTime, double step, long intervalToStore,
			String reactionWheelControllerName, boolean plot, double alpha1, double externalTorque,
			Properties inertiaTensor) throws OrekitException {
		logger.info("Configuring simulation...");
		// STEP
		this.step = step;
		this.simulationTime = simulationTime;
		this.intervalToStore = intervalToStore;
		// FRAMES
		// ----------------------
		this.eci = new EarthCenteredInertialFrame(FramesFactory.getTEME());
		this.ecef = new EarthCenteredEarthFixedFrame(eci);

		// DATE INITIAL CONDITION
		// ----------------------------------------
		if (simulationConfiguration.isEmpty()) {
			// 01/06/2017 �s 11:00 GMT.
			this.startTime = new AbsoluteDate(2017, 6, 1, 11, 0, 0, TimeScalesFactory.getUTC());

			// ORBIT PARAMETERS
			// ----------------------------------------
			// a - semi-major axis (m)
			// 7130.092 (km)
			// e - eccentricity
			// 0.001111
			// i - inclination (rad)
			// 98.405 (degrees)
			// pa - perigee argument
			// 98.405 (degrees)
			// raan - right ascension of ascending node
			// 227.088 (degrees)
			// anomaly - mean anomaly
			// 305 (degrees)
			this.orbit = new KeplerianOrbitAroundEarth(7130.092d * 1000, 0.001111d, FastMath.toRadians(98.405d),
					FastMath.toRadians(98.405d), FastMath.toRadians(227.088d), FastMath.toRadians(305d), eci,
					startTime);
		} else {
			this.startTime = new AbsoluteDate(
					Integer.valueOf(simulationConfiguration.getProperty("startTime.year", "2017")),
					Integer.valueOf(simulationConfiguration.getProperty("startTime.month", "6")),
					Integer.valueOf(simulationConfiguration.getProperty("startTime.day", "1")),
					Integer.valueOf(simulationConfiguration.getProperty("startTime.hour", "11")),
					Integer.valueOf(simulationConfiguration.getProperty("startTime.minute", "0")),
					Integer.valueOf(simulationConfiguration.getProperty("startTime.second", "0")),
					TimeScalesFactory.getUTC());

			// ORBIT PARAMETERS
			// ----------------------------------------
			this.orbit = new KeplerianOrbitAroundEarth(
					Double.valueOf(simulationConfiguration.getProperty("orbit.a", "7130.092d")) * 1000,
					Double.valueOf(simulationConfiguration.getProperty("orbit.e", "0.001111d")),
					FastMath.toRadians(Double.valueOf(simulationConfiguration.getProperty("orbit.i", "98.405d"))),
					FastMath.toRadians(Double.valueOf(simulationConfiguration.getProperty("orbit.pa", "98.405d"))),
					FastMath.toRadians(Double.valueOf(simulationConfiguration.getProperty("orbit.raan", "227.088d"))),
					FastMath.toRadians(Double.valueOf(simulationConfiguration.getProperty("orbit.mean", "305d"))), eci,
					startTime);
		}

		String magnetorquersController = null;
		if (!simulationConfiguration.isEmpty()) {
			magnetorquersController = simulationConfiguration.getProperty("simulation.magnetorquersController", null);
		}

		this.satellite = new Satellite(ecef, reactionWheelControllerName, magnetorquersController, alpha1,
				externalTorque, satelliteConfiguration, inertiaTensor);

		this.stepHandler = new StepHandler(ecef, startTime, satellite, simulationTime, step, intervalToStore, plot);

		// ATTITUDE - INITIAL CONDITION
		// ----------------------------------------------
		final Rotation c_body_eci = new Rotation(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR,
				FastMath.toRadians((initialAttitude != null && initialAttitude.length == 3) ? initialAttitude[0] : 0),
				FastMath.toRadians((initialAttitude != null && initialAttitude.length == 3) ? initialAttitude[1] : 0),
				FastMath.toRadians(
						(initialAttitude != null && initialAttitude.length == 3) ? initialAttitude[2] : 180));
		final Vector3D angularVelocity = new Vector3D(
				(initialAngularVelocity != null && initialAngularVelocity.length == 3) ? initialAngularVelocity[0] : 0,
				(initialAngularVelocity != null && initialAngularVelocity.length == 3) ? initialAngularVelocity[1] : 0,
				(initialAngularVelocity != null && initialAngularVelocity.length == 3) ? initialAngularVelocity[2]
						: 0.024);
		final Vector3D angularAcceleration = Vector3D.ZERO;
		final AngularCoordinates initialAngularCoordinates = new AngularCoordinates(c_body_eci, angularVelocity,
				angularAcceleration);
		initialAttitudeS = new Attitude(startTime, eci, initialAngularCoordinates);

		final Vector3D eulerAngles = new Vector3D(
				c_body_eci.getAngles(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR));

		// LOGGING CONFIGURATION
		logger.info("Orbit: {}", orbit);
		logger.info("MeanMotion: {} radians/s", orbit.getKeplerianMeanMotion());
		logger.info("Period: {} s", orbit.getKeplerianPeriod());
		logger.info("Simulation Time: {} s ({} % of the orbit period)", this.simulationTime,
				(this.simulationTime / orbit.getKeplerianPeriod()) * 100);
		logger.info("Start Time: {} ", startTime);
		logger.info("End Time: {} ", startTime.shiftedBy(this.simulationTime));
		logger.info("Fixed simulation step: {} s", this.step);
		logger.debug("Number of points to be stored: {}",
				(long) (this.simulationTime / this.step) / this.intervalToStore + 1);
		logger.info("Satellite Configuration: {}", satellite);
		logger.info("Initial Attitude - Euler Angles: X {} Y {} Z {} ", FastMath.toDegrees(eulerAngles.getX()),
				FastMath.toDegrees(eulerAngles.getY()), FastMath.toDegrees(eulerAngles.getZ()));
		logger.info("Initial Attitude - Quaternions: Q1 {} Q2 {} Q3 {} Q4 {} ", c_body_eci.getQ1(), c_body_eci.getQ2(),
				c_body_eci.getQ3(), c_body_eci.getQ0());
		logger.info("Initial Attitude - Velocity (rad/s): X {} Y {} Z {}", angularVelocity.getX(),
				angularVelocity.getY(), angularVelocity.getZ());
		logger.info("Initial Attitude - Acceleration (rad/s^2): X {} Y {} Z {}", angularAcceleration.getX(),
				angularAcceleration.getY(), angularAcceleration.getZ());

		logger.info("Simulation configured.");
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Runnable#run()
	 */
	public void run() {
		// propagators
		// propagate kinematics
		final KinematicsAttitudeProvider kinematicsAttitudeProvider = new KinematicsAttitudeProvider(initialAttitudeS,
				satellite);
		// propagate kinetics
		final KineticsAttitudeModifier kineticsAttitudeModifier = new KineticsAttitudeModifier(
				kinematicsAttitudeProvider, satellite);
		// perform control based on previous propagated kinematics and kinetics
		final ControllerAttitudeModifier controllerModifier = new ControllerAttitudeModifier(kineticsAttitudeModifier,
				satellite);

		// additional states
		final SetOfSunSensorsStateProvider sunSensorsAdditionalStateProvider = new SetOfSunSensorsStateProvider(
				satellite);
		final SunErrorStateProvider sunerrorAdditionalStateProvider = new SunErrorStateProvider(satellite);
		final MagnetometerStateProvider magnetometerAdditionalStateProvider = new MagnetometerStateProvider(satellite);
		final SatelliteUpdaterStateProvider satelliteUpdaterAdditionalStateProvider = new SatelliteUpdaterStateProvider(
				satellite);

		// orbit propagator
		try {
			propagator = new KeplerianPropagator(orbit, controllerModifier);

			// additional providers
			propagator.addAdditionalStateProvider(sunSensorsAdditionalStateProvider);
			propagator.addAdditionalStateProvider(sunerrorAdditionalStateProvider);
			propagator.addAdditionalStateProvider(magnetometerAdditionalStateProvider);
			propagator.addAdditionalStateProvider(satelliteUpdaterAdditionalStateProvider);

			propagator.setMasterMode(step, stepHandler);

			// propagating
			propagator.propagate(startTime.shiftedBy(simulationTime));
		} catch (OrekitException e) {
			throw new RuntimeException("Error in the running of a simulation", e);
		}

	}

	/**
	 * Check convergence in the final of the simulation.
	 */
	protected boolean checkConvergence() {
		boolean convergenceStateSpace = true;

		if (!ran()) {
			throw new RuntimeException("Check convergence must be only called at the final of the simulation!");
		}
		
		// getting last 5% of time TO TEST CONVERGENCE
		final double tToTestConvergence = this.stepHandler.lastStoredTime - (this.stepHandler.lastStoredTime * .05d);
		logger.debug("time to test convergence {}",tToTestConvergence);
		
		// filtering times to evaluate
		final Set<Double> timesToEvaluate = new TreeSet<Double>(this.stepHandler.quaternionError.keySet());
		timesToEvaluate.removeIf(v -> v < tToTestConvergence );

		final double epsilon = 1.E-5;
		for (final Double t : timesToEvaluate) {
			final double[] quaternionError = this.stepHandler.quaternionError.get(t);
			final double[] angularVelocityError = this.stepHandler.angularVelocityBody.get(t);
			// compute the norm of state space: 
			// quaternion (last entry as scalar and adjusted to origin) 
			// and angular velocity
			final RealVector stateSpace = new ArrayRealVector(new double[] { 
					quaternionError[0],
					quaternionError[1],
					quaternionError[2], 
					1-FastMath.abs(quaternionError[3]), // adjusting to origin 0
					angularVelocityError[0],
					angularVelocityError[1],
					angularVelocityError[2]});
			if (stateSpace.getNorm() > epsilon) {
				convergenceStateSpace = false;
			}
		}

		logger.debug("convergence {} epsilon {}", convergenceStateSpace, epsilon);

		return convergenceStateSpace;
	}
	
	/**
	 * Returns the initial angular velocities.
	 */
	protected RealVector getAngularVelocityOfInitialCondition() {
		final RealVector initialConditionAngularVelocity = new ArrayRealVector(this.initialAttitudeS.getSpin().toArray());
		return initialConditionAngularVelocity;
	}
	
	/**
	 * Returns the initial Euler angles.
	 */
	protected RealVector getEulerAnglesOfInitialCondition() {
		final double[] initialAnglesRadians = this.initialAttitudeS.getRotation().getAngles(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR);
		final double[] initialConditionEulerAngles = new double[] {
				FastMath.toDegrees(initialAnglesRadians[0]),
				FastMath.toDegrees(initialAnglesRadians[1]),
				FastMath.toDegrees(initialAnglesRadians[2])};
		final RealVector initialConditionEulerAnglesV = new ArrayRealVector(initialConditionEulerAngles);
		return initialConditionEulerAnglesV;
	}
	
	/**
	 * Returns whether the simulation ran.
	 */
	protected boolean ran() {
		return this.stepHandler.currentTime * this.step >= this.simulationTime ;
	}
	
	/**
	 * Entry point for the simulation.
	 * 
	 * @param args
	 * @throws OrekitException
	 */
	public static void main(String[] args) throws OrekitException {
		logger.info("**********************************");
		logger.info("Satellite Simulation "
				+ (SimulationController.class.getPackage().getImplementationVersion() == null ? ""
						: SimulationController.class.getPackage().getImplementationVersion()));
		logger.info("**********************************");

		// new SimulationController(130000, .1, 100).run();
		// new SimulationController(20000, .1, 100).run();
		// new SimulationController(1300, .1, 1000,
		// "ProportionalNonLinearQuaternionPartialSDREController", true).run();
		// new SimulationController(700, .1, 1000).run();
		// new SimulationController(20000, .01, 5000).run();
		// new SimulationController(7200, .01, 1000).run();
		// new SimulationController(1300, .01, 1000).run();
		// new SimulationController(700, .01, 1000,
		// "ProportionalLinearQuaternionPartialLQRController", true).run();
		// new SimulationController(700, .01, 1000,
		// "ProportionalNonLinearQuaternionSDREController_GIBBS", true)
		// .run();
		// new SimulationController(700, .01, 1000,
		// "ProportionalNonLinearMRPSDREController_SECOND", true).run();
		// new SimulationController(700, .01, 1000,
		// "ProportionalNonLinearMRPSDREController_FIRST", true).run();
		// new SimulationController(700, .01, 1000,
		// "ProportionalNonLinearAngularVelocitySDREController", true)
		// .run();

		// IFTOOM // 700, 0.001, 5000
		// new SimulationController(700, .001, 5000,
		// "ProportionalNonLinearEulerAnglesSDREController", true).run();
		// new SimulationController(700, .001, 5000,
		// "ProportionalDerivativeLinearSunVectorController", true).run();

		// SPACEOPS
		// new SimulationController(700, .01, 1000,
		// "ProportionalNonLinearQuaternionSDREController_GIBBS", true,
		// 0d, 0).run();

		new SimulationController().run();

		// new SimulationController(1000, .1, 1000,
		// "ProportionalNonLinearQuaternionSDREController_GIBBS_SECOND", true)
		// .run();

		// new SimulationController(700, .01, 1000,
		// "ProportionalNonLinearMRPSDREController_FIRST", true, 0d, 0)
		// .run();

		// new SimulationController(700, .01, 1000,
		// "ProportionalNonLinearMRPSDREController_SECOND", true, 0d, 0)
		// .run();

		// new SimulationController(700, .01, 1000,
		// "ProportionalNonLinearMRPSDREController_ALPHA", true, 0d, 0)
		// .run();

		// new SimulationController(700, .01, 1000,
		// "ProportionalNonLinearEulerAnglesSDREWithKalmanFilterController",
		// true,
		// 0d, 0).run();

		// new SimulationController(20000, .001, 5000).run();
		// new SimulationController(7200, .001, 5000).run();
		// new SimulationController(1300, .001, 5000).run();
		// new SimulationController(700, .001, 5000).run();
	}
}