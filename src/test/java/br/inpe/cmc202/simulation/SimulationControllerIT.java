package br.inpe.cmc202.simulation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.hipparchus.util.FastMath;
import org.junit.BeforeClass;
import org.junit.FixMethodOrder;
import org.junit.Test;
import org.junit.runners.MethodSorters;
import org.orekit.errors.OrekitException;

/**
 * Test all the controllers available for reaction wheels.
 * 
 * @author alessandro.g.romero
 * 
 */
@FixMethodOrder(MethodSorters.NAME_ASCENDING)
public class SimulationControllerIT {

	private static final int INTERVAL_TO_STORE = 10000;
	private static final double STEP = .001;
	private static final int TIME_HINF = 1000;
	private static final int TIME = 1000;
	
	private static final double QUATERNION_MAX_ERROR = 0.015;
	private static final double VELOCITY_MAX_ERROR = 0.0008;

	@BeforeClass
	public static void beforeClass() {
		System.setProperty("orekit.data.path",
				"./src/main/resources/orekit-data");
	}

	@Test
	public void testProportionalDerivativeLinearSunVectorControllerModeTrue()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalDerivativeLinearSunVectorController", false, 0d,
				0d);
		simCtrl.satellite.setModeUsingAttitude(true);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalDerivativeLinearSunVectorControllerModeFalse()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalDerivativeLinearSunVectorController", false, 0d,
				0d);
		simCtrl.satellite.setModeUsingAttitude(false);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalLinearEulerAnglesLQRControllerModeTrue()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalLinearEulerAnglesLQRController", false, 0d, 0d);
		simCtrl.satellite.setModeUsingAttitude(true);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalLinearEulerAnglesLQRControllerModeFalse()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalLinearEulerAnglesLQRController", false, 0d, 0d);
		simCtrl.satellite.setModeUsingAttitude(false);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test(expected = RuntimeException.class)
	public void testProportionalLinearQuaternionFullLQRController_NOT_completely_controllable()
			throws OrekitException {
		new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalLinearQuaternionFullLQRController", false, 0d, 0d);
	}

	@Test
	public void testProportionalLinearQuaternionPartialLQRController()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalLinearQuaternionPartialLQRController", false, 0d,
				0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearEulerAnglesSDREController()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalNonLinearEulerAnglesSDREController", false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test(expected = RuntimeException.class)
	public void testProportionalNonLinearQuaternionFullSDREController_OMEGA_NOT_completely_controllable()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalNonLinearQuaternionSDREController_OMEGA", false,
				0d, 0d);
		simCtrl.run();
	}

	@Test(expected = RuntimeException.class)
	public void testProportionalNonLinearQuaternionFullSDREController_XI_NOT_completely_controllable()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalNonLinearQuaternionSDREController_XI", false, 0d,
				0d);
		simCtrl.run();
	}

	@Test
	public void testProportionalNonLinearQuaternionFullSDREController_GIBBS()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalNonLinearQuaternionSDREController_GIBBS", false,
				0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearQuaternionFullSDREController_GIBBS_SECOND()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalNonLinearQuaternionSDREController_GIBBS_SECOND",
				false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearQuaternionFullSDREController_GIBBS_ALPHA_0()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalNonLinearQuaternionSDREController_ALPHA",
				false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearQuaternionFullSDREController_GIBBS_ALPHA_1()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalNonLinearQuaternionSDREController_ALPHA",
				false, 1d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearQuaternionFullSDREController_GIBBS_ALPHA_OneHalf()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalNonLinearQuaternionSDREController_ALPHA",
				false, .5d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearMRPSDREController_COMPOSED()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalNonLinearMRPSDREController_FIRST", false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearMRPSDREController_PURE_OMEGA()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalNonLinearMRPSDREController_SECOND", false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearMRPSDREController_ALPHA_0()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalNonLinearMRPSDREController_ALPHA", false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearMRPSDREController_ALPHA_1()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalNonLinearMRPSDREController_ALPHA", false, 1d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearMRPSDREController_ALPHA_OneHalf()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME, STEP, INTERVAL_TO_STORE,
				"ProportionalNonLinearMRPSDREController_ALPHA", false, .5d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearMRPSDREHInfinityController()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME_HINF, STEP, INTERVAL_TO_STORE,
				"ProportionalNonLinearMRPSDREHInfinityController", false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearQuaternionFullSDREHInfinityController()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(TIME_HINF, STEP, INTERVAL_TO_STORE,
				"ProportionalNonLinearQuaternionFullSDREHInfinityController", false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	/**
	 * Check convergence for a given controller
	 * 
	 * @param simCtrl
	 */
	private void assertConvergence(SimulationController simCtrl) {
		// checking basic normal behavior
		assertNotNull(simCtrl.stepHandler);

		assertConvergenceVelocity(simCtrl);

		// checking quaternion error - lasttime
		double[] quaternionError = simCtrl.stepHandler.quaternionError
				.get(simCtrl.stepHandler.lastStoredTime);
		assertNotNull(quaternionError);
		assertEquals(0, quaternionError[0], QUATERNION_MAX_ERROR);
		assertEquals(0, quaternionError[1], QUATERNION_MAX_ERROR);
		assertEquals(0, quaternionError[2], QUATERNION_MAX_ERROR);
		assertEquals(1, FastMath.abs(quaternionError[3]), QUATERNION_MAX_ERROR); // quaternion can "wind"
	}

	/**
	 * Check convergence for a given controller
	 * 
	 * @param simCtrl
	 */
	private void assertConvergenceVelocity(SimulationController simCtrl) {
		// checking basic normal behavior
		assertNotNull(simCtrl.stepHandler);

		// checking velocity - lasttime
		double[] velocity = simCtrl.stepHandler.angularVelocityBody
				.get(simCtrl.stepHandler.lastStoredTime);
		assertNotNull(velocity);
		assertEquals(0, velocity[0], VELOCITY_MAX_ERROR);
		assertEquals(0, velocity[1], VELOCITY_MAX_ERROR);
		assertEquals(0, velocity[2], VELOCITY_MAX_ERROR);
	}
}
