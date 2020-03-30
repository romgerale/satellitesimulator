package br.inpe.cmc202.simulation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

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

	private static final double QUATERNION_MAX_ERROR = 0.015;
	private static final double VELOCITY_MAX_ERROR = 0.0008;

	@BeforeClass
	public static void beforeClass() {
		System.setProperty("orekit.data.path",
				"./src/main/resources/orekit-data");
	}

	@Test
	public void testProportionalDerivativeLinearSunVectorController()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalDerivativeLinearSunVectorController", false, 0d,
				0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalLinearEulerAnglesLQRController()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalLinearEulerAnglesLQRController", false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test(expected = RuntimeException.class)
	public void testProportionalLinearQuaternionFullLQRController_NOT_completely_controllable()
			throws OrekitException {
		new SimulationController(700, .01, 1000,
				"ProportionalLinearQuaternionFullLQRController", false, 0d, 0d);
	}

	@Test
	public void testProportionalLinearQuaternionPartialLQRController()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalLinearQuaternionPartialLQRController", false, 0d,
				0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearEulerAnglesSDREController()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalNonLinearEulerAnglesSDREController", false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test(expected = RuntimeException.class)
	public void testProportionalNonLinearQuaternionFullSDREController_OMEGA_NOT_completely_controllable()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalNonLinearQuaternionSDREController_OMEGA", false,
				0d, 0d);
		simCtrl.run();
	}

	@Test(expected = RuntimeException.class)
	public void testProportionalNonLinearQuaternionFullSDREController_XI_NOT_completely_controllable()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalNonLinearQuaternionSDREController_XI", false, 0d,
				0d);
		simCtrl.run();
	}

	@Test
	public void testProportionalNonLinearQuaternionFullSDREController_GIBBS()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalNonLinearQuaternionSDREController_GIBBS", false,
				0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearQuaternionFullSDREController_GIBBS_SECOND()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalNonLinearQuaternionSDREController_GIBBS_SECOND",
				false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearQuaternionFullSDREController_GIBBS_ALPHA_0()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalNonLinearQuaternionSDREController_ALPHA",
				false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearQuaternionFullSDREController_GIBBS_ALPHA_1()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalNonLinearQuaternionSDREController_ALPHA",
				false, 1d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearQuaternionFullSDREController_GIBBS_ALPHA_OneHalf()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalNonLinearQuaternionSDREController_ALPHA",
				false, .5d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearMRPSDREController_COMPOSED()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalNonLinearMRPSDREController_FIRST", false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearMRPSDREController_PURE_OMEGA()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalNonLinearMRPSDREController_SECOND", false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearMRPSDREController_ALPHA_0()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalNonLinearMRPSDREController_ALPHA", false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearMRPSDREController_ALPHA_1()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalNonLinearMRPSDREController_ALPHA", false, 1d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearMRPSDREController_ALPHA_OneHalf()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(700, .01, 1000,
				"ProportionalNonLinearMRPSDREController_ALPHA", false, .5d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearMRPSDREHInfinityController()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(1000, .01, 1000,
				"ProportionalNonLinearMRPSDREHInfinityController", false, 0d, 0d);
		simCtrl.run();
		assertConvergence(simCtrl);
	}

	@Test
	public void testProportionalNonLinearQuaternionFullSDREHInfinityController()
			throws OrekitException {
		SimulationController simCtrl = new SimulationController(1000, .01, 1000,
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
		assertEquals(1, quaternionError[3], QUATERNION_MAX_ERROR);
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
