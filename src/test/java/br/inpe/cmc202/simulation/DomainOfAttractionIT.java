package br.inpe.cmc202.simulation;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

import org.hipparchus.linear.DecompositionSolver;
import org.hipparchus.linear.LUDecomposition;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.util.FastMath;
import org.junit.FixMethodOrder;
import org.junit.Test;
import org.junit.runners.MethodSorters;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.satellite.Satellite;
import br.inpe.cmc202.satellite.actuators.SetOfReactionWheels;

/**
 * To determine the EXTERNAL BOUNDARIES of the domain of attraction.
 * 
 * @author alessandro.g.romero
 * 
 */
@FixMethodOrder(MethodSorters.NAME_ASCENDING)
public class DomainOfAttractionIT {

	final private Logger logger = LoggerFactory.getLogger(DomainOfAttractionIT.class);

	private Properties loadSatelliteConfiguration(String satelliteName) {
		final Properties p = new Properties();
		try {
			InputStream in = ClassLoader.getSystemResourceAsStream(satelliteName + ".properties");
			p.load(in);
			in.close();
			return p;
		} catch (IOException io) {
			throw new RuntimeException("Problems loading amazonia1.properties");
		}
	}

	private RealMatrix maxActuatorAngularMomentum(String satelliteName) {
		final Satellite sat = new Satellite(null, "NopeController", null, loadSatelliteConfiguration(satelliteName),
				null);
		final SetOfReactionWheels set = sat.getSetOfReactionWheels();
		final RealMatrix inertia = set.getInertiaTensorContribution();
		final RealMatrix maxAngularVelocity = MatrixUtils
				.createRealMatrix(new double[][] { { set.getMAX_ANGULAR_VELOCITY() }, { set.getMAX_ANGULAR_VELOCITY() },
						{ set.getMAX_ANGULAR_VELOCITY() } });
		logger.info(" Maximum Actuator angular velocity degrees/s: X={} Y={} Z={}",
				FastMath.toDegrees(maxAngularVelocity.getEntry(0, 0)),
				FastMath.toDegrees(maxAngularVelocity.getEntry(1, 0)),
				FastMath.toDegrees(maxAngularVelocity.getEntry(2, 0)));
		final RealMatrix maxActuatorsAngularMomentum = inertia.multiply(maxAngularVelocity);
		logger.info(" Maximum Actuators angular momentum kg m^2 s^{−1}: {}",
				maxActuatorsAngularMomentum.toString());
		return maxActuatorsAngularMomentum;

	}

	private void maxSatelliteAngularVelocity(final String satelliteName) {
		final Satellite sat = new Satellite(null, "NopeController", null, loadSatelliteConfiguration(satelliteName),
				null);
		final RealMatrix inertia = sat.getI();

		logger.info(" Inertia Tensor Satellite: {}", inertia.toString());
		// calculating max angular velocity = Iw = L
		DecompositionSolver solver = new LUDecomposition(inertia).getSolver();
		final RealMatrix maxAngVelocity = solver.solve(maxActuatorAngularMomentum(satelliteName));

		logger.info(" Maximum Satellite angular velocity r/s: {}", maxAngVelocity.toString());

		logger.info(" NORM L2 - Maximum Satellite angular velocity r/s: {}", maxAngVelocity.getFrobeniusNorm());

		logger.info(" Maximum Actuator angular velocity degrees/s: X={} Y={} Z={}",
				FastMath.toDegrees(maxAngVelocity.getEntry(0, 0)), FastMath.toDegrees(maxAngVelocity.getEntry(1, 0)),
				FastMath.toDegrees(maxAngVelocity.getEntry(2, 0)));
	}

	@Test
	public void testMaxReactionWheelAngularMomentumAmazonia1() {
		logger.info(" AMAZONIA1");
		maxActuatorAngularMomentum("amazonia1");
	}

	@Test
	public void testMaxSatelliteAngularVelocityAmazonia1() {
		logger.info(" AMAZONIA1");
		maxSatelliteAngularVelocity("amazonia1");
	}

	@Test
	public void testMaxReactionWheelAngularMomentumConasat() {
		logger.info(" CONASAT");
		maxActuatorAngularMomentum("conasat");
	}

	@Test
	public void testMaxSatelliteAngularVelocityConasat() {
		logger.info(" CONASAT");
		maxSatelliteAngularVelocity("conasat");
	}
}
