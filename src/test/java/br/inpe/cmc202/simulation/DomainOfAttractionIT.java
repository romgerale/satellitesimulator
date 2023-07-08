package br.inpe.cmc202.simulation;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

import org.hipparchus.linear.RealVector;
import org.hipparchus.util.FastMath;
import org.junit.FixMethodOrder;
import org.junit.Test;
import org.junit.runners.MethodSorters;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.satellite.Satellite;

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
			throw new RuntimeException("Problems loading "+ satelliteName + ".properties");
		}
	}


	@Test
	public void testMaxSatelliteAngularVelocityAmazonia1() {
		logger.info(" AMAZONIA1");
		final Satellite sat = new Satellite(null, "NopeController", null, loadSatelliteConfiguration("amazonia1"),
				null);
		final RealVector maxAngVelocity = sat.getMaximumAngularVelocityControllableByReactionWheels();
		logger.info(" Maximum Satellite angular velocity r/s: {}", maxAngVelocity.toString());

		logger.info(" Maximum Satellite angular velocity degrees/s: X={} Y={} Z={}",
				FastMath.toDegrees(maxAngVelocity.getEntry(0)), FastMath.toDegrees(maxAngVelocity.getEntry(1)),
				FastMath.toDegrees(maxAngVelocity.getEntry(2)));

		logger.info(" NORM L2 - Maximum Satellite angular velocity r/s: {}", maxAngVelocity.getNorm());

	}


	@Test
	public void testMaxSatelliteAngularVelocityConasat() {
		logger.info(" CONASAT");
		final Satellite sat = new Satellite(null, "NopeController", null, loadSatelliteConfiguration("conasat"),
				null);
		
		if (sat.getSetOfReactionWheels() != null) {
			final RealVector maxAngVelocity = sat.getMaximumAngularVelocityControllableByReactionWheels();
			logger.info(" Maximum Satellite angular velocity r/s: {}", maxAngVelocity.toString());
	
			logger.info(" Maximum Satellite angular velocity degrees/s: X={} Y={} Z={}",
					FastMath.toDegrees(maxAngVelocity.getEntry(0)), FastMath.toDegrees(maxAngVelocity.getEntry(1)),
					FastMath.toDegrees(maxAngVelocity.getEntry(2)));
	
			logger.info(" NORM L2 - Maximum Satellite angular velocity r/s: {}", maxAngVelocity.getNorm());
		}
	}
}
