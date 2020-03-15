/**
 * 
 */
package br.inpe.cmc202.orbit;

import static org.junit.Assert.assertEquals;

import org.hipparchus.util.FastMath;
import org.junit.Test;
import org.orekit.errors.OrekitException;
import org.orekit.frames.FramesFactory;
import org.orekit.time.AbsoluteDate;

import br.inpe.cmc202.orbit.KeplerianOrbitAroundEarth;

/**
 * 
 * Test against http://www.jgiesen.de/kepler/kepler.html
 * 
 * @author alessandro.g.romero
 * 
 */
public class KleperianOrbitAroundEarth {

	/**
	 * Test method for
	 * {@link br.inpe.cmc202.orbit.KeplerianOrbitAroundEarth#meanToEllipticEccentric(double)}
	 * .
	 */
	@Test
	public void testMeanToEllipticEccentric() throws OrekitException {
		KeplerianOrbitAroundEarth o = new KeplerianOrbitAroundEarth(
				7130.092d * 1000, 0.001111d, FastMath.toRadians(98.405d),
				FastMath.toRadians(98.405d), FastMath.toRadians(227.088d),
				FastMath.toRadians(305d), FramesFactory.getTEME(),
				(AbsoluteDate) null);
		assertEquals(5.322343560728217d,
				o.meanToEllipticEccentric(FastMath.toRadians(305d)), 0.00001d);

		o = new KeplerianOrbitAroundEarth(7130.092d * 1000, 0.5d,
				FastMath.toRadians(98.405d), FastMath.toRadians(98.405d),
				FastMath.toRadians(227.088d), FastMath.toRadians(27d),
				FramesFactory.getTEME(), (AbsoluteDate) null);
		assertEquals(0.84533591001795d,
				o.meanToEllipticEccentric(FastMath.toRadians(27d)), 0.001d);
	}

	@Test
	public void testEllipticEccentricToTrue() throws OrekitException {
		KeplerianOrbitAroundEarth o = new KeplerianOrbitAroundEarth(
				7130.092d * 1000, 0.5d, FastMath.toRadians(98.405d),
				FastMath.toRadians(98.405d), FastMath.toRadians(227.088d),
				FastMath.toRadians(27d), FramesFactory.getTEME(),
				(AbsoluteDate) null);
		assertEquals(1.3236527687622d,
				o.ellipticEccentricToTrue(0.84533591001795d), 0.001d);

		o = new KeplerianOrbitAroundEarth(7130.092d * 1000, 0.001111d,
				FastMath.toRadians(98.405d), FastMath.toRadians(98.405d),
				FastMath.toRadians(227.088d), FastMath.toRadians(305d),
				FramesFactory.getTEME(), (AbsoluteDate) null);
		assertEquals(5.3214326, o.ellipticEccentricToTrue(5.322343560728217d),
				0.00001d);

	}

}
