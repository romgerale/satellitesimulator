package br.inpe.cmc202.satellite.sensors;

import static org.junit.Assert.assertEquals;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.junit.Before;
import org.junit.Test;
import org.orekit.errors.OrekitException;
import org.orekit.frames.FramesFactory;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;

import br.inpe.cmc202.frame.EarthCenteredInertialFrame;
import br.inpe.cmc202.orbit.KeplerianOrbitAroundEarth;

public class MagnetometerTest {

	private SpacecraftState state;

	@Before
	public void setup() throws OrekitException {
		EarthCenteredInertialFrame eci = new EarthCenteredInertialFrame(
				FramesFactory.getTEME());
		AbsoluteDate startTime = new AbsoluteDate(2017, 6, 1, 11, 0, 0,
				TimeScalesFactory.getUTC());
		KeplerianOrbitAroundEarth orbit = new KeplerianOrbitAroundEarth(
				7130.092d * 1000, 0.001111d, FastMath.toRadians(98.405d),
				FastMath.toRadians(98.405d), FastMath.toRadians(227.088d),
				FastMath.toRadians(305d), eci, startTime);
		state = new SpacecraftState(orbit);
	}

	// {0,7486573797; 2,8644804062; 757.283,8341245214}
	// degrees
	// 42,8949081582396; 164,122637773587; 757.283,8341245214
	// 42+53';164+7'
	// MagneticField[B={18.350,533; -110,295;
	// 27.164,614},H=18.350,865,F=32.782,168,I=55,959,D=-0,344]
	@Test
	public void testIGRF_getMagneticFieldVector_ned() throws OrekitException {
		Magnetometer m = new Magnetometer(null);
		Vector3D vectorField_ned = m.getMagneticFieldVector_ned(state,
				new Vector3D(0.7486573797, 2.8644804062, 757283.8341245214));

		assertEquals(32782, vectorField_ned.getNorm(), 0.9d);
	}

	// 42+53';164+7'; 757.283
	// Geomagnetic elements Lat.: 42.883N Long.: 164.117E Altitude: 757283.0m
	// Year:2017 (Prediction) (IGRF)
	// Total Intensity (F): 32767.0 nT Declination (D): -0.320o Inclination (I):
	// 55.935o
	// Northward (X): 18353.4 nT Eastward (Y): -102.5 nT Downward (Z): 27144.4
	// nT
	// Horizontal (H): 18353.7 nT
	//
	// source:http://wdc.kugi.kyoto-u.ac.jp/igrf/point/
	//
	// MagneticField[B={18.350,533; -110,295;
	// 27.164,614},H=18.350,865,F=32.782,168,I=55,959,D=-0,344]
	@Test
	public void testIGRF_getMagneticFieldVector_ned_online()
			throws OrekitException {
		Magnetometer m = new Magnetometer(null);
		Vector3D vectorField_ned = m.getMagneticFieldVector_ned(state,
				new Vector3D(0.7486573797, 2.8644804062, 757283.8341245214));

		assertEquals(32767, vectorField_ned.getNorm(), 20d);
	}
}
