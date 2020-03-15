package br.inpe.cmc202.satellite.sensors;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.Transform;
import org.orekit.models.earth.GeoMagneticElements;
import org.orekit.models.earth.GeoMagneticField;
import org.orekit.models.earth.GeoMagneticFieldFactory;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.TimeStampedPVCoordinates;

/**
 * 
 * This class represent the magnetometer in the satellite.
 * 
 * @author alessandro.g.romero
 * 
 */
public class Magnetometer {

	final private Frame ecef;

	/**
	 * Constructor with reference to ECEF.
	 * 
	 * @param ecef
	 */
	public Magnetometer(Frame ecef) {
		this.ecef = ecef;
	}

	/**
	 * Read method.
	 * 
	 * @param currentState
	 * @return
	 * @throws OrekitException
	 */
	public Vector3D read(SpacecraftState currentState) throws OrekitException {
		// current position and angular cordinates
		TimeStampedPVCoordinates pvCoord = currentState.getPVCoordinates();

		// transform from eci to body
		Transform c_body_eci = currentState.toTransform();
		// transform from eci to ecef
		Transform c_ecef_eci = currentState.getOrbit().getFrame()
				.getTransformTo(ecef, currentState.getDate());
		// transform from ecef to eci
		Transform c_eci_ecef = c_ecef_eci.getInverse();

		// getting ECEF coordinates
		Vector3D position_ecef = c_ecef_eci.transformVector(pvCoord
				.getPosition());

		// getting LLA coordinates
		Vector3D position_lla = transformECEFToLLA(position_ecef);

		// get magnetic field in ned
		Vector3D magneticFieldVector_ned = getMagneticFieldVector_ned(
				currentState, position_lla);

		// transform to ecef
		// THERE WAS A PROBLEM HERE v0.0.1 - PARAMETERS EXCHANGED position_lla,
		// magneticFieldVector_ned,
		Vector3D magneticFieldVector_ecef = transformNEDtoECEF(
				magneticFieldVector_ned, position_lla);
		// transform to eci
		Vector3D magneticFieldVector_eci = c_eci_ecef
				.transformVector(magneticFieldVector_ecef);
		// transform to body
		Vector3D magneticFieldVector_body = c_body_eci
				.transformVector(magneticFieldVector_eci);

		// checking the norm of the vector, trying to detect computation
		// problems during the transformations
		double err = 0.0001d;
		if (((magneticFieldVector_ned.getNorm() - err) > magneticFieldVector_ecef
				.getNorm() || (magneticFieldVector_ned.getNorm() + err) < magneticFieldVector_ecef
				.getNorm())
				|| ((magneticFieldVector_ned.getNorm() - err) > magneticFieldVector_eci
						.getNorm() || (magneticFieldVector_ned.getNorm() + err) < magneticFieldVector_eci
						.getNorm())
				|| ((magneticFieldVector_ned.getNorm() - err) > magneticFieldVector_body
						.getNorm() || (magneticFieldVector_ned.getNorm() + err) < magneticFieldVector_body
						.getNorm())) {
			throw new RuntimeException(
					"Error rotating the magnetic vector! Compare the norms: ned "
							+ magneticFieldVector_ned.getNorm() + " ecef "
							+ magneticFieldVector_ecef.getNorm() + " eci "
							+ magneticFieldVector_eci.getNorm() + " body "
							+ magneticFieldVector_body.getNorm());

		}
		return magneticFieldVector_body;
	}

	/**
	 * 
	 * https://microem.ru/files/2012/08/GPS.G1-X-00006.pdf
	 * 
	 * 2.2 ECEF to LLA
	 * 
	 * WGS84 Parameters
	 * 
	 * @param ecefPosition
	 * @return
	 */
	public Vector3D transformECEFToLLA(Vector3D ecefPosition) {
		// auxiliary parametrers - constant in the simulation
		final double a = Constants.WGS84_EARTH_EQUATORIAL_RADIUS;
		final double f = Constants.WGS84_EARTH_FLATTENING;
		final double b = a * (1 - f);
		final double firstEccentricity = FastMath
				.sqrt((FastMath.pow(a, 2) - FastMath.pow(b, 2))
						/ FastMath.pow(a, 2));
		final double secondEccentricity = FastMath
				.sqrt((FastMath.pow(a, 2) - FastMath.pow(b, 2))
						/ FastMath.pow(b, 2));

		// auxiliary parameters - dependent of position
		final double p = FastMath.sqrt(FastMath.pow(ecefPosition.getX(), 2)
				+ FastMath.pow(ecefPosition.getY(), 2));
		final double theta = FastMath.atan((ecefPosition.getZ() * a) / (p * b));

		// latitude (radians), longitude(radians) and altitude (meters)
		// varphi - latitude ranges from -pi/2 to pi/2
		final double varphi = FastMath.atan((ecefPosition.getZ() + FastMath
				.pow(secondEccentricity, 2)
				* b
				* FastMath.pow(FastMath.sin(theta), 3))
				/ (p - FastMath.pow(firstEccentricity, 2) * a
						* FastMath.pow(FastMath.cos(theta), 3)));
		// lambda - longitude ranges from -pi to pi
		// so it is needed to use atan2 (y,x) instead of atan(y/x)
		final double lambda = FastMath.atan2(ecefPosition.getY(),
				ecefPosition.getX());
		// intermediary value need - dependent on varphi
		final double n = a
				/ (FastMath.sqrt(1 - FastMath.pow(firstEccentricity, 2)
						* FastMath.pow(FastMath.sin(varphi), 2)));
		// altitude
		final double h = (p / FastMath.cos(varphi)) - n;

		return new Vector3D(varphi, lambda, h);
	}

	/**
	 * @param currentState
	 * @param pvCoord
	 * @return
	 * @throws OrekitException
	 */
	protected Vector3D getMagneticFieldVector_ned(SpacecraftState currentState,
			Vector3D llaPosition) throws OrekitException {
		// GET GeoMagneticField
		double year = GeoMagneticField.getDecimalYear(currentState.getDate()
				.getComponents(TimeScalesFactory.getUTC()).getDate().getDay(),
				currentState.getDate()
						.getComponents(TimeScalesFactory.getUTC()).getDate()
						.getMonth(),
				currentState.getDate()
						.getComponents(TimeScalesFactory.getUTC()).getDate()
						.getYear());
		GeoMagneticField model = GeoMagneticFieldFactory.getIGRF(year);
		// lat(degrees), long (degrees), alt (km)
		GeoMagneticElements result = model.calculateField(
				FastMath.toDegrees(llaPosition.getX()),
				FastMath.toDegrees(llaPosition.getY()),
				llaPosition.getZ() / 1000);

		// magneticFieldVector in ned (measured in - nT - nanoTesla)
		return result.getFieldVector();
	}

	/**
	 * 
	 * https://microem.ru/files/2012/08/GPS.G1-X-00006.pdf
	 * 
	 * 2.4 Converting ECEF Velocities to Local Tangent Plane Velocities
	 * 
	 * https://www.mathworks.com/help/aeroblks/directioncosinematrixeceftoned.
	 * html
	 * 
	 * WGS84 Parameters
	 * 
	 * @param nedPosition
	 *            (radians)
	 * @return
	 */
	protected Vector3D transformNEDtoECEF(Vector3D vector_ned,
			Vector3D position_lla) {

		Rotation c_ned_ecef = new Rotation(new double[][] {
				{
						-FastMath.sin(position_lla.getX())
								* FastMath.cos(position_lla.getY()),
						-FastMath.sin(position_lla.getX())
								* FastMath.sin(position_lla.getY()),
						FastMath.cos(position_lla.getX()) },
				{ -FastMath.sin(position_lla.getY()),
						FastMath.cos(position_lla.getY()), 0 },
				{
						-FastMath.cos(position_lla.getX())
								* FastMath.cos(position_lla.getY()),
						-FastMath.cos(position_lla.getX())
								* FastMath.sin(position_lla.getY()),
						-FastMath.sin(position_lla.getX()) } }, 0.000001d);
		Rotation c_ecef_ned = c_ned_ecef.revert();

		// vector in ecef
		Vector3D vector_ecef = c_ecef_ned.applyTo(vector_ned);

		return vector_ecef;
	}
}
