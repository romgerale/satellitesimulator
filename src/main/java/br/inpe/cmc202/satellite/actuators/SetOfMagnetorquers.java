package br.inpe.cmc202.satellite.actuators;

import java.util.Properties;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;

/**
 * Set of three Magnetorquers installed in the satellite in each axis.
 * 
 * (Marcel J. Sichi, p 161) Magnetic techniques provide continuous and smooth
 * control. Moreover, the level of torques that can be achieved with
 * magnetorquer is normally low (in the range of 1-10mN.m 0r 0.001-0.01Nm)
 * 
 * (Marcel J. Sichi, p 185) Interaction between a magnetic moment generated
 * within a spacecraft M and the earth's magnetic field (B) produces a
 * mechanicall torque acting on the spacecraft: T = M x B
 * 
 * (Marcel J. Sichi, p 191) The basic control equation for momentum unloading is
 * T= -k(h-h_n) = -k\Deltah
 * 
 * @author alessandro.g.romero
 * 
 */
public class SetOfMagnetorquers {

	/**
	 * Maximum magnetic dipole.
	 * 
	 * A.m^2
	 */
	private final double MAX_MAGNETIC_DIPOLE;

	/**
	 * Mean magnetic field.
	 * 
	 * nT (nano Tesla)
	 * 
	 * Only applicable for the specific orbit
	 */
	private final double MEAN_MAGNETIC_FIELD;

	/**
	 * @param satelliteConfiguration
	 */
	public SetOfMagnetorquers(Properties satelliteConfiguration) {
		if (satelliteConfiguration != null) {
			this.MAX_MAGNETIC_DIPOLE = Double.valueOf(satelliteConfiguration
					.getProperty("magnetorque.maxDipole", "30d"));
			this.MEAN_MAGNETIC_FIELD = Double.valueOf(satelliteConfiguration
					.getProperty("magnetorque.meanMagneticField", "0.948d"));
		} else {
			this.MAX_MAGNETIC_DIPOLE = 30d;
			this.MEAN_MAGNETIC_FIELD = 0.948d;
		}
	}

	/**
	 * State
	 */
	private SetOfMagnetorquersState state = new SetOfMagnetorquersState(
			Vector3D.ZERO, Vector3D.ZERO);

	/**
	 * @param magneticField
	 *            nT
	 * @param angular
	 *            momentum Kg.m^2.radians/s
	 */
	public SetOfMagnetorquersState actuate(final Vector3D magneticField_body,
			final Vector3D t_mtr) {

		// computing the magneticDipole with cross-product-control-law
		Vector3D magneticDipole_body_Am2 = (Vector3D.crossProduct(
				magneticField_body, t_mtr)).scalarMultiply(-1
				/ FastMath.pow(MEAN_MAGNETIC_FIELD, 2));

		// unit conversion
		// to Tesla
		final Vector3D magneticField_body_T = magneticField_body
				.scalarMultiply(1E-9);

		// check saturation in each magnetorquer
		if (FastMath.abs(magneticDipole_body_Am2.getX()) > MAX_MAGNETIC_DIPOLE) {
			magneticDipole_body_Am2 = new Vector3D(
					FastMath.signum(magneticDipole_body_Am2.getX())
							* MAX_MAGNETIC_DIPOLE,
					magneticDipole_body_Am2.getY(),
					magneticDipole_body_Am2.getZ());
		}
		if (FastMath.abs(magneticDipole_body_Am2.getY()) > MAX_MAGNETIC_DIPOLE) {
			magneticDipole_body_Am2 = new Vector3D(
					magneticDipole_body_Am2.getX(),
					FastMath.signum(magneticDipole_body_Am2.getY())
							* MAX_MAGNETIC_DIPOLE,
					magneticDipole_body_Am2.getZ());
		}
		if (FastMath.abs(magneticDipole_body_Am2.getZ()) > MAX_MAGNETIC_DIPOLE) {
			magneticDipole_body_Am2 = new Vector3D(
					magneticDipole_body_Am2.getX(),
					magneticDipole_body_Am2.getY(),
					FastMath.signum(magneticDipole_body_Am2.getZ())
							* MAX_MAGNETIC_DIPOLE);
		}

		// computing the torque
		final Vector3D torque_body = Vector3D.crossProduct(
				magneticDipole_body_Am2, magneticField_body_T);

		this.state = new SetOfMagnetorquersState(torque_body,
				magneticDipole_body_Am2);
		return this.state;
	}

	/**
	 * @return the state
	 */
	public SetOfMagnetorquersState getState() {
		return state;
	}

}