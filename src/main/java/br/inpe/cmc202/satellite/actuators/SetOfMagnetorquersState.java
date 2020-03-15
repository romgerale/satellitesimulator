package br.inpe.cmc202.satellite.actuators;

import org.hipparchus.geometry.euclidean.threed.Vector3D;

/**
 * SetOfMagnetorquerState.
 * 
 * @author alessandro.g.romero
 * 
 */
public class SetOfMagnetorquersState {
	/**
	 * Store the instantaneous torque applied by the magnetotorquers
	 * 
	 * N.m
	 */
	final private Vector3D torque_body;
	/**
	 * Store the instantaneous magnetic dipole produced by the magnetotorquers
	 * nA.m^2
	 */
	final private Vector3D magneticDipole_body;

	public SetOfMagnetorquersState(Vector3D torque_body,
			Vector3D magneticDipole_body) {
		this.torque_body = torque_body;
		this.magneticDipole_body = magneticDipole_body;
	}

	/**
	 * Get the control torque. N.m
	 * 
	 * @return
	 */
	public Vector3D getControlTorque() {
		return torque_body;
	}

	/**
	 * Get the magnetic dipole. nA.m^2
	 * 
	 * @return
	 */
	public Vector3D getMagneticDipole() {
		return magneticDipole_body;
	}
}