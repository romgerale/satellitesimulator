package br.inpe.cmc202.satellite.actuators;

import org.hipparchus.geometry.euclidean.threed.Vector3D;

/**
 * SetOfThrustersState.
 * 
 * @author alessandro.g.romero
 * 
 */
public class SetOfThrustersState {
	/**
	 * Store the instantaneous torque applied by the thrusters
	 * 
	 * N.m
	 */
	final private Vector3D torque_body;

	/**
	 * Store the instantaneous torque 'desired' from each reaction wheel.
	 * 
	 * N.m
	 */
	private Vector3D torque_desired_body;
	
	public SetOfThrustersState(Vector3D torque_body, Vector3D torque_desired_body) {
		this.torque_body = torque_body;
		this.torque_desired_body = torque_desired_body;
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
	 * Get the 'desired' control torque of the set of reaction wheels. N.m
	 * 
	 * @return
	 */
	public Vector3D getIdealControlTorque() {
		return torque_desired_body;
	}
}