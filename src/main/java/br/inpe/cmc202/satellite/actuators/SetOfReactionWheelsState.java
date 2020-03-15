package br.inpe.cmc202.satellite.actuators;

import org.hipparchus.geometry.euclidean.threed.Vector3D;

/**
 * SetOfReactionWheelsState.
 * 
 * @author alessandro.g.romero
 * 
 */
public class SetOfReactionWheelsState {
	/**
	 * Store the instantaneous torque applied by each reaction wheel.
	 */
	private Vector3D torque_body;
	/**
	 * Store the angular velocity in each reaction wheel (integrated).
	 */
	private Vector3D angularVelocity;
	/**
	 * Store inertia
	 */
	private double inertia;

	public SetOfReactionWheelsState(Vector3D torque_body,
			Vector3D angularVelocity, double inertia) {
		this.angularVelocity = angularVelocity;
		this.torque_body = torque_body;
		this.inertia = inertia;
	}

	/**
	 * Get angular velocity of the set of reaction wheels.
	 * 
	 * radians/s
	 * 
	 * @return
	 */
	public Vector3D getAngularVelocity() {
		return angularVelocity;
	}

	/**
	 * Get angular momentum of the set of reaction wheels.
	 * 
	 * kg.m^2.radians/s
	 * 
	 * @param angularVelocity_body
	 * @return
	 */
	public Vector3D getAngularMomentum(Vector3D angularVelocity_body) {
		// h_{w,n} = I_{n,s}a_n^T\omega + I_{n,s}\omega_n
		return angularVelocity.add(angularVelocity_body)
				.scalarMultiply(inertia);
	}

	/**
	 * Get the norm of angular momentum of the set of reaction wheels.
	 * kg.m^2.radians/s
	 * 
	 * @param angularVelocity_body
	 * @return
	 */
	public double getAngularMomentumNorm(Vector3D angularVelocity_body) {
		return getAngularMomentum(angularVelocity_body).getNorm();
	}

	/**
	 * Get the control torque of the set of reaction wheels. N.m
	 * 
	 * @return
	 */
	public Vector3D getControlTorque() {
		return torque_body;
	}

}