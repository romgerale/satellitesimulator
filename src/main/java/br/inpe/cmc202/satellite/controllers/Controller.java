package br.inpe.cmc202.satellite.controllers;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;

/**
 * Interface for controllers.
 * 
 * @author alessandro.g.romero
 * 
 */
public interface Controller {

	/**
	 * Receives the state and return the control torque.
	 * 
	 * @param errorEulerAngles
	 * @param angularVelocity
	 * @return
	 */
	Vector3D computeControlEulerAngles(Vector3D errorEulerAngles,
			Vector3D angularVelocity);

	/**
	 * Receives the state and return the control torque.
	 * 
	 * @param sunVector_body
	 * @param angularVelocity
	 * @return
	 */
	Vector3D computeControlSunVector(Vector3D sunVector_body,
			Vector3D angularVelocity);

	/**
	 * Receives the state and return the control torque.
	 * 
	 * @param errorQuaternion
	 * @param angularVelocity
	 * @return
	 */
	Vector3D computeControlQuaternion(Rotation errorQuaternion,
			Vector3D angularVelocity);
}
