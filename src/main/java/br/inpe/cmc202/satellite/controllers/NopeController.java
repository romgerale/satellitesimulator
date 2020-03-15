package br.inpe.cmc202.satellite.controllers;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;

/**
 * NOPE controller
 * 
 * @author alessandro.g.romero
 * 
 */
public class NopeController implements Controller {

	public Vector3D computeControlEulerAngles(Vector3D eulerAngles,
			Vector3D angularVelocity) {
		return Vector3D.ZERO;
	}

	public Vector3D computeControlSunVector(Vector3D sunVector_body,
			Vector3D angularVelocity) {
		return null;
	}

	public Vector3D computeControlQuaternion(Rotation errorQuaternion,
			Vector3D angularVelocity) {
		return null;
	}

}
