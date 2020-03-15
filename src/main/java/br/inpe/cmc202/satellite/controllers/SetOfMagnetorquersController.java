package br.inpe.cmc202.satellite.controllers;

import org.hipparchus.geometry.euclidean.threed.Vector3D;

/**
 * Compute the control of the set of magnetorques.
 * 
 * @author alessandro.g.romero
 * 
 */
public class SetOfMagnetorquersController {

	/**
	 * GAIN K
	 * 
	 * Empirically defined.
	 */
	private final static double K = 0.0013;

	/**
	 * @param reactionWheelsMomentum_body
	 * @return
	 */
	public Vector3D computeControl(Vector3D reactionWheelsMomentum_body) {
		return reactionWheelsMomentum_body.scalarMultiply(K);

	}
}
