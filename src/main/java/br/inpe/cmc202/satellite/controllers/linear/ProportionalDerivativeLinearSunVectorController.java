package br.inpe.cmc202.satellite.controllers.linear;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;

import br.inpe.cmc202.satellite.controllers.BaseController;

/**
 * SunVector controller
 * 
 * @author alessandro.g.romero
 * 
 */
public class ProportionalDerivativeLinearSunVectorController extends
		BaseController {

	final RealMatrix K = MatrixUtils.createRealMatrix(new double[][] {
			{ 1, 0, 0, -24, 0, 0 }, { 0, 1, 0, 0, -26, 0 },
			{ 0, 0, 1, 0, 0, -32 } });

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * br.inpe.cmc202.satellite.controllers.Controller#computeControlSunVector
	 * (org.hipparchus.geometry.euclidean.threed.Vector3D,
	 * org.hipparchus.geometry.euclidean.threed.Vector3D)
	 */
	public Vector3D computeControlSunVector(Vector3D sunVector_body,
			Vector3D angularVelocity) {

		// Sun Vector and angular velocity
		final RealVector X = new ArrayRealVector(new double[] { 0,
				-sunVector_body.getZ(), sunVector_body.getY(),
				angularVelocity.getX(), angularVelocity.getY(),
				angularVelocity.getZ() });

		return new Vector3D(K.operate(X).mapMultiply(-1).toArray());
	}

}
