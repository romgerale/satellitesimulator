package br.inpe.cmc202.satellite.controllers.lqr;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.hipparchus.linear.RiccatiEquationSolver;
import org.hipparchus.linear.RiccatiEquationSolverImpl;

import br.inpe.cmc202.satellite.Satellite;
import br.inpe.cmc202.satellite.controllers.BaseController;

/**
 * LQR CONTROL based on Euler Angles
 * 
 * @author alessandro.g.romero
 * 
 */
public class ProportionalLinearEulerAnglesLQRController extends BaseController {

	final private RealMatrix K;

	/**
	 * Constructor of the controller. It computes the K using the
	 * RiccatiEquationSolver - LQR.
	 */
	public ProportionalLinearEulerAnglesLQRController(final Satellite satellite) {
		final RealMatrix A = MatrixUtils.createRealMatrix(new double[][] {
				{ 0, 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 1, 0 },
				{ 0, 0, 0, 0, 0, 1 }, { 0, 0, 0, 0, 0, 0 },
				{ 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0 } });
		final RealMatrix B = MatrixUtils.createRealMatrix(6, 3);
		B.setSubMatrix(satellite.getI_inverse_nominal().scalarMultiply(-1).getData()
				.clone(), 3, 0);

		// reseting elements outside diagonal
		// avoiding numerical problems due to small numbers
		for (int i = 0; i < B.getRowDimension() - 3; i++) {
			for (int j = 0; j < B.getColumnDimension(); j++) {
				if (i != j) {
					B.setEntry(i + 3, j, 0d);
				}
			}
		}

		final RealMatrix R = MatrixUtils.createRealIdentityMatrix(3);
		final RealMatrix Q = MatrixUtils.createRealIdentityMatrix(6);

		checkPointwiseControlability(A, B);

		RiccatiEquationSolver riccatiSolver = new RiccatiEquationSolverImpl(A,
				B, Q, R);
		this.K = riccatiSolver.getK();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * br.inpe.cmc202.satellite.controllers.Controller#computeControlEulerAngles
	 * (org.hipparchus.geometry.euclidean.threed.Vector3D,
	 * org.hipparchus.geometry.euclidean.threed.Vector3D)
	 */
	public Vector3D computeControlEulerAngles(Vector3D errorEulerAngles,
			Vector3D angularVelocity) {

		// Euler angles and angular velocity
		final RealVector X = new ArrayRealVector(new double[] {
				errorEulerAngles.getX(), errorEulerAngles.getY(),
				errorEulerAngles.getZ(), angularVelocity.getX(),
				angularVelocity.getY(), angularVelocity.getZ() });

		return new Vector3D(K.operate(X).mapMultiply(-1).toArray());
	}

}
