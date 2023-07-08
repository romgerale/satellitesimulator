package br.inpe.cmc202.satellite.controllers.sdre;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.CholeskyDecomposition;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.hipparchus.linear.RiccatiEquationSolver;
import org.hipparchus.linear.RiccatiEquationSolverImpl;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.satellite.Satellite;

/**
 * SDRE CONTROL for Angular Velocity disregarding Quaternion error.
 * 
 * @author alessandro.g.romero
 * 
 */
public class ProportionalNonLinearAngularVelocitySDREController extends
		ProportionalNonLinearBaseSDREController {

	final private Logger logger = LoggerFactory
			.getLogger(ProportionalNonLinearAngularVelocitySDREController.class);


	/**
	 * Constructor.
	 * 
	 * @param satellite
	 */
	public ProportionalNonLinearAngularVelocitySDREController(
			final Satellite satellite) {
		super(satellite);
		R = MatrixUtils.createRealIdentityMatrix(3);
		Q = MatrixUtils.createRealIdentityMatrix(3);
		C = MatrixUtils.createRealIdentityMatrix(3);

		// B
		B = MatrixUtils.createRealMatrix(satellite.getI_inverse_nominal().scalarMultiply(-1).getData()
				.clone());

		// Q_sqrt
		// it is an identity, but this works for the general case
		Q_sqrt = new CholeskyDecomposition(Q).getL();

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * br.inpe.cmc202.satellite.controllers.ProportionalNonLinearBaseSDREController
	 * #
	 * computeControlQuaternion(org.hipparchus.geometry.euclidean.threed.Rotation
	 * , org.hipparchus.geometry.euclidean.threed.Vector3D)
	 */
	public Vector3D computeControlQuaternion(Rotation errorQuaternion,
			Vector3D angularVelocity) {

		// Quaternion (axis first and real part last) and angular velocity
		final RealVector X = new ArrayRealVector(new double[] {
				angularVelocity.getX(), angularVelocity.getY(),
				angularVelocity.getZ() });

		if (logger.isTraceEnabled()) {
			logger.trace("State: X {}", X);
			logger.trace("State Vector Norm ||X|| {}", X.getNorm());
		}

		if (X.getNorm() < 1E-4d) {
			// State Vector Norm ||X|| 7.811843713517442E-7
			// breaking the loop
			// too much close to the origin
			logger.debug(
					"Too much close to the origin (Vector Norm ||X|| {}). Assuming control ZERO.",
					X.getNorm());
		//	return new Vector3D(0, 0, 0);
		}

		// important define the formulation for kinematics
		final RealMatrix A = computeAKinetics(angularVelocity);

		if (logger.isTraceEnabled()) {
			logger.trace("-- A {}", f.format(A));
		}

		try {
			checkPointwiseControlability(A, B);
			checkPointwiseObservability(A, Q_sqrt);

			// solving Riccati
			RiccatiEquationSolver riccatiSolver = new RiccatiEquationSolverImpl(
					A, B, Q, R);
			final RealMatrix K = riccatiSolver.getK();

			checkPointwiseStability(A, B, K);

			// computing control
			// u = -Kx
			return new Vector3D(K.operate(X).mapMultiply(-1).toArray());
		} catch (RuntimeException re) {
			this.countNumericalErrors++;
			// if a numerical error occurs in the SDRE the default control is 0
			logger.warn(
					"Unexpected numerical error was occurred in the SDRE solving. Assuming control ZERO.",
					re);
			return new Vector3D(0, 0, 0);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return "ProportionalNonLinearAngularVelocitySDREController [ Q_sqrt=" + Q_sqrt + "]";
	}

}
