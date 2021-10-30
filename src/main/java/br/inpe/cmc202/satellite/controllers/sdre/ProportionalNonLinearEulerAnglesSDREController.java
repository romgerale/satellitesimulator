package br.inpe.cmc202.satellite.controllers.sdre;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.CholeskyDecomposition;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.hipparchus.linear.RiccatiEquationSolver;
import org.hipparchus.linear.RiccatiEquationSolverImpl;
import org.math.plot.utils.FastMath;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.satellite.Satellite;

/**
 * SDRE CONTROL based on Euler Angles
 * 
 * @author alessandro.g.romero
 * 
 */
public class ProportionalNonLinearEulerAnglesSDREController extends
		ProportionalNonLinearBaseSDREController {

	final private Logger logger = LoggerFactory
			.getLogger(ProportionalNonLinearEulerAnglesSDREController.class);

	/**
	 * Constructor.
	 * 
	 * @param satellite
	 */
	public ProportionalNonLinearEulerAnglesSDREController(
			final Satellite satellite) {
		super(satellite);
		// to handle angular velocity in X (roll)
		// with error in the first 3000 seconds
		Q.setEntry(0, 0, 40);

		// Q_sqrt
		// it is an identity, but this works for the general case
		Q_sqrt = new CholeskyDecomposition(Q).getL();
	}

	/**
	 * Computes A for a given set of states.
	 * 
	 * @param eulerAngles
	 * @param angularVelocity
	 * @return
	 */
	protected RealMatrix computeA(final Vector3D eulerAngles,
			final Vector3D angularVelocity) {
		// A_eulerAngles
		// 0 & \frac{\sin\theta_3}{\cos\theta_2} &
		// \frac{\cos\theta_3}{\cos\theta_2} \\
		// 0 & \cos\theta_3 & -\sin\theta_3\\
		// 1 & \frac{\sin\theta_3\sin\theta_2}{\cos\theta_2} &
		// \frac{\cos\theta_3\sin\theta_2}{\cos\theta_2}
		final RealMatrix A_eulerAngles = MatrixUtils
				.createRealMatrix(new double[][] {
						{
								0,
								FastMath.sin(eulerAngles.getZ())
										/ FastMath.cos(eulerAngles.getY()),
								FastMath.cos(eulerAngles.getZ())
										/ FastMath.cos(eulerAngles.getY()) },
						{ 0, FastMath.cos(eulerAngles.getZ()),
								-FastMath.sin(eulerAngles.getZ()) },
						{
								1,
								(FastMath.sin(eulerAngles.getZ()) * FastMath
										.sin(eulerAngles.getY()))
										/ FastMath.cos(eulerAngles.getY()),
								(FastMath.cos(eulerAngles.getZ()) * FastMath
										.sin(eulerAngles.getY()))
										/ FastMath.cos(eulerAngles.getY()) } });

		final RealMatrix A_angularVelocity = computeAKinetics(angularVelocity);

		// A
		final RealMatrix A = MatrixUtils.createRealMatrix(6, 6);
		A.setSubMatrix(A_eulerAngles.getData(), 0, 3);
		A.setSubMatrix(A_angularVelocity.getData(), 3, 3);

		return A;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * br.inpe.cmc202.satellite.controllers.Controller#computeControlEulerAngles
	 * (org.hipparchus.geometry.euclidean.threed.Vector3D,
	 * org.hipparchus.geometry.euclidean.threed.Vector3D)
	 */
	public Vector3D computeControlEulerAngles(Vector3D eulerAngles,
			Vector3D angularVelocity) {

		// Euler angles and angular velocity
		final RealVector X = new ArrayRealVector(new double[] {
				eulerAngles.getX(), eulerAngles.getY(), eulerAngles.getZ(),
				angularVelocity.getX(), angularVelocity.getY(),
				angularVelocity.getZ() });

		// A(x)
		final RealMatrix A = computeA(eulerAngles, angularVelocity);

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

}
