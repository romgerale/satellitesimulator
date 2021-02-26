package br.inpe.cmc202.satellite.controllers.sdre;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.geometry.euclidean.threed.Vector3DComplement;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.hipparchus.linear.RiccatiEquationSolver;
import org.hipparchus.linear.RiccatiEquationSolverImpl;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.satellite.Satellite;

/**
 * SDRE CONTROL based on modified Rodrigues Parameters(MRPs)
 * 
 * @author alessandro.g.romero
 * 
 */
public class ProportionalNonLinearMRPSDREController extends
		ProportionalNonLinearBaseSDREController {

	final private Logger logger = LoggerFactory
			.getLogger(ProportionalNonLinearMRPSDREController.class);

	final protected String algebraicEquation;

	/**
	 * Constructor.
	 * 
	 * @param satellite
	 */
	public ProportionalNonLinearMRPSDREController(final Satellite satellite,
			String algebraicEquation) {
		super(satellite);
		this.algebraicEquation = algebraicEquation;
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
		// computing MRP
		final double inv = (1.0d + errorQuaternion.getQ0());
		final Vector3D mrp = new Vector3D(errorQuaternion.getQ1() / inv,
				errorQuaternion.getQ2() / inv, errorQuaternion.getQ3() / inv);

		if (logger.isTraceEnabled()) {
			logger.trace("MRP: {}", mrp);
		}

		// State
		// MRP and angular velocity
		final RealVector X = new ArrayRealVector(new double[] { mrp.getX(),
				mrp.getY(), mrp.getZ(), angularVelocity.getX(),
				angularVelocity.getY(), angularVelocity.getZ() });

		if (logger.isTraceEnabled()) {
			logger.trace("State: {}", X);
		}

		final RealMatrix angularVelocity2times = new Vector3DComplement(
				angularVelocity.toArray()).getTimesMatrix();
		final RealMatrix angularVelocity2transpose = MatrixUtils
				.createRowRealMatrix(angularVelocity.toArray());
		final RealMatrix mrp2transpose = MatrixUtils.createRowRealMatrix(mrp
				.toArray());
		final RealMatrix mrp2matrix = MatrixUtils.createColumnRealMatrix(mrp
				.toArray());
		final RealMatrix mrp2times = new Vector3DComplement(mrp.toArray())
				.getTimesMatrix();

		RealMatrix A = null;

		switch (algebraicEquation) {
		case "FIRST": {
			A = computeA1(angularVelocity, angularVelocity2times,
					angularVelocity2transpose, mrp2transpose, mrp2matrix);
			break;
		}
		case "SECOND": {
			A = computeA2(angularVelocity, angularVelocity2transpose,
					mrp2transpose, mrp2matrix, mrp2times);
			break;
		}

		case "ALPHA": { // FIRST * ALPHA + SECOND * (1-ALPHA)
			double alpha = satellite.getAlpha1();
			A = computeA1(angularVelocity, angularVelocity2times,
					angularVelocity2transpose, mrp2transpose, mrp2matrix)
					.scalarMultiply(alpha).add(
							computeA2(angularVelocity,
									angularVelocity2transpose, mrp2transpose,
									mrp2matrix, mrp2times).scalarMultiply(
									1 - alpha));
			break;
		}

		default:
			throw new RuntimeException("Approach for kinematics is not defined");
		}

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

	/**
	 * Compute A for the quaternion.
	 * 
	 * @param angularVelocity
	 * @param angularVelocity2times
	 * @param angularVelocity2transpose
	 * @param mrp2transpose
	 * @param mrp2matrix
	 * @return
	 */
	protected RealMatrix computeA1(Vector3D angularVelocity,
			final RealMatrix angularVelocity2times,
			final RealMatrix angularVelocity2transpose,
			final RealMatrix mrp2transpose, final RealMatrix mrp2matrix) {
		RealMatrix A;
		{
			// FIRST ALGEBRAIC EQUATION

			final RealMatrix first_A_1 = angularVelocity2times.scalarMultiply(
					-.5d).add(
					MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(
							angularVelocity2transpose.multiply(mrp2matrix)
									.scalarMultiply(.5d).getEntry(0, 0)));
			final RealMatrix first_A_2 = MatrixUtils
					.createRealIdentityMatrix(3).scalarMultiply(
							.25d * (1 - mrp2transpose.multiply(mrp2matrix)
									.getEntry(0, 0)));

			final RealMatrix A_angularVelocity = computeAKinetics(angularVelocity);

			// A
			A = MatrixUtils.createRealMatrix(6, 6);
			A.setSubMatrix(first_A_1.getData(), 0, 0);
			// dynamics defines 1
			// however, for equalizing quaternion non-unique description -1
			// A.setSubMatrix(first_A_2.getData(), 0, 3);
			A.setSubMatrix(first_A_1.scalarMultiply(1).getData(), 0, 0);
			A.setSubMatrix(first_A_2.scalarMultiply(-1).getData(), 0, 3);
			A.setSubMatrix(A_angularVelocity.getData(), 3, 3);
		}
		return A;
	}

	/**
	 * Compute A for the quaternion.
	 * 
	 * @param angularVelocity
	 * @param angularVelocity2transpose
	 * @param mrp2transpose
	 * @param mrp2matrix
	 * @param mrp2times
	 * @return
	 */
	protected RealMatrix computeA2(Vector3D angularVelocity,
			final RealMatrix angularVelocity2transpose,
			final RealMatrix mrp2transpose, final RealMatrix mrp2matrix,
			final RealMatrix mrp2times) {
		RealMatrix A;
		{
			// SECOND ALGEBRAIC EQUATION
			final RealMatrix first_A_1 = MatrixUtils
					.createRealIdentityMatrix(3).scalarMultiply(
							.5d * (1 - mrp2transpose.multiply(mrp2matrix)
									.getEntry(0, 0)));

			final RealMatrix first_A_2 = mrp2times;

			final RealMatrix first_A_3 = mrp2matrix
					.multiply(angularVelocity2transpose);

			// dynamics defines .5d
			// however, for equalizing quaternion non-unique description -.5
			// final RealMatrix first_A =
			// first_A_1.add(first_A_2).add(first_A_3)
			// .scalarMultiply(.5d);
			final RealMatrix first_A = first_A_1.add(first_A_2).add(first_A_3)
					.scalarMultiply(-.5d);

			final RealMatrix A_angularVelocity = computeAKinetics(angularVelocity);

			// A
			A = MatrixUtils.createRealMatrix(6, 6);
			A.setSubMatrix(first_A.getData(), 0, 3);
			A.setSubMatrix(A_angularVelocity.getData(), 3, 3);
		}
		return A;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return "ProportionalNonLinearMRPSDREController [algebraicEquation="
				+ algebraicEquation + ", alpha=" + satellite.getAlpha1() + "]";
	}
}
