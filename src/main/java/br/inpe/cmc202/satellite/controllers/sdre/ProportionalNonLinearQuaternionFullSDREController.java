package br.inpe.cmc202.satellite.controllers.sdre;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.geometry.euclidean.threed.Vector3DComplement;
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
 * SDRE CONTROL based on full Quaternions and GIBBS vector
 * 
 * @author alessandro.g.romero
 * 
 */
public class ProportionalNonLinearQuaternionFullSDREController extends
		ProportionalNonLinearBaseSDREController {

	final private Logger logger = LoggerFactory
			.getLogger(ProportionalNonLinearQuaternionFullSDREController.class);

	final protected String kinematicsDefinition;

	/**
	 * Constructor.
	 * 
	 * @param satellite
	 */
	public ProportionalNonLinearQuaternionFullSDREController(
			final Satellite satellite, final String kinematicsDefinition) {
		super(satellite);
		R = MatrixUtils.createRealIdentityMatrix(3);
		Q = MatrixUtils.createRealIdentityMatrix(7);
		C = MatrixUtils.createRealIdentityMatrix(7);

		// B
		B = MatrixUtils.createRealMatrix(7, 3);
		B.setSubMatrix(satellite.getI_inverse_nominal().scalarMultiply(-1).getData()
				.clone(), 4, 0);

		// reseting elements outside diagonal
		// avoiding numerical problems due to small numbers
		for (int i = 0; i < B.getRowDimension() - 4; i++) {
			for (int j = 0; j < B.getColumnDimension(); j++) {
				if (i != j) {
					B.setEntry(i + 4, j, 0d);
				}
			}
		}

		// Q_sqrt
		// it is an identity, but this works for the general case
		Q_sqrt = new CholeskyDecomposition(Q).getL();

		this.kinematicsDefinition = kinematicsDefinition;
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
				errorQuaternion.getQ1(), errorQuaternion.getQ2(),
				errorQuaternion.getQ3(),
				0, // errorQuaternion.getQ0(),
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
		final RealMatrix A = computeA(errorQuaternion, angularVelocity,
				kinematicsDefinition);

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
	 * Compute A for the quaternion dynamics taking into account the kinematics
	 * equation.
	 * 
	 * @param errorQuaternion
	 * @param angularVelocity
	 * @param kinematicsEquation
	 * @return
	 */
	protected RealMatrix computeA(Rotation errorQuaternion,
			Vector3D angularVelocity, String kinematicsEquation) {
		RealMatrix A = MatrixUtils.createRealMatrix(7, 7);

		switch (kinematicsEquation) {
		case "OMEGA": {
			// Quaternion OMEGA

			final RealMatrix half_omega = MatrixUtils.createRealMatrix(
					new double[][] {
							{ 0, angularVelocity.getZ(),
									-angularVelocity.getY(),
									angularVelocity.getX() },
							{ -angularVelocity.getZ(), 0,
									angularVelocity.getX(),
									angularVelocity.getY() },
							{ angularVelocity.getY(), -angularVelocity.getX(),
									0, angularVelocity.getZ() },
							{ -angularVelocity.getX(), -angularVelocity.getY(),
									-angularVelocity.getZ(), 0 } })
					.scalarMultiply(.5d);

			final RealMatrix A_angularVelocity = computeAKinetics(angularVelocity);

			// A
			A.setSubMatrix(half_omega.getData(), 0, 0);
			A.setSubMatrix(A_angularVelocity.getData(), 4, 4);
			break;
		}

		case "XI": {
			// Quaternion XI
			final RealMatrix half_xi = MatrixUtils.createRealMatrix(
					new double[][] {
							{ errorQuaternion.getQ0(),
									-errorQuaternion.getQ3(),
									errorQuaternion.getQ2() },
							{ errorQuaternion.getQ3(), errorQuaternion.getQ0(),
									-errorQuaternion.getQ1() },
							{ -errorQuaternion.getQ2(),
									errorQuaternion.getQ1(),
									errorQuaternion.getQ0() },
							{ -errorQuaternion.getQ1(),
									-errorQuaternion.getQ2(),
									-errorQuaternion.getQ3() } })
					.scalarMultiply(.5d);

			final RealMatrix A_angularVelocity = computeAKinetics(angularVelocity);

			// A
			A.setSubMatrix(half_xi.getData(), 0, 4);
			A.setSubMatrix(A_angularVelocity.getData(), 4, 4);
			break;
		}

		case "GIBBS": {
			A = computeA1(errorQuaternion, angularVelocity);
			break;
		}

		case "GIBBS_SECOND": {
			A = computeA2(errorQuaternion, angularVelocity);
			break;
		}

		case "ALPHA": {
			double alpha = satellite.getAlpha1();
			A = computeA1(errorQuaternion, angularVelocity).scalarMultiply(
					alpha).add(
					computeA2(errorQuaternion, angularVelocity).scalarMultiply(
							1 - alpha));

			break;
		}

		default:
			throw new RuntimeException("Approach for kinemtics is not defined");
		}

		return A;
	}

	/**
	 * ComputeA for Gibbs.
	 * 
	 * @param errorQuaternion
	 * @param angularVelocity
	 * @return
	 */
	private RealMatrix computeA1(Rotation errorQuaternion,
			Vector3D angularVelocity) {
		RealMatrix A = MatrixUtils.createRealMatrix(7, 7);
		// GIBBS
		final RealMatrix angularVelocity2times = new Vector3DComplement(
				angularVelocity.toArray()).getTimesMatrix();
		final RealMatrix angularVelocity2transpose = MatrixUtils
				.createRowRealMatrix(angularVelocity.toArray());

		final RealMatrix half_omega1 = MatrixUtils.createRealMatrix(4, 3);
		half_omega1.setSubMatrix(angularVelocity2times.getData(), 0, 0);
		half_omega1.setSubMatrix(angularVelocity2transpose.getData(), 3, 0);
		final RealMatrix half_omega11 = half_omega1.scalarMultiply(-.5d);

		final RealMatrix half_omega2 = MatrixUtils.createRealMatrix(4, 3);
		half_omega2
				.setSubMatrix(MatrixUtils.createRealIdentityMatrix(3)
						.scalarMultiply(errorQuaternion.getQ0() * -.5d)
						.getData(), 0, 0); // it should be .5d

		final RealMatrix A_angularVelocity = computeAKinetics(angularVelocity);

		// A
		A.setSubMatrix(half_omega11.getData(), 0, 0);
		A.setSubMatrix(half_omega2.getData(), 0, 4);
		A.setSubMatrix(A_angularVelocity.getData(), 4, 4);
		return A;
	}

	/**
	 * Compute one more option.
	 * 
	 * @param errorQuaternion
	 * @param angularVelocity
	 * @return
	 */
	private RealMatrix computeA2(Rotation errorQuaternion,
			Vector3D angularVelocity) {
		RealMatrix A = MatrixUtils.createRealMatrix(7, 7);
		// GIBBS
		final double[] g = new double[3];
		g[0] = errorQuaternion.getQ1();
		g[1] = errorQuaternion.getQ2();
		g[2] = errorQuaternion.getQ3();
		final Vector3DComplement gVector = new Vector3DComplement(g);
		final RealMatrix g2Times = gVector.getTimesMatrix();
		final RealMatrix angularVelocity2transpose = MatrixUtils
				.createRowRealMatrix(angularVelocity.toArray());

		final RealMatrix half_omega1 = MatrixUtils.createRealMatrix(4, 4);
		half_omega1.setSubMatrix(angularVelocity2transpose.scalarMultiply(-.5d)
				.getData(), 3, 0);

		final RealMatrix half_omega2 = MatrixUtils.createRealMatrix(4, 3);
		half_omega2
				.setSubMatrix(
						g2Times.scalarMultiply(.5d)
								.add(MatrixUtils.createRealIdentityMatrix(3)
										.scalarMultiply(
												errorQuaternion.getQ0() * -.5d))
								.getData(), 0, 0); // it should be .5d

		final RealMatrix A_angularVelocity = computeAKinetics(angularVelocity);

		// A
		A.setSubMatrix(half_omega1.getData(), 0, 0);
		A.setSubMatrix(half_omega2.scalarMultiply(1d).getData(), 0, 4);// -1 due
																		// to
																		// quaternion
																		// signal
		A.setSubMatrix(A_angularVelocity.getData(), 4, 4);
		return A;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return "ProportionalNonLinearQuaternionSDREController [kinematicsDefinition="
				+ kinematicsDefinition + ", Q_sqrt=" + Q_sqrt + "]";
	}

}
