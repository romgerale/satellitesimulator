package br.inpe.cmc202.satellite.controllers.sdre.hinfinity;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.geometry.euclidean.threed.Vector3DComplement;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.DecompositionSolver;
import org.hipparchus.linear.LUDecomposition;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.OrderedEigenDecomposition;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.hipparchus.linear.RiccatiEquationSolver;
import org.hipparchus.linear.RiccatiEquationSolverImpl;
import org.hipparchus.util.FastMath;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.satellite.Satellite;
import br.inpe.cmc202.satellite.controllers.sdre.ProportionalNonLinearMRPSDREController;

/**
 * SDRE CONTROL based on modified Rodrigues Parameters(MRPs)
 * 
 * @author alessandro.g.romero
 * 
 */
public class ProportionalNonLinearMRPSDREHInfinityController extends ProportionalNonLinearMRPSDREController {

	final private Logger logger = LoggerFactory.getLogger(ProportionalNonLinearMRPSDREHInfinityController.class);

	protected RealMatrix D = MatrixUtils.createRealMatrix(6, 3);

	private double gama = 0.0d;

	/**
	 * Constructor.
	 * 
	 * @param satellite
	 */
	public ProportionalNonLinearMRPSDREHInfinityController(final Satellite satellite, String algebraicEquation) {
		super(satellite, algebraicEquation);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * br.inpe.cmc202.satellite.controllers.ProportionalNonLinearBaseSDREController
	 * # computeControlQuaternion(org.hipparchus.geometry.euclidean.threed.Rotation
	 * , org.hipparchus.geometry.euclidean.threed.Vector3D)
	 */
	public Vector3D computeControlQuaternion(Rotation errorQuaternion, Vector3D angularVelocity) {
		// computing MRP
		final double inv = (1.0d + errorQuaternion.getQ0());
		final Vector3D mrp = new Vector3D(errorQuaternion.getQ1() / inv, errorQuaternion.getQ2() / inv,
				errorQuaternion.getQ3() / inv);

		if (logger.isTraceEnabled()) {
			logger.trace("MRP: {}", mrp);
		}

		// State
		// MRP and angular velocity
		final RealVector X = new ArrayRealVector(new double[] { mrp.getX(), mrp.getY(), mrp.getZ(),
				angularVelocity.getX(), angularVelocity.getY(), angularVelocity.getZ() });

		if (logger.isTraceEnabled()) {
			logger.trace("State: {}", X);
		}

		final RealMatrix angularVelocity2times = new Vector3DComplement(angularVelocity.toArray()).getTimesMatrix();
		final RealMatrix angularVelocity2transpose = MatrixUtils.createRowRealMatrix(angularVelocity.toArray());
		final RealMatrix mrp2transpose = MatrixUtils.createRowRealMatrix(mrp.toArray());
		final RealMatrix mrp2matrix = MatrixUtils.createColumnRealMatrix(mrp.toArray());
		final RealMatrix mrp2times = new Vector3DComplement(mrp.toArray()).getTimesMatrix();

		RealMatrix A = null;

		switch (algebraicEquation) {
		case "FIRST": {
			A = computeA1(angularVelocity, angularVelocity2times, angularVelocity2transpose, mrp2transpose, mrp2matrix);
			break;
		}
		case "SECOND": {
			A = computeA2(angularVelocity, angularVelocity2transpose, mrp2transpose, mrp2matrix, mrp2times);
			break;
		}
		case "ALPHA": { // FIRST * ALPHA + SECOND * (1-ALPHA)
			double alpha = satellite.getAlpha1();
			A = computeA1(angularVelocity, angularVelocity2times, angularVelocity2transpose, mrp2transpose, mrp2matrix)
					.scalarMultiply(alpha)
					.add(computeA2(angularVelocity, angularVelocity2transpose, mrp2transpose, mrp2matrix, mrp2times)
							.scalarMultiply(1 - alpha));
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

			// HINFINTY WITH LEFT COPRIME FACTORIZATION
			final RealMatrix R_ = MatrixUtils.createRealIdentityMatrix(6).add(D.multiply(D.transpose()));
			final RealMatrix R_inv = MatrixUtils.inverse(R_);
			final RealMatrix S_ = MatrixUtils.createRealIdentityMatrix(3).add(D.transpose().multiply(D));
			final RealMatrix S_inv = MatrixUtils.inverse(S_);

			// X
			final RealMatrix AX = A.subtract(B.multiply(S_inv).multiply(D.transpose()).multiply(C));
			final RealMatrix BX = B;
			final RealMatrix QX = C.transpose().multiply(R_inv).multiply(C);
			final RealMatrix RX = S_;
			// solving Riccati
			final RiccatiEquationSolver riccatiSolverX = new RiccatiEquationSolverImpl(AX, BX, QX, RX);
			final RealMatrix PX = riccatiSolverX.getP();

			// Z
			final RealMatrix AZ = AX.transpose();
			final RealMatrix BZ = C.transpose();
			final RealMatrix QZ = B.multiply(S_inv).multiply(B.transpose());
			final RealMatrix RZ = R_;
			// solving Riccati
			final RiccatiEquationSolver riccatiSolverZ = new RiccatiEquationSolverImpl(AZ, BZ, QZ, RZ);
			final RealMatrix PZ = riccatiSolverZ.getP();

			// COMPUTING GAMA
			final RealMatrix PXZ = PX.multiply(PZ);
			final OrderedEigenDecomposition oed = new OrderedEigenDecomposition(PXZ);
			final double spectralRadiusOfPXZ = oed.getD().getEntry(PXZ.getRowDimension() - 1,
					PXZ.getColumnDimension() - 1);
			gama = FastMath.sqrt(1 + spectralRadiusOfPXZ);
			if (gama == Double.NaN) {
				logger.warn("Gama computed: {}", gama);
				throw new RuntimeException("Problems in the computation of Gama!");
			}

			// COMPUTING NEW SYSTEM BASED ON GAMA
			final RealMatrix F = S_inv.scalarMultiply(-1.0)
					.multiply(D.transpose().multiply(C).add(B.transpose().multiply(PX)));
			final RealMatrix L = MatrixUtils.createRealIdentityMatrix(6).scalarMultiply(gama * gama).add(PXZ);
			// COMPUTING INVERSE OF L^T USING LU Decomposition
			// numerically better for square matrix
			final LUDecomposition lDecom = new LUDecomposition(L.transpose());
			final DecompositionSolver s = lDecom.getSolver();
			if (!s.isNonSingular()) {
				logger.warn("L^T is singular: {}", !s.isNonSingular());
				throw new RuntimeException("Problems in the computation of inverse of L^T! It is singular!");
			}
			final RealMatrix L_inv = s.getInverse();
			//
			final RealMatrix AH = A.add(B.multiply(F)).add(L_inv.scalarMultiply(gama * gama).multiply(PZ)
					.multiply(C.transpose()).multiply(C.add(D.multiply(F))));
			final RealMatrix BH = L_inv.scalarMultiply(gama * gama).multiply(PZ).multiply(C.transpose());
			// RealMatrix CH = B.transpose().multiply(PX);
			// RealMatrix DH = D.transpose().scalarMultiply(-1.0);
			final RealMatrix QH = MatrixUtils.createRealIdentityMatrix(6);
			final RealMatrix RH = MatrixUtils.createRealIdentityMatrix(6);

			// solving Riccati for H-infinity and computing K
			final RiccatiEquationSolver riccatiSolverH = new RiccatiEquationSolverImpl(AH, BH, QH, RH);
			final RealMatrix PH = riccatiSolverH.getP();
			final RealMatrix K = this.R_inv.multiply(B.transpose().multiply(PH));

			checkPointwiseStability(A, B, K);

			// computing control
			// u = -Kx
			return new Vector3D(K.operate(X).mapMultiply(-1).toArray());
		} catch (RuntimeException re) {
			this.countNumericalErrors++;
			// if a numerical error occurs in the SDRE the default control is 0
			logger.warn("Unexpected numerical error was occurred in the SDRE solving. Assuming control ZERO.", re);
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
		return "ProportionalNonLinearMRPSDREHInfinityController [algebraicEquation=" + algebraicEquation + ", alpha="
				+ satellite.getAlpha1() + "]";
	}

	/**
	 * Returns the last computed gama.
	 * 
	 * @return
	 */
	public double getGama() {
		return gama;
	}
}
