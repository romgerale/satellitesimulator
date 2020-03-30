package br.inpe.cmc202.satellite.controllers.sdre.hinfinity;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
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
import br.inpe.cmc202.satellite.controllers.sdre.ProportionalNonLinearQuaternionFullSDREController;

/**
 * SDRE CONTROL + H INFINITY (WITH LEFT COPRIME FACTORIZATION) based on full
 * Quaternions and GIBBS vector
 * 
 * @author alessandro.g.romero
 * 
 */
public class ProportionalNonLinearQuaternionFullSDREHInfinityController
		extends ProportionalNonLinearQuaternionFullSDREController {

	final private Logger logger = LoggerFactory
			.getLogger(ProportionalNonLinearQuaternionFullSDREHInfinityController.class);

	protected RealMatrix D = MatrixUtils.createRealMatrix(7, 3);
	
	private double gama = 0.0d;

	/**
	 * Constructor.
	 * 
	 * @param satellite
	 */
	public ProportionalNonLinearQuaternionFullSDREHInfinityController(final Satellite satellite,
			final String kinematicsDefinition) {
		super(satellite, kinematicsDefinition);
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

		// Quaternion (axis first and real part last) and angular velocity
		final RealVector X = new ArrayRealVector(
				new double[] { errorQuaternion.getQ1(), errorQuaternion.getQ2(), errorQuaternion.getQ3(), 0, // errorQuaternion.getQ0(),
						angularVelocity.getX(), angularVelocity.getY(), angularVelocity.getZ() });

		if (logger.isTraceEnabled()) {
			logger.trace("State: X {}", X);
			logger.trace("State Vector Norm ||X|| {}", X.getNorm());
		}

		if (X.getNorm() < 1E-4d) {
			// State Vector Norm ||X|| 7.811843713517442E-7
			// breaking the loop
			// too much close to the origin
			logger.debug("Too much close to the origin (Vector Norm ||X|| {}). Assuming control ZERO.", X.getNorm());
			return new Vector3D(0, 0, 0);
		}

		// important define the formulation for kinematics
		final RealMatrix A = computeA(errorQuaternion, angularVelocity, this.kinematicsDefinition);

		if (logger.isTraceEnabled()) {
			logger.trace("-- A {}", f.format(A));
		}

		checkPointwiseControlability(A, B);
		checkPointwiseObservability(A, Q_sqrt);

		try {
			// HINFINTY WITH LEFT COPRIME FACTORIZATION
			final RealMatrix R_ = MatrixUtils.createRealIdentityMatrix(7).add(D.multiply(D.transpose()));
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
			final double spectralRadiusOfPXZ = oed.getD().getEntry(PXZ.getRowDimension() - 1, PXZ.getColumnDimension() - 1);
			gama = FastMath.sqrt(1 + spectralRadiusOfPXZ);
			if (gama == Double.NaN) {
				logger.warn("Gama computed: {}", gama);
				throw new RuntimeException("Problems in the computation of Gama!");
			}

			// COMPUTING NEW SYSTEM BASED ON GAMA
			final RealMatrix F = S_inv.scalarMultiply(-1.0)
					.multiply(D.transpose().multiply(C).add(B.transpose().multiply(PX)));
			final RealMatrix L = MatrixUtils.createRealIdentityMatrix(7).scalarMultiply(gama * gama).add(PXZ);
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
			final RealMatrix QH = MatrixUtils.createRealIdentityMatrix(7);
			final RealMatrix RH = MatrixUtils.createRealIdentityMatrix(7);

			// solving Riccati for H-infinity and computing K
			final RiccatiEquationSolver riccatiSolverH = new RiccatiEquationSolverImpl(AH, BH, QH, RH);
			final RealMatrix PH = riccatiSolverH.getP();
			final RealMatrix K = this.R_inv.multiply(B.transpose().multiply(PH));

			// it is not pointwise stable but it converges
			// it demands analysis
			//checkPointwiseStability(A, B, K);

			// computing control
			// u = -Kx
			return new Vector3D(K.operate(X).mapMultiply(-1.0).toArray());
		} catch (RuntimeException re) {
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
		return "ProportionalNonLinearQuaternionFullSDREHInfinityController [kinematicsDefinition="
				+ kinematicsDefinition + ", Q_sqrt=" + Q_sqrt + "]";
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
