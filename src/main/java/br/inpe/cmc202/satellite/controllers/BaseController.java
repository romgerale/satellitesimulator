package br.inpe.cmc202.satellite.controllers;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.EigenDecomposition;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.SingularValueDecomposition;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Base controller with helper functions for checking controllability,
 * obsevability and stability.
 * 
 * @author alessandro.g.romero
 * 
 */
public class BaseController implements Controller {

	private final Logger logger = LoggerFactory.getLogger(BaseController.class);

	private double detControllability;
	private double conditionNumberControllability;

	public BaseController() {
		super();
	}

	/**
	 * Check stability of the step.
	 * 
	 * @param A
	 * @param B
	 * @param K
	 */
	protected void checkPointwiseStability(final RealMatrix A,
			final RealMatrix B, final RealMatrix K) {
		final RealMatrix closedLoop = A.subtract(B.multiply(K));
		EigenDecomposition eigDecomp = new EigenDecomposition(closedLoop);
		for (double eigenValues : eigDecomp.getRealEigenvalues()) {
			if (eigenValues > 0) {
				logger.warn(
						"STABILITY CHECK - UNSTABLE STEP - Real part of an eigen value of the step is non negative. {} ",
						eigenValues);
				throw new RuntimeException(
						"STABILITY CHECK - UNSTABLE STEP - Real part of an eigen value of the step is non negative.");
			}
		}

	}

	/**
	 * Check controlability of the step.
	 * 
	 * @param A
	 * @param B
	 * @param K
	 */
	protected void checkPointwiseControlability(final RealMatrix A,
			final RealMatrix B) {
		final int numberOfStates = A.getRowDimension();
		final int numberOfColumns = numberOfStates * B.getColumnDimension();
		final int numberOfRows = B.getRowDimension();
		final RealMatrix controlabilityMatrix = MatrixUtils.createRealMatrix(
				numberOfRows, numberOfColumns);

		// B
		controlabilityMatrix.setSubMatrix(B.getData(), 0, 0);
		for (int i = 1; i < numberOfStates; i++) {
			// A^i*B
			controlabilityMatrix.setSubMatrix(
					(A.power(i).multiply(B)).getData(), 0,
					i * B.getColumnDimension());
		}

		SingularValueDecomposition svd = new SingularValueDecomposition(
				controlabilityMatrix);
		if (svd.getRank() < numberOfStates) {
			logger.warn(
					"CONTROLABILITY CHECK - The step is not completely controllable. Expected rank {} but found {}. ",
					numberOfStates, svd.getRank());
			throw new RuntimeException(
					"CONTROLABILITY CHECK - The step is not completely controllable.");
		}

		// For multi-input systems, although MC is a nonsquare
		// MCMCT is a square matrix, and thus the
		// abovementioned systematic procedure can be applied to
		// to guide the designer towards an appropriate choice
		SingularValueDecomposition svdD = new SingularValueDecomposition(
				controlabilityMatrix.multiply(controlabilityMatrix.transpose()));
		// determinant is the product of the matrix singular values
		detControllability = 1.0d;
		double[] singularValues = svdD.getSingularValues();
		for (double d : singularValues) {
			detControllability *= d;
		}
		conditionNumberControllability = svdD.getConditionNumber();
		if (logger.isTraceEnabled()) {
			logger.trace("Determinant of controlability matrix (CC^T): {}",
					detControllability);
		}
	}

	/**
	 * Check observability of the step.
	 * 
	 * @param A
	 * @param C
	 * @param K
	 */
	protected void checkPointwiseObservability(final RealMatrix A,
			final RealMatrix C) {
		final int numberOfStates = A.getRowDimension();
		final int numberOfColumns = numberOfStates;
		final int numberOfRows = C.getRowDimension() * numberOfStates;
		final RealMatrix observabilityMatrix = MatrixUtils.createRealMatrix(
				numberOfRows, numberOfColumns);

		// C
		observabilityMatrix.setSubMatrix(C.getData(), 0, 0);
		for (int i = 1; i < numberOfStates; i++) {
			// C*A^i
			observabilityMatrix.setSubMatrix(
					(C.multiply(A.power(i))).getData(),
					i * C.getRowDimension(), 0);
		}

		SingularValueDecomposition svd = new SingularValueDecomposition(
				observabilityMatrix);
		if (svd.getRank() < numberOfStates) {
			logger.warn(
					"OBSERVABILITY CHECK - The step is not completely observable. Expected rank {} but found {} ",
					numberOfStates, svd.getRank());
			throw new RuntimeException(
					"OBSERVABILITY CHECK - The step is not completely observable.");
		}
	}

	/**
	 * @return the detControllability
	 */
	public double getDetControllability() {
		return detControllability;
	}

	/**
	 * @return the detControllability
	 */
	public double getConditionNumberControllability() {
		return conditionNumberControllability;
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
		return null;
	}

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
		return null;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * br.inpe.cmc202.satellite.controllers.Controller#computeControlQuaternion
	 * (org.hipparchus.geometry.euclidean.threed.Rotation,
	 * org.hipparchus.geometry.euclidean.threed.Vector3D)
	 */
	public Vector3D computeControlQuaternion(Rotation errorQuaternion,
			Vector3D angularVelocity) {
		return null;
	}

}