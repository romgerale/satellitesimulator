package br.inpe.cmc202.satellite.controllers.sdre;

import java.text.DecimalFormat;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.geometry.euclidean.threed.Vector3DComplement;
import org.hipparchus.linear.CholeskyDecomposition;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealMatrixFormat;

import br.inpe.cmc202.satellite.Satellite;
import br.inpe.cmc202.satellite.controllers.BaseController;

/**
 * Base class for SDRE CONTROL
 * 
 * @author alessandro.g.romero
 * 
 */
public abstract class ProportionalNonLinearBaseSDREController extends
		BaseController {

	static protected final RealMatrixFormat f = new RealMatrixFormat(
			"",
			"",
			"\n",
			"",
			"",
			"\t\t",
			new DecimalFormat(
					" ###########0000.00000000000;-###########0000.00000000000"));

	protected RealMatrix R = MatrixUtils.createRealIdentityMatrix(3);
	protected RealMatrix Q = MatrixUtils.createRealIdentityMatrix(6);
	protected RealMatrix Q_sqrt;
	protected RealMatrix C = MatrixUtils.createRealIdentityMatrix(6);
	protected RealMatrix B;

	protected final Satellite satellite;

	/**
	 * Constructor.
	 * 
	 * @param satellite
	 */
	public ProportionalNonLinearBaseSDREController(final Satellite satellite) {
		this.satellite = satellite;

		// B
		B = MatrixUtils.createRealMatrix(6, 3);
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

		// Q_sqrt
		// it is an identity, but this works for the general case
		// square root of the Q computed by CholeskyDecomposition
		Q_sqrt = new CholeskyDecomposition(Q).getL();
	}

	/**
	 * Compute the part of A related to kinetics.
	 * 
	 * @param angularVelocity
	 * @return
	 */
	protected RealMatrix computeAKinetics(final Vector3D angularVelocity) {
		// omega times
		final Vector3DComplement angularVelocity2times = new Vector3DComplement(
				angularVelocity.toArray());
		// reaction wheel angular momentum times
		final Vector3DComplement reactionWheelAngularMomentum2times = new Vector3DComplement(
				satellite.getSetOfReactionWheels().getState()
						.getAngularMomentum(angularVelocity).toArray());

		// A_angularVelocity
		// - I_b^{-1}\omega^{\times}I_b
		// + I_b^{-1}(\sum_{n=1}^{3}h_{w,n}a_n)^{\times}
		final RealMatrix A_angularVelocity_firstComponent = satellite
				.getI_inverse_nominal()
				.multiply(angularVelocity2times.getTimesMatrix())
				.multiply(satellite.getI_nominal()).scalarMultiply(-1);
		final RealMatrix A_angularVelocity_secondComponent = satellite
				.getI_inverse_nominal().multiply(
						reactionWheelAngularMomentum2times.getTimesMatrix());
		final RealMatrix A_angularVelocity = A_angularVelocity_firstComponent
				.add(A_angularVelocity_secondComponent);
		return A_angularVelocity;
	}

}
