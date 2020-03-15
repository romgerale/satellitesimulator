package org.hipparchus.geometry.euclidean.threed;

import org.hipparchus.exception.MathIllegalArgumentException;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;

/**
 * Vector3DComplement. It provides the matrix for cross product.
 * 
 * @author alessandro.g.romero
 * 
 */
public class Vector3DComplement extends
		org.hipparchus.geometry.euclidean.threed.Vector3D {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * Constructor.
	 * 
	 * @param v
	 * @throws MathIllegalArgumentException
	 */
	public Vector3DComplement(double[] v) throws MathIllegalArgumentException {
		super(v);
	}

	/**
	 * Cross product matrix for a given vector.
	 * 
	 * @return
	 */
	public RealMatrix getTimesMatrix() {
		return MatrixUtils.createRealMatrix(new double[][] {
				{ 0, (this.getZ() != 0) ? -this.getZ() : 0, this.getY() },
				{ this.getZ(), 0, (this.getX() != 0) ? -this.getX() : 0 },
				{ (this.getY() != 0) ? -this.getY() : 0, this.getX(), 0 } });

	}

}
