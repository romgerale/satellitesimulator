package br.inpe.cmc202.orbit;

import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.analysis.differentiation.UnivariateDifferentiableFunction;
import org.hipparchus.exception.MathIllegalArgumentException;

/**
 * NewtonRaphson method to solve the Kepler's equation.
 * 
 * @author alessandro.g.romero
 * 
 */
public class KeplerEquation implements UnivariateDifferentiableFunction {
	private double e;
	private double M;

	public KeplerEquation(double e, double M) {
		this.e = e;
		this.M = M;
	}

	public double value(double E) {
		throw new UnsupportedOperationException();
		// return E - e * FastMath.sin(E) - M;
	}

	public DerivativeStructure value(DerivativeStructure E)
			throws MathIllegalArgumentException {
		return E.subtract(E.sin().multiply(e)).subtract(M);
	}
}
