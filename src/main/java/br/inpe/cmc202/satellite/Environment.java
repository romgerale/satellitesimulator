package br.inpe.cmc202.satellite;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.RealVector;
import org.hipparchus.random.RandomDataGenerator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.simulation.plotter.Plotter;

/**
 * 
 * Class that helps in the external torques computing and sharing between
 * simulations for the same external torque magnitude.
 * 
 * 
 * @author alessandro.g.romero
 * 
 */
public class Environment {

	/**
	 * It is used to share the same data for all satellites with a given
	 * externalTorqueMagnitude.
	 */
	private static final Map<Double, List<RealVector>> externalTorques = new HashMap<Double, List<RealVector>>();

	/**
	 * It controls the usage of procomputed data.
	 */
	private int currentIndexExternalTorque = 0;

	private final static int MAX_DATA = 80000;

	final private Logger logger = LoggerFactory.getLogger(Environment.class);

	// it is based on a Additive White Gaussian Noise (AWGN)
	// selecting sigma to strive for max 1 externalTorqueMagnitude
	// Sigma	Chance to have more than 1 externalTorqueMagnitude
	// 1		69%	
	// 2		31%	
	// 3		6.7%	
	// 4		0.62%	
	// 5		0.023%	
	// 6		0.00034%	
	// 7		0.0000019%	
	final private static double SIGMA = 7d;

	/**
	 * Constructor for Environment.
	 * 
	 * @param externalTorquesMagnitude
	 */
	public Environment(double externalTorquesMagnitude) {
		prepareExternalTorques(externalTorquesMagnitude);
	}

	/**
	 * Prepare the external torques for the satellites. It prepares MAX_DATA real
	 * vectors.
	 */
	private void prepareExternalTorques(double externalTorquesMagnitude) {
		synchronized(externalTorques) {
			if (externalTorquesMagnitude != 0 && externalTorques.get(externalTorquesMagnitude) == null) {
				logger.error("Computing external torque for magnitude {} ...", externalTorquesMagnitude);
				List<RealVector> iExternalTorques = new ArrayList<RealVector>();
				externalTorques.put(externalTorquesMagnitude, iExternalTorques);
				RandomDataGenerator externalTorqueGeneratorX = new RandomDataGenerator();
				RandomDataGenerator externalTorqueGeneratorY = new RandomDataGenerator();
				RandomDataGenerator externalTorqueGeneratorZ = new RandomDataGenerator();

				// for inspection of perturbed torque
				final Map<Double, double[]> torques = new HashMap<Double, double[]>();

				// computing points
				for (int i = 0; i < MAX_DATA; i++) {
					RealVector externalTorque = new ArrayRealVector(new double[] {
							externalTorqueGeneratorX.nextNormal(0d, externalTorquesMagnitude / SIGMA),
							externalTorqueGeneratorY.nextNormal(0d, externalTorquesMagnitude / SIGMA),
							externalTorqueGeneratorZ.nextNormal(0d, externalTorquesMagnitude / SIGMA)});
	
					iExternalTorques.add(externalTorque);
					torques.put(Double.valueOf(i), externalTorque.toArray());
					
				}

				logger.error("Computed external torque for magnitude {} using {} sigmas N(0,{})!", externalTorquesMagnitude, SIGMA, externalTorquesMagnitude / SIGMA);

				// plotting external torque
				Plotter.plot3DScatter(torques, "externalTorques for p=" + externalTorquesMagnitude);
			}
		}
	}

	/**
	 * Returns the precomputed external torque and it increments the counter for the
	 * next iteration.
	 * 
	 * @return the externalTorque vector
	 */
	public RealVector getCurrentAndPrepareNextExternalTorque(double externalTorquesMagnitude) {
		RealVector toRet = null;
		List<RealVector> iExternalTorques = externalTorques.get(externalTorquesMagnitude);
		if (iExternalTorques == null) {
			toRet = new ArrayRealVector(3, 0);
		} else {
			if (currentIndexExternalTorque >= iExternalTorques.size()) {
				throw new RuntimeException(
						"There are external torques computed, however, the simulation time is too big for the computed data!");
			}
			toRet = iExternalTorques.get(currentIndexExternalTorque);
			currentIndexExternalTorque++;
		}
		return toRet;
	}

	/**
	 * Return the precomputed external torque for current iteration.
	 * 
	 * @return the externalTorque vector
	 */
	public RealVector getCurrentExternalTorque(double externalTorquesMagnitude) {
		RealVector toRet = null;
		List<RealVector> iExternalTorques = externalTorques.get(externalTorquesMagnitude);
		if (iExternalTorques == null) {
			toRet = new ArrayRealVector(3, 0);
		} else {
			if (currentIndexExternalTorque >= iExternalTorques.size()) {
				throw new RuntimeException(
						"There are external torques computed, however, the simulation time is too big for the computed data!");
			}
			toRet = iExternalTorques.get(currentIndexExternalTorque);
		}
		return toRet;
	}

}
