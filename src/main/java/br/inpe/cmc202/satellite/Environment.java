package br.inpe.cmc202.satellite;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.RealVector;
import org.hipparchus.random.RandomDataGenerator;

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
	private static Map<Double, List<RealVector>> externalTorques;

	/**
	 * It controls the usage of procomputed data.
	 */
	private int currentIndexExternalTorque = 0;

	private final static int MAX_DATA = 80000;

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
	private synchronized void prepareExternalTorques(double externalTorquesMagnitude) {
		if (externalTorques == null) {
			externalTorques = new HashMap<Double, List<RealVector>>();
		}
		if (externalTorquesMagnitude != 0 && externalTorques.get(externalTorquesMagnitude) == null) {
			List<RealVector> iExternalTorques = new ArrayList<RealVector>();
			externalTorques.put(externalTorquesMagnitude, iExternalTorques);
			RandomDataGenerator externalTorqueGeneratorX = new RandomDataGenerator();
			RandomDataGenerator externalTorqueGeneratorY = new RandomDataGenerator();
			RandomDataGenerator externalTorqueGeneratorZ = new RandomDataGenerator();

			// computing points
			for (int i = 0; i < MAX_DATA; i++) {
				RealVector externalTorque = new ArrayRealVector(new double[] {
						// normal external torque of max magnitude given by 'external Torques Magnitude'
						// externalTorqueGeneratorX.nextNormal(0d,1d) *
						// satellite.getExternalTorquesMagnitude(),
						// externalTorqueGeneratorY.nextNormal(0d,1d) *
						// satellite.getExternalTorquesMagnitude(),
						// externalTorqueGeneratorZ.nextNormal(0d,1d) *
						// satellite.getExternalTorquesMagnitude()});

						// it is based on a Additive White Gaussian Noise (AWGN)
						// 0.33 to allow at max 1 externalTorqueMagnitude
						externalTorqueGeneratorX.nextNormal(0d, 0.33d) * externalTorquesMagnitude,
						externalTorqueGeneratorY.nextNormal(0d, 0.33d) * externalTorquesMagnitude,
						externalTorqueGeneratorZ.nextNormal(0d, 0.33d) * externalTorquesMagnitude });

				// applying equally, in the three axes, the 'external Torques Magnitude'
				// satellite.getExternalTorquesMagnitude(),
				// satellite.getExternalTorquesMagnitude(),
				// satellite.getExternalTorquesMagnitude()});
				iExternalTorques.add(externalTorque);
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
		if (externalTorques.get(externalTorquesMagnitude) == null) {
			toRet = new ArrayRealVector(3, 0);
		} else {
			List<RealVector> iExternalTorques = externalTorques.get(externalTorquesMagnitude);
			if (currentIndexExternalTorque >= iExternalTorques.size()) {
				throw new RuntimeException(
						"There is external torques computed, however, the simulation time is too big for the computed data!");
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
		if (externalTorques.get(externalTorquesMagnitude) == null) {
			toRet = new ArrayRealVector(3, 0);
		} else {
			List<RealVector> iExternalTorques = externalTorques.get(externalTorquesMagnitude);
			if (currentIndexExternalTorque >= iExternalTorques.size()) {
				throw new RuntimeException(
						"There is external torques computed, however, the simulation time is too big for the computed data!");
			}
			toRet = iExternalTorques.get(currentIndexExternalTorque);
		}
		return toRet;
	}

}
