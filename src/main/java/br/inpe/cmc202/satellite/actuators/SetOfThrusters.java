package br.inpe.cmc202.satellite.actuators;

import java.util.Properties;

import org.apache.commons.math3.util.Precision;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;

/**
 * The cold gas propulsion subsystem, available in the simulator, 
 * is modeled by tree thrusters pairs, one for each rotation: yaw, pitch and roll. 
 * The thrusters of a pair are mounted in the opposite faces of the satellite, 
 * moreover, they are positioned back to back so that the nozzles are able 
 * to generate symmetric torques.
 * 
 * @author alessandro.g.romero
 * 
 */
public class SetOfThrusters {
	
	/**
	 * EPSILON.
	 * 
	 * N.m
	 */	
	private final static double EPSILON = 0.01d; // the magnitude of the torquers

	/**
	 * Maximum torque.
	 * 
	 * N.m
	 */
	private final double MAX_TORQ;

	/**
	 * State
	 */
	private SetOfThrustersState state;
	
	/**
	 * @param satelliteConfiguration
	 */
	public SetOfThrusters(Properties satelliteConfiguration) {
		if (satelliteConfiguration != null) {
			this.MAX_TORQ = Double.valueOf(satelliteConfiguration.getProperty(
					"thurster.maxTorque", "0d"));
		} else {
			this.MAX_TORQ = 0d;
		}

		this.state = new SetOfThrustersState(Vector3D.ZERO, Vector3D.ZERO);
	}

	/**
	 * Actuate in to a given state based on the desired control torque.
	 * 
	 * @param controlTorque
	 *            N.m
	 * @return
	 */
	public SetOfThrustersState actuate(final Vector3D idealControlTorque) {

		Vector3D controlTorque = idealControlTorque;
		
		// ON OFF CONTROL
		// INPULSIVE CONTROL
		//
		// comparing if the control torque is above some epsilon
		// then turning ON the thrusters in that axis
		// else turning OFF the thruster in that axis
		if (Precision.compareTo(FastMath.abs(controlTorque.getX()), 0d, EPSILON) != 0d) {
			controlTorque = new Vector3D(FastMath.copySign(MAX_TORQ,controlTorque.getX()), 
					controlTorque.getY(), 
					controlTorque.getZ());
		} else {
			controlTorque = new Vector3D(0d, controlTorque.getY(), controlTorque.getZ());
		}
		if (Precision.compareTo(FastMath.abs(controlTorque.getY()), 0d, EPSILON) != 0d) {
			controlTorque = new Vector3D(controlTorque.getX(),
					FastMath.copySign(MAX_TORQ, controlTorque.getY()),
					controlTorque.getZ());
		} else {
			controlTorque = new Vector3D(controlTorque.getX(), 0d, controlTorque.getZ());
		}
		if (Precision.compareTo(FastMath.abs(controlTorque.getZ()), 0d, EPSILON) != 0d) {
			controlTorque = new Vector3D(controlTorque.getX(),
							controlTorque.getY(), 
							FastMath.copySign(MAX_TORQ, controlTorque.getZ()));
		} else {
			controlTorque = new Vector3D(controlTorque.getX(), controlTorque.getY(), 0d);
		}

		// updating instantaneous torque
		this.state = new SetOfThrustersState(controlTorque.scalarMultiply(-1d), idealControlTorque.scalarMultiply(-1d));
		return this.state;
	}

	/**
	 * @return the state
	 */
	public SetOfThrustersState getState() {
		return state;
	}

}
