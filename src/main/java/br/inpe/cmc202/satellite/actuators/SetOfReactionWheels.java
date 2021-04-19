package br.inpe.cmc202.satellite.actuators;

import java.util.Properties;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.util.FastMath;

/**
 * Set of three Reaction Wheels installed in the satellite in each axis.
 * 
 * @author alessandro.g.romero
 * 
 */
public class SetOfReactionWheels {

	/**
	 * Maximum torque.
	 * 
	 * N.m
	 */
	private final double MAX_TORQ;

	/**
	 * Inertia from each reaction wheel.
	 * 
	 * kg.m^2
	 */
	protected final double INERTIA;

	/**
	 * Maximum angular velocity.
	 * 
	 * RPM => RPM * PI /30 radians/second
	 */
	private final double MAX_ANGULAR_VELOCITY;

	/**
	 * Approximated maximum angular acceleration from each reaction wheel.
	 * 
	 * radians/s^2
	 */
	private final double MAX_ANGULAR_ACCELERATION_APPROX;

	/**
	 * 
	 */
	private final RealMatrix INERTIA_TENSOR_CONTRIBUTION;

	/**
	 * State
	 */
	private SetOfReactionWheelsState state;

	/**
	 * @param satelliteConfiguration
	 */
	public SetOfReactionWheels(Properties satelliteConfiguration) {
		if (satelliteConfiguration != null) {
			this.MAX_TORQ = Double.valueOf(satelliteConfiguration.getProperty(
					"reactionWheel.maxTorque", "0.075d"));
			this.INERTIA = Double.valueOf(satelliteConfiguration.getProperty(
					"reactionWheel.inertia", "0.01911d"));
			this.MAX_ANGULAR_VELOCITY = Double.valueOf(satelliteConfiguration
					.getProperty("reactionWheel.maxAngularVelocity", "6000"))
					* FastMath.PI / 30;
		} else {
			this.MAX_TORQ = 0.075d;
			this.INERTIA = 0.01911d;
			this.MAX_ANGULAR_VELOCITY = 6000 * FastMath.PI / 30;
		}

		this.MAX_ANGULAR_ACCELERATION_APPROX = MAX_TORQ / INERTIA;
		this.INERTIA_TENSOR_CONTRIBUTION = MatrixUtils
				.createRealMatrix(new double[][] { { INERTIA, 0, 0 },
						{ 0, INERTIA, 0 }, { 0, 0, INERTIA } });
		this.state = new SetOfReactionWheelsState(Vector3D.ZERO, Vector3D.ZERO,
				INERTIA);
	}

	/**
	 * Actuate in to a given state based on the desired control torque and the
	 * timeshift.
	 * 
	 * @param controlTorque
	 *            N.m
	 * @param dt
	 *            s
	 * @param state
	 *            previousState - important for velocity (integrated)
	 * @return
	 */
	public SetOfReactionWheelsState actuate(Vector3D controlTorque, double dt,
			SetOfReactionWheelsState state) {

		// check limits of torques
		if (FastMath.abs(controlTorque.getX()) > MAX_TORQ) {
			controlTorque = new Vector3D(FastMath.signum(controlTorque.getX())
					* MAX_TORQ, controlTorque.getY(), controlTorque.getZ());
		}
		if (FastMath.abs(controlTorque.getY()) > MAX_TORQ) {
			controlTorque = new Vector3D(controlTorque.getX(),
					FastMath.signum(controlTorque.getY()) * MAX_TORQ,
					controlTorque.getZ());
		}
		if (FastMath.abs(controlTorque.getZ()) > MAX_TORQ) {
			controlTorque = new Vector3D(controlTorque.getX(),
					controlTorque.getY(), FastMath.signum(controlTorque.getZ())
							* MAX_TORQ);
		}

		// computing angular acceleration
		// \dot{\omega} = T/I
		Vector3D angularAcceleration = controlTorque
				.scalarMultiply(1 / INERTIA);

		// computing desired angular velocity
		Vector3D angularVelocity = state.getAngularVelocity().add(
				angularAcceleration.scalarMultiply(dt));

		// check saturation in the angular velocity,and change the
		// torque accordingly
		if (FastMath.abs(angularVelocity.getX()) > MAX_ANGULAR_VELOCITY) {
			controlTorque = new Vector3D(FastMath.signum(controlTorque.getX())
					* (MAX_ANGULAR_VELOCITY - FastMath.abs(state
							.getAngularVelocity().getX())) * INERTIA / dt,
					controlTorque.getY(), controlTorque.getZ());
			angularVelocity = new Vector3D(MAX_ANGULAR_VELOCITY
					* FastMath.signum(angularVelocity.getX()),
					angularVelocity.getY(), angularVelocity.getZ());
		}
		if (FastMath.abs(angularVelocity.getY()) > MAX_ANGULAR_VELOCITY) {
			controlTorque = new Vector3D(controlTorque.getX(),
					FastMath.signum(controlTorque.getY())
							* (MAX_ANGULAR_VELOCITY - FastMath.abs(state
									.getAngularVelocity().getY())) * INERTIA
							/ dt, controlTorque.getZ());
			angularVelocity = new Vector3D(angularVelocity.getX(),
					MAX_ANGULAR_VELOCITY
							* FastMath.signum(angularVelocity.getY()),
					angularVelocity.getZ());
		}
		if (FastMath.abs(angularVelocity.getZ()) > MAX_ANGULAR_VELOCITY) {
			controlTorque = new Vector3D(controlTorque.getX(),
					controlTorque.getY(), FastMath.signum(controlTorque.getZ())
							* (MAX_ANGULAR_VELOCITY - FastMath.abs(state
									.getAngularVelocity().getZ())) * INERTIA
							/ dt);
			angularVelocity = new Vector3D(angularVelocity.getX(),
					angularVelocity.getY(), MAX_ANGULAR_VELOCITY
							* FastMath.signum(angularVelocity.getZ()));
		}

		// checking continuity
		// changes in the norm of the angular velocity should be continuous (at
		// most max acceleration in each axis)
		// TODO changed from 3 to 4
		if (FastMath.abs(state.getAngularVelocity().getNorm())
				- FastMath.abs(angularVelocity.getNorm()) > (FastMath.sqrt(4
				* MAX_ANGULAR_ACCELERATION_APPROX
				* MAX_ANGULAR_ACCELERATION_APPROX) * dt)) {
			throw new RuntimeException(
					"Error in the processing of angular velocity");
		}

		// updating instantaneous torque and the angular velocity
		this.state = new SetOfReactionWheelsState(controlTorque,
				angularVelocity, INERTIA);
		return this.state;
	}

	/**
	 * @return
	 */
	public RealMatrix getInertiaTensorContribution() {
		return INERTIA_TENSOR_CONTRIBUTION;

	}

	/**
	 * @return the state
	 */
	public SetOfReactionWheelsState getState() {
		return state;
	}

	/**
	 * @return the maximum angular velocity in radians/s
	 */	
	public double getMAX_ANGULAR_VELOCITY() {
		return MAX_ANGULAR_VELOCITY;
	}

	/**
	 * @return the maximum torque in Newton.meter (N.m)
	 */	
	public double getMAX_TORQ() {
		return MAX_TORQ;
	}

}
