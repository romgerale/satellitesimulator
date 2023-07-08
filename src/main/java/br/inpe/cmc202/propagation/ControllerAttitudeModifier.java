package br.inpe.cmc202.propagation;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.RotationOrder;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.geometry.euclidean.threed.Vector3DComplement;
import org.orekit.attitudes.Attitude;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.attitudes.AttitudeProviderModifier;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.PVCoordinatesProvider;

import br.inpe.cmc202.satellite.Satellite;
import br.inpe.cmc202.satellite.actuators.SetOfReactionWheelsState;

/**
 * Controller Atitude Modifier.
 * 
 * @author alessandro.g.romero
 * 
 */
public class ControllerAttitudeModifier implements AttitudeProviderModifier {
	/**
	 * 
	 */
	private static final long serialVersionUID = -3985396032871290639L;

	final private KineticsAttitudeModifier underlyingAttitudeProvider;
	final private Satellite satellite;

	/**
	 * @param attitudeProvider
	 * @param satellite
	 */
	public ControllerAttitudeModifier(AttitudeProvider attitudeProvider,
			Satellite satellite) {

		if (!(attitudeProvider instanceof KineticsAttitudeModifier)) {
			throw new UnsupportedOperationException(
					"It must work with a kinetic attitude");
		}
		if (satellite == null) {
			throw new UnsupportedOperationException(
					"It must work with a satellite");
		}

		this.underlyingAttitudeProvider = (KineticsAttitudeModifier) attitudeProvider;
		this.satellite = satellite;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.orekit.attitudes.AttitudeProvider#getAttitude(org.orekit.utils.
	 * PVCoordinatesProvider, org.orekit.time.AbsoluteDate,
	 * org.orekit.frames.Frame)
	 */
	public Attitude getAttitude(PVCoordinatesProvider pvProv,
			AbsoluteDate date, Frame frame) throws OrekitException {

		Attitude previousAttitude = underlyingAttitudeProvider
				.getPreviousAttitude();

		final double dt = date.durationFrom(previousAttitude.getDate());
		if (dt <= 0) {
			return previousAttitude;
		}

		// recomputing quaternion
		double[] errorQuaternionArray = satellite.getSpacecraftState()
				.getAdditionalState("sunerror");
		Rotation errorQuaternion = new Rotation(errorQuaternionArray[0],
				errorQuaternionArray[1], errorQuaternionArray[2],
				errorQuaternionArray[3], false);

		// saving state
		final Vector3DComplement previousAngularVelocity_body = new Vector3DComplement(
				previousAttitude.getSpin().toArray());
		final Vector3D previousSunError_body = new Vector3D(
				errorQuaternion.getAngles(RotationOrder.ZYX,
						RotationConvention.VECTOR_OPERATOR));
		final Vector3D previousSunSensor_body = new Vector3D(satellite
				.getSpacecraftState().getAdditionalState("setofsunsensors"));

		SetOfReactionWheelsState previousSetOfReactionWheelsState = null;
		if (satellite.getSetOfReactionWheels() != null) {
			previousSetOfReactionWheelsState = satellite
					.getSetOfReactionWheels().getState();
		}
		Vector3D lastMagnetometerRead = null;
		if (satellite.getSetOfMagnetorquerController() != null) {
			lastMagnetometerRead = new Vector3D(satellite.getSpacecraftState()
					.getAdditionalState("magnetometer"));
		}

		// calculate controlTorque
		// Euler Angles
		Vector3D controlTorque = satellite.getController()
				.computeControlEulerAngles(previousSunError_body,
						previousAngularVelocity_body);
		if (controlTorque == null) {
			// sun vector
			controlTorque = satellite.getController().computeControlSunVector(
					previousSunSensor_body, previousAngularVelocity_body);
		}
		if (controlTorque == null) {
			// quaternions
			controlTorque = satellite.getController().computeControlQuaternion(
					errorQuaternion, previousAngularVelocity_body);
		}
		if (controlTorque == null) {
			throw new RuntimeException(
					"Control torque for ReactionWheels is undefined!");
		}

		// actuate MAGNETORQUER
		if (satellite.getSetOfMagnetorquerController() != null) {
			// compute control torque of magnetometers
			final Vector3D controlTorqueM = satellite
					.getSetOfMagnetorquerController()
					.computeControl(
							previousSetOfReactionWheelsState
									.getAngularMomentum(previousAngularVelocity_body));

			if (controlTorqueM == null) {
				throw new RuntimeException(
						"Control torque for Magnetorques is undefined!");
			}

			// actuate magnetorquer
			if (satellite.getSetOfMagnetorquer() != null) {
				satellite.getSetOfMagnetorquer().actuate(lastMagnetometerRead,
						controlTorqueM);
			}
		}

		// actuate REACTIONWHEELS
		if (satellite.getSetOfReactionWheels() != null) {
			satellite.getSetOfReactionWheels().actuate(controlTorque, dt,
					previousSetOfReactionWheelsState);
		}

		// actuate THRUSTERS
		if (satellite.getSetOfThrusters() != null) {
			satellite.getSetOfThrusters().actuate(controlTorque);
		}

		// propagate
		// KINETICS AND KINEMATICS
		Attitude attitudePropagatedByKinematicsAndKinetics = underlyingAttitudeProvider
				.getAttitude(pvProv, date, frame);

		return attitudePropagatedByKinematicsAndKinetics;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * org.orekit.attitudes.AttitudeProviderModifier#getUnderlyingAttitudeProvider
	 * ()
	 */
	public AttitudeProvider getUnderlyingAttitudeProvider() {
		return underlyingAttitudeProvider;
	}

}
