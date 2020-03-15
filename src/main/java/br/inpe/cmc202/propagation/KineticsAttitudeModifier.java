package br.inpe.cmc202.propagation;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.geometry.euclidean.threed.Vector3DComplement;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.RealVector;
import org.hipparchus.random.RandomDataGenerator;
import org.orekit.attitudes.Attitude;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.attitudes.AttitudeProviderModifier;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.PVCoordinatesProvider;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.satellite.Satellite;

/**
 * Kinetics propagation.
 * 
 * @author alessandro.g.romero
 * 
 */
public class KineticsAttitudeModifier implements AttitudeProviderModifier {
	/**
	 * 
	 */
	private static final long serialVersionUID = -3985396032871290639L;

	final private Logger logger = LoggerFactory
			.getLogger(KineticsAttitudeModifier.class);

	final private KinematicsAttitudeProvider underlyingAttitudeProvider;
	final private Satellite satellite;

	final private RandomDataGenerator externalTorqueGenerator;

	/**
	 * @param attitudeProvider
	 * @param satellite
	 */
	public KineticsAttitudeModifier(AttitudeProvider attitudeProvider,
			Satellite satellite) {
		if (!(attitudeProvider instanceof KinematicsAttitudeProvider)) {
			throw new UnsupportedOperationException(
					"It must work with a kinematic attitude");
		}
		if (satellite == null) {
			throw new UnsupportedOperationException(
					"It must work with a satellite");
		}

		this.underlyingAttitudeProvider = (KinematicsAttitudeProvider) attitudeProvider;
		this.satellite = satellite;

		if (this.satellite.getExternalTorquesMagnitude() != 0) {
			externalTorqueGenerator = new RandomDataGenerator();
		} else {
			externalTorqueGenerator = null;
		}

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

		final Attitude previousAttitude = underlyingAttitudeProvider
				.getPreviousAttitude();

		final double dt = date.durationFrom(previousAttitude.getDate());

		if (dt <= 0) {
			return previousAttitude;
		}

		// kinetics propagation
		final Vector3DComplement previousAngularVelocity_body = new Vector3DComplement(
				previousAttitude.getSpin().toArray());
		final Vector3DComplement previousAngularAcceleration_body = new Vector3DComplement(
				previousAttitude.getRotationAcceleration().toArray());

		// external torque
		RealVector externalTorque = new ArrayRealVector(3, 0);
		if (externalTorqueGenerator != null) {
			externalTorque = new ArrayRealVector(new double[] {
					externalTorqueGenerator.nextNormal(satellite.getExternalTorquesMagnitude(),1d),
					externalTorqueGenerator.nextNormal(satellite.getExternalTorquesMagnitude(),1d),
					externalTorqueGenerator.nextNormal(satellite.getExternalTorquesMagnitude(),1d) });
		}

		// magnetorquer
		// and external torques in the center of mass
		// I_b^{-1}g_{cm}
		RealVector firstComponent = new ArrayRealVector(3, 0);
		if (satellite.getSetOfMagnetorquer() != null) {
			firstComponent = satellite.getI_inverse().operate(
					new ArrayRealVector(satellite.getSetOfMagnetorquer()
							.getState().getControlTorque().toArray())
							.add(externalTorque));
		}

		// rigid body
		// - I_b^{-1}\omega^{\times}I_b\omega
		final RealVector secondComponent = satellite
				.getI_inverse()
				.multiply(previousAngularVelocity_body.getTimesMatrix())
				.multiply(satellite.getI())
				.operate(
						new ArrayRealVector(previousAngularVelocity_body
								.toArray())).mapMultiply(-1);

		// reaction wheel
		RealVector thirdComponent = new ArrayRealVector(3, 0);
		RealVector fourthComponent = new ArrayRealVector(3, 0);
		if (satellite.getSetOfReactionWheels() != null) {
			// - I_b^{-1}\omega^{\times}\sum_{n=1}^{N}h_{w,n}a_n
			thirdComponent = satellite
					.getI_inverse()
					.multiply(previousAngularVelocity_body.getTimesMatrix())
					.operate(
							new ArrayRealVector(satellite
									.getSetOfReactionWheels()
									.getState()
									.getAngularMomentum(
											previousAngularVelocity_body)
									.toArray())).mapMultiply(-1);
			// - I_b^{-1}\sum_{n=1}^{N}\dot{h}_{w,n} a_n
			fourthComponent = satellite
					.getI_inverse()
					.operate(
							new ArrayRealVector(satellite
									.getSetOfReactionWheels().getState()
									.getControlTorque().toArray()))
					.mapMultiply(-1);
		}

		// computed derivative of angular velocity
		RealVector angularAcceleration = firstComponent.add(secondComponent)
				.add(thirdComponent).add(fourthComponent);

		// change in the angular acceleration (constant through dt)
		final Vector3D updated_angularAcceleration = previousAngularAcceleration_body
				.add(new Vector3D(angularAcceleration.mapMultiply(dt).toArray()));

		// change in the angular velocity (constant through dt)
		final Vector3D updated_angularVelocity = previousAngularVelocity_body
				.add(new Vector3D(angularAcceleration.mapMultiply(dt).toArray()));

		logger.trace(
				"Instantaneous angular velocity {} and angular acceleration {} using dt {} at t {}",
				updated_angularVelocity, updated_angularAcceleration, dt, date);

		//
		// propagating kinematics
		Attitude attitudePropagatedByKinematics = underlyingAttitudeProvider
				.getAttitude(pvProv, date, frame);
		//
		// updating velocity using the results of kinetics propagation
		final Attitude attitudePropagatedByKinematicsAndKinetics = new Attitude(
				date, frame, attitudePropagatedByKinematics.getRotation(),
				updated_angularVelocity, updated_angularAcceleration);

		// updating for next propagation
		underlyingAttitudeProvider
				.setPreviousAttitude(attitudePropagatedByKinematicsAndKinetics);
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

	/**
	 * @return the previousAttitude
	 */
	protected Attitude getPreviousAttitude() {
		return underlyingAttitudeProvider.getPreviousAttitude();
	}

}