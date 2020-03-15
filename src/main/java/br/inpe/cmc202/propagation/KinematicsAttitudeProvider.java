package br.inpe.cmc202.propagation;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.orekit.attitudes.Attitude;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.AngularCoordinates;
import org.orekit.utils.PVCoordinatesProvider;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.satellite.Satellite;

/**
 * Kinematics propagation.
 * 
 * @author alessandro.g.romero
 * 
 */
public class KinematicsAttitudeProvider implements AttitudeProvider {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	final private Logger logger = LoggerFactory
			.getLogger(KinematicsAttitudeProvider.class);

	private Attitude previousAttitude;

	public KinematicsAttitudeProvider(final Attitude initialAttitude,
			final Satellite satellite) {
		this.previousAttitude = initialAttitude;
	}

	/** {@inheritDoc} */
	public Attitude getAttitude(final PVCoordinatesProvider pvProv,
			final AbsoluteDate date, final Frame frame) throws OrekitException {
		final double dt = date.durationFrom(previousAttitude.getDate());

		if (dt <= 0) {
			return this.previousAttitude.withReferenceFrame(frame);
		}

		// kinematic propagation
		// final Attitude shifted = previousAttitude.shiftedBy(dt);

		//
		// quaternion kinematics propagation using \Omega

		// computing derivative of quaternion
		final Vector3D angularAcceleration = previousAttitude
				.getRotationAcceleration();
		final Vector3D angularVelocity = previousAttitude.getSpin();
		final RealMatrix omega = MatrixUtils.createRealMatrix(4, 4);
		omega.setEntry(0, 0, 0);
		omega.setEntry(0, 1, angularVelocity.getZ());
		omega.setEntry(0, 2, -angularVelocity.getY());
		omega.setEntry(0, 3, angularVelocity.getX());

		omega.setEntry(1, 0, -angularVelocity.getZ());
		omega.setEntry(1, 1, 0);
		omega.setEntry(1, 2, angularVelocity.getX());
		omega.setEntry(1, 3, angularVelocity.getY());

		omega.setEntry(2, 0, angularVelocity.getY());
		omega.setEntry(2, 1, -angularVelocity.getX());
		omega.setEntry(2, 2, 0);
		omega.setEntry(2, 3, angularVelocity.getZ());

		omega.setEntry(3, 0, -angularVelocity.getX());
		omega.setEntry(3, 1, -angularVelocity.getY());
		omega.setEntry(3, 2, -angularVelocity.getZ());
		omega.setEntry(3, 3, 0);

		// previous quaternion
		// NOTE: derivative equation takes the scalar element as last
		final RealVector q = new ArrayRealVector(new double[] {
				previousAttitude.getRotation().getQ1(),
				previousAttitude.getRotation().getQ2(),
				previousAttitude.getRotation().getQ3(),
				previousAttitude.getRotation().getQ0() });

		// computed derivative of quaternion
		final RealVector qt = omega.scalarMultiply(.5d).operate(q);

		// changed quaternion
		final RealVector updated_q = q.add(qt.mapMultiply(dt));
		final Rotation updatedRotation = new Rotation(updated_q.getEntry(3),
				updated_q.getEntry(0), updated_q.getEntry(1),
				updated_q.getEntry(2), true);
		final RealVector updated_qq = new ArrayRealVector(new double[] {
				updatedRotation.getQ1(), updatedRotation.getQ2(),
				updatedRotation.getQ3(), updatedRotation.getQ0() });

		// updating the attitude
		// NOTE: derivative equation takes the scalar element as last
		final AngularCoordinates sac = new AngularCoordinates(updatedRotation,
				angularVelocity, angularAcceleration);
		final TimeStampedAngularCoordinates tsac = new TimeStampedAngularCoordinates(
				date.shiftedBy(dt), sac.getRotation(), sac.getRotationRate(),
				sac.getRotationAcceleration());
		final Attitude shifted = new Attitude(frame, tsac);

		logger.trace("Instantaneous quaternion {} using dt {} at t {}",
				updated_qq, dt, date);

		this.previousAttitude = shifted;

		return this.previousAttitude.withReferenceFrame(frame);
	}

	/**
	 * @return the previousAttitude
	 */
	protected Attitude getPreviousAttitude() {
		return previousAttitude;
	}

	/**
	 * @param previousAttitude
	 *            the previousAttitude to set
	 */
	protected void setPreviousAttitude(Attitude previousAttitude) {
		this.previousAttitude = previousAttitude;
	}

}
