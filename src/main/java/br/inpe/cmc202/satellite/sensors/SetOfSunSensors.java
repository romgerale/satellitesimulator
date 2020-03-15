package br.inpe.cmc202.satellite.sensors;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.Transform;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.PVCoordinatesProvider;
import org.orekit.utils.TimeStampedPVCoordinates;

/**
 * This class simulates the measures of 3 sun sensors positioned in the
 * satellite, namely: sun sensor roll, sun sensor pitch, sun sensor yaw.
 * 
 * In this simplified exercise, the measures are constant.
 * 
 * @author alessandro.g.romero
 */
public class SetOfSunSensors {
	final PVCoordinatesProvider sunCoordinatesProvider;

	public SetOfSunSensors() {
		// SUN - INITIAL CONDITION - FIXED THROUGH SIMULATION
		// ----------------------------------------------
		sunCoordinatesProvider = new PVCoordinatesProvider() {
			private final Vector3D solarPosition_eci = new Vector3D(0.323116d,
					0.868285d, 0.376401d);

			public TimeStampedPVCoordinates getPVCoordinates(AbsoluteDate date,
					Frame frame) throws OrekitException {
				return new TimeStampedPVCoordinates(date, solarPosition_eci,
						Vector3D.ZERO, Vector3D.ZERO);
			}
		};
	}

	/**
	 * Read the sensors.
	 * 
	 * @param currentState
	 * @return
	 * @throws OrekitException
	 */
	public Vector3D read(SpacecraftState currentState) throws OrekitException {
		// rotation from eci to body
		Transform c_body_eci = currentState.toTransform();
		// current position in eci
		Vector3D sun_eci = sunCoordinatesProvider.getPVCoordinates(
				currentState.getDate(), currentState.getFrame()).getPosition();

		// alternative - more accurate
		// CelestialBody sun = CelestialBodyFactory.getSun();
		// Vector3D sun_eci = sun.getPVCoordinates(currentState.getDate(),
		// currentState.getFrame()).getPosition();

		// transform to body
		Vector3D sun_body = c_body_eci.transformVector(sun_eci);
		return sun_body;
	}
}
