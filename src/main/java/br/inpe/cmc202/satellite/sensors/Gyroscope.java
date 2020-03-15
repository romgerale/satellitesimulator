package br.inpe.cmc202.satellite.sensors;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.SpacecraftState;

/**
 * This class represents the gyroscopes.
 * 
 * @author alessandro.g.romero
 * 
 */
public class Gyroscope {

	/**
	 * Return the angular velocity.
	 * 
	 * @param currentState
	 * @return
	 * @throws OrekitException
	 */
	public Vector3D read(SpacecraftState currentState) throws OrekitException {
		return currentState.getAttitude().getSpin();
	}
}
