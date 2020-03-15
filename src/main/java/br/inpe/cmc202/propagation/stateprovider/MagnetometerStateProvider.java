package br.inpe.cmc202.propagation.stateprovider;

import org.orekit.errors.OrekitException;
import org.orekit.propagation.AdditionalStateProvider;
import org.orekit.propagation.SpacecraftState;

import br.inpe.cmc202.satellite.Satellite;

/**
 * MagnetometerStateProvider.
 * 
 * @author alessandro.g.romero
 * 
 */
public class MagnetometerStateProvider implements AdditionalStateProvider {

	private final Satellite satellite;

	public MagnetometerStateProvider(Satellite satellite) {
		this.satellite = satellite;
	}

	public String getName() {
		return "magnetometer";
	}

	public double[] getAdditionalState(SpacecraftState state)
			throws OrekitException {

		return satellite.getMagnetometer().read(state).toArray();
	}

}
