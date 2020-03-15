package br.inpe.cmc202.propagation.stateprovider;

import org.orekit.errors.OrekitException;
import org.orekit.propagation.AdditionalStateProvider;
import org.orekit.propagation.SpacecraftState;

import br.inpe.cmc202.satellite.Satellite;

/**
 * SatelliteUpdaterStateProvider. The last state provider that updates the
 * Satelite.
 * 
 * @author alessandro.g.romero
 * 
 */
public class SatelliteUpdaterStateProvider implements AdditionalStateProvider {

	private final Satellite satellite;

	public SatelliteUpdaterStateProvider(Satellite satellite) {
		this.satellite = satellite;

	}

	public String getName() {
		return "satelliteupdater";
	}

	public double[] getAdditionalState(SpacecraftState state)
			throws OrekitException {
		satellite.setSpacecraftState(state);
		return new double[] {};
	}

}
