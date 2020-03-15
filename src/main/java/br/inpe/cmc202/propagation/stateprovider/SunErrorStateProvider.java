package br.inpe.cmc202.propagation.stateprovider;

import org.orekit.errors.OrekitException;
import org.orekit.propagation.AdditionalStateProvider;
import org.orekit.propagation.SpacecraftState;

import br.inpe.cmc202.satellite.Satellite;

/**
 * SunErrorStateProvider.
 * 
 * @author alessandro.g.romero
 * 
 */
public class SunErrorStateProvider implements AdditionalStateProvider {

	private final Satellite satellite;

	public SunErrorStateProvider(Satellite satellite) {
		this.satellite = satellite;

	}

	public String getName() {
		return "sunerror";
	}

	public double[] getAdditionalState(SpacecraftState state)
			throws OrekitException {
		return satellite.getSunError(state);
	}

}
