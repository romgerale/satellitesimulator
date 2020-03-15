package br.inpe.cmc202.propagation.stateprovider;

import org.orekit.errors.OrekitException;
import org.orekit.propagation.AdditionalStateProvider;
import org.orekit.propagation.SpacecraftState;

import br.inpe.cmc202.satellite.Satellite;

/**
 * SetOfSunSensorsStateProvider.
 * 
 * @author alessandro.g.romero
 * 
 */
public class SetOfSunSensorsStateProvider implements AdditionalStateProvider {

	private final Satellite satellite;

	public SetOfSunSensorsStateProvider(Satellite satellite) {
		this.satellite = satellite;

	}

	public String getName() {
		return "setofsunsensors";
	}

	public double[] getAdditionalState(SpacecraftState state)
			throws OrekitException {

		return satellite.getSetOfSunSensors().read(state).toArray();
	}

}
