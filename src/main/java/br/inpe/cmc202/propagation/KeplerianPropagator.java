package br.inpe.cmc202.propagation;

import org.orekit.attitudes.AttitudeProvider;
import org.orekit.errors.OrekitException;
import org.orekit.orbits.Orbit;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.AbstractAnalyticalPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.TimeSpanMap;

/**
 * Orbit Propagator.
 * 
 * @author alessandro.g.romero
 * 
 */
public class KeplerianPropagator extends AbstractAnalyticalPropagator {

	/** Initial state. */
	private SpacecraftState initialState;

	/** All states. */
	private transient TimeSpanMap<SpacecraftState> states;

	public KeplerianPropagator(final Orbit initialOrbit,
			AttitudeProvider attitudeProvider) throws OrekitException {
		super(attitudeProvider);
		resetInitialState(new SpacecraftState(initialOrbit,
				getAttitudeProvider().getAttitude(initialOrbit,
						initialOrbit.getDate(), initialOrbit.getFrame()), 0L));
	}

	public void resetInitialState(final SpacecraftState state)
			throws OrekitException {
		super.resetInitialState(state);
		initialState = state;
		states = new TimeSpanMap<SpacecraftState>(initialState);
	}

	@Override
	protected Orbit propagateOrbit(AbsoluteDate date) throws OrekitException {
		Orbit orbit = states.get(date).getOrbit();
		do {
			orbit = orbit.shiftedBy(date.durationFrom(orbit.getDate()));
		} while (!date.equals(orbit.getDate()));
		return orbit;
	}

	@Override
	protected double getMass(AbsoluteDate date) throws OrekitException {
		return DEFAULT_MASS;
	}

	@Override
	protected void resetIntermediateState(SpacecraftState state, boolean forward)
			throws OrekitException {
		throw new UnsupportedOperationException();
	}
}
