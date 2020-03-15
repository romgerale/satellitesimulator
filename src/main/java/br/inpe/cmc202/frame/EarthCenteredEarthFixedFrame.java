package br.inpe.cmc202.frame;

import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;

import br.inpe.cmc202.frame.transform.EarthCenteredEarthFixedTransformProvider;

/**
 * A implementation of ECEFE frame .
 * 
 * @author alessandro.g.romero
 * 
 */
public class EarthCenteredEarthFixedFrame extends Frame {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * Constructor.
	 * 
	 * @param parent
	 * @throws IllegalArgumentException
	 * @throws OrekitException
	 */
	public EarthCenteredEarthFixedFrame(Frame parent)
			throws IllegalArgumentException, OrekitException {
		super(parent, new EarthCenteredEarthFixedTransformProvider(),
				"EarthCenteredEarthFixed - ECEF - Pseudo Earth Fixed", false);
	}

}
