package br.inpe.cmc202.frame;

import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.Transform;

/**
 * A implementation of ECI frame that it is used the first child of Teme.
 * 
 * new EarthCenteredInertialFrame(FramesFactory.getTEME());
 * 
 * @author alessandro.g.romero
 * 
 */
public class EarthCenteredInertialFrame extends Frame {

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
	public EarthCenteredInertialFrame(Frame parent)
			throws IllegalArgumentException, OrekitException {
		super(
				parent,
				Transform.IDENTITY,
				"EarthCenteredInertial - ECI - TEME (True Equator, Mean Equinox)",
				true);
	}
}
