package br.inpe.cmc202.frame.transform;

import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Transform;
import org.orekit.frames.TransformProvider;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeFunction;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.IERSConventions;

/**
 * A transform that based on Greenwich mean sidereal time (in radians) defines
 * the ECEF.
 * 
 * @author alessandro.g.romero
 * 
 */
public class EarthCenteredEarthFixedTransformProvider implements
		TransformProvider {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private TimeFunction<DerivativeStructure> conversionToGMST;

	/**
	 * Constructor
	 * 
	 * @throws OrekitException
	 */
	public EarthCenteredEarthFixedTransformProvider() throws OrekitException {
		// Get the function computing Greenwich mean sidereal time, in radians.
		// https://www.orekit.org/static/apidocs/index.html?org/orekit/time/TimeScale.html
		conversionToGMST = IERSConventions.IERS_2010
				.getGMSTFunction(TimeScalesFactory.getUT1(null, true));
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * org.orekit.frames.TransformProvider#getTransform(org.orekit.time.AbsoluteDate
	 * )
	 */
	public Transform getTransform(AbsoluteDate date) throws OrekitException {
		// compute the angle - radians
		DerivativeStructure gmst = conversionToGMST.value(date);
		// define the matrix
		//
		// This class implements rotations in a three-dimensional space.
		// Rotations can be represented by several different mathematical
		// entities (matrices, axe and angle, Cardan or Euler angles,
		// quaternions).
		// This class presents an higher level abstraction, more user-oriented
		// and
		// hiding this implementation details.
		// Well, for the curious, we use quaternions for the internal
		// representation.
		// https://www.hipparchus.org/apidocs/org/hipparchus/geometry/euclidean/threed/Rotation.html
		Rotation rot = new Rotation(
				new double[][] {
						{ gmst.cos().getReal(), gmst.sin().getReal(), 0 },
						{ -gmst.sin().getReal(), gmst.cos().getReal(), 0 },
						{ 0, 0, 1 } }, 0.000001d);
		// return transformation
		return new Transform(date, rot);
	}
}
