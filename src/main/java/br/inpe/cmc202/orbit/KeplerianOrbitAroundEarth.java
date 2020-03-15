package br.inpe.cmc202.orbit;

import java.util.Collection;

import org.hipparchus.analysis.solvers.NewtonRaphsonSolver;
import org.hipparchus.analysis.solvers.UnivariateDifferentiableSolver;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.TimeStampedPVCoordinates;

/**
 * The implementation of that Keplerian orbit using the Kepler Equation to
 * propagate orbit.
 * 
 * @author alessandro.g.romero
 * 
 */
public class KeplerianOrbitAroundEarth extends Orbit {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	/** Semi-major axis (m). */
	private final double a;

	/** Eccentricity. */
	private final double e;

	/** Inclination (rad). */
	private final double i;

	/** Perigee Argument (rad). */
	private final double pa;

	/** Right Ascension of Ascending Node (rad). */
	private final double raan;

	/** mean anomaly (rad). */
	private final double meanAnomaly;

	/** True anomaly (rad). */
	private final double v;

	/** eccentricity anomaly (rad). */
	private final double E;

	/**
	 * Creates a new instance.
	 * 
	 * @param a
	 *            semi-major axis (m), negative for hyperbolic orbits
	 * @param e
	 *            eccentricity
	 * @param i
	 *            inclination (rad)
	 * @param pa
	 *            perigee argument (rad)
	 * @param raan
	 *            right ascension of ascending node (rad)
	 * @param mean
	 *            anomaly mean anomaly (rad)
	 * @param frame
	 *            the frame in which the parameters are defined (<em>must</em>
	 *            be a {@link Frame#isPseudoInertial pseudo-inertial frame})
	 * @param date
	 *            date of the orbital parameters
	 * @exception IllegalArgumentException
	 *                if frame is not a {@link Frame#isPseudoInertial
	 *                pseudo-inertial frame} or a and e don't match for
	 *                hyperbolic orbits, or v is out of range for hyperbolic
	 *                orbits
	 */
	public KeplerianOrbitAroundEarth(final double a, final double e,
			final double i, final double pa, final double raan,
			final double meanAnomaly, final Frame frame, final AbsoluteDate date)
			throws IllegalArgumentException {
		// mu - central attraction coefficient (m3/s2)
		// 3.986004418x 10(14) m3/s2 (3.986004418 x10(5) km3/s2)
		super(frame, date, Constants.EGM96_EARTH_MU);

		this.a = a;
		this.e = e;
		this.i = i;
		this.pa = pa;
		this.raan = raan;

		this.E = meanToEllipticEccentric(meanAnomaly);
		this.v = ellipticEccentricToTrue(E);
		this.meanAnomaly = meanAnomaly;
	}

	/**
	 * Computes the elliptic eccentric anomaly from the mean anomaly.
	 * <p>
	 * The algorithm used here for solving Kepler equation has been published
	 * in: "Procedures for  solving Kepler's Equation", A. W. Odell and R. H.
	 * Gooding, Celestial Mechanics 38 (1986) 307-334
	 * </p>
	 * 
	 * @param M
	 *            mean anomaly (rad)
	 * @return E elliptic eccentric anomaly
	 */
	protected double meanToEllipticEccentric(final double M) {
		KeplerEquation keplerEquation = new KeplerEquation(e, M);
		UnivariateDifferentiableSolver solver = new NewtonRaphsonSolver();
		double E = solver.solve(10, keplerEquation, M);
		return E;
	}

	/**
	 * Computes the true anomaly from the elliptic eccentric anomaly.
	 * 
	 * @param E
	 *            eccentric anomaly (rad)
	 * @return v the true anomaly
	 */
	protected double ellipticEccentricToTrue(final double E) {
		final double v = FastMath.copySign(
				2 * FastMath.atan(FastMath.sqrt((1 + e) / (1 - e))
						* FastMath.tan(E / 2)), FastMath.tan(E / 2));

		// get rid off negative angles
		if (FastMath.tan(E / 2) < 0) {
			return 2 * FastMath.PI + v;
		} else {
			return v;
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.orekit.orbits.Orbit#getA()
	 */
	public double getA() {
		return a;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.orekit.orbits.Orbit#getE()
	 */
	public double getE() {
		return e;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.orekit.orbits.Orbit#getI()
	 */
	public double getI() {
		return i;
	}

	/**
	 * Get the perigee argument.
	 * 
	 * @return perigee argument (rad)
	 */
	public double getPerigeeArgument() {
		return pa;
	}

	/**
	 * Get the right ascension of the ascending node.
	 * 
	 * @return right ascension of the ascending node (rad)
	 */
	public double getRightAscensionOfAscendingNode() {
		return raan;
	}

	/**
	 * Get the true anomaly.
	 * 
	 * @return true anomaly (rad)
	 */
	public double getTrueAnomaly() {
		return v;
	}

	/**
	 * Get the eccentric anomaly.
	 * 
	 * @return eccentric anomaly (rad)
	 */
	public double getEccentricAnomaly() {
		return E;
	}

	/**
	 * Returns a string representation of this keplerian parameters object.
	 * 
	 * @return a string representation of this object
	 */
	public String toString() {
		return new StringBuffer().append("keplerian parameters: ").append('{')
				.append("a: ").append(a).append("; e: ").append(e)
				.append("; i: ").append(FastMath.toDegrees(i)).append("; pa: ")
				.append(FastMath.toDegrees(pa)).append("; raan: ")
				.append(FastMath.toDegrees(raan)).append("; v: ")
				.append(FastMath.toDegrees(v)).append(";}").toString();
	}

	@Override
	public Orbit shiftedBy(double dt) {
		// propagating mean anomaly
		final double n = Math.sqrt(getMu() / FastMath.pow(getA(), 3));
		final double meanAnomaly = this.meanAnomaly + n * dt;
		// creating a new orbit
		return new KeplerianOrbitAroundEarth(a, e, i, pa, raan, meanAnomaly,
				getFrame(), getDate().shiftedBy(dt));
	}

	@Override
	protected TimeStampedPVCoordinates initPVCoordinates() {
		// preliminary variables
		final double cosRaan = FastMath.cos(raan);
		final double sinRaan = FastMath.sin(raan);
		final double cosPa = FastMath.cos(pa);
		final double sinPa = FastMath.sin(pa);
		final double cosI = FastMath.cos(i);
		final double sinI = FastMath.sin(i);

		final double crcp = cosRaan * cosPa;
		final double crsp = cosRaan * sinPa;
		final double srcp = sinRaan * cosPa;
		final double srsp = sinRaan * sinPa;

		// reference axes defining the orbital plane
		final Vector3D p = new Vector3D(crcp - cosI * srsp, srcp + cosI * crsp,
				sinI * sinPa);
		final Vector3D q = new Vector3D(-crsp - cosI * srcp, -srsp + cosI
				* crcp, sinI * cosPa);

		// elliptic eccentric anomaly
		final double uME2 = (1 - e) * (1 + e);
		final double s1Me2 = FastMath.sqrt(uME2);
		final double E = getEccentricAnomaly();
		final double cosE = FastMath.cos(E);
		final double sinE = FastMath.sin(E);

		// coordinates of position and velocity in the orbital plane
		final double x = a * (cosE - e);
		final double y = a * sinE * s1Me2;
		final double factor = FastMath.sqrt(getMu() / a) / (1 - e * cosE);
		final double xDot = -sinE * factor;
		final double yDot = cosE * s1Me2 * factor;

		final Vector3D position = new Vector3D(x, p, y, q);
		final double r2 = x * x + y * y;
		final Vector3D velocity = new Vector3D(xDot, p, yDot, q);
		final Vector3D acceleration = new Vector3D(-getMu()
				/ (r2 * FastMath.sqrt(r2)), position);
		return new TimeStampedPVCoordinates(getDate(), position, velocity,
				acceleration);
	}

	// -------------------------------------------------------------
	// TO COMPILE
	public Orbit interpolate(AbsoluteDate date, Collection<Orbit> sample)
			throws OrekitException {
		throw new UnsupportedOperationException();
	}

	@Override
	public OrbitType getType() {
		throw new UnsupportedOperationException();
	}

	@Override
	public double getEquinoctialEx() {
		throw new UnsupportedOperationException();
	}

	@Override
	public double getEquinoctialEy() {
		throw new UnsupportedOperationException();
	}

	@Override
	public double getHx() {
		throw new UnsupportedOperationException();
	}

	@Override
	public double getHy() {
		throw new UnsupportedOperationException();
	}

	@Override
	public double getLE() {
		throw new UnsupportedOperationException();
	}

	@Override
	public double getLv() {
		throw new UnsupportedOperationException();
	}

	@Override
	public double getLM() {
		throw new UnsupportedOperationException();
	}

	@Override
	protected double[][] computeJacobianMeanWrtCartesian() {
		throw new UnsupportedOperationException();
	}

	@Override
	protected double[][] computeJacobianEccentricWrtCartesian() {
		throw new UnsupportedOperationException();
	}

	@Override
	protected double[][] computeJacobianTrueWrtCartesian() {
		throw new UnsupportedOperationException();
	}

	@Override
	public void addKeplerContribution(PositionAngle type, double gm,
			double[] pDot) {
		throw new UnsupportedOperationException();
	}

}