package br.inpe.cmc202.satellite;

import java.util.Properties;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.DecompositionSolver;
import org.hipparchus.linear.LUDecomposition;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.propagation.SpacecraftState;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.satellite.actuators.SetOfMagnetorquers;
import br.inpe.cmc202.satellite.actuators.SetOfReactionWheels;
import br.inpe.cmc202.satellite.controllers.Controller;
import br.inpe.cmc202.satellite.controllers.NopeController;
import br.inpe.cmc202.satellite.controllers.SetOfMagnetorquersController;
import br.inpe.cmc202.satellite.controllers.linear.ProportionalDerivativeLinearSunVectorController;
import br.inpe.cmc202.satellite.controllers.lqr.ProportionalLinearEulerAnglesLQRController;
import br.inpe.cmc202.satellite.controllers.lqr.ProportionalLinearQuaternionFullLQRController;
import br.inpe.cmc202.satellite.controllers.lqr.ProportionalLinearQuaternionPartialLQRController;
import br.inpe.cmc202.satellite.controllers.sdre.ProportionalNonLinearEulerAnglesSDREController;
import br.inpe.cmc202.satellite.controllers.sdre.ProportionalNonLinearMRPSDREController;
import br.inpe.cmc202.satellite.controllers.sdre.ProportionalNonLinearQuaternionFullSDREController;
import br.inpe.cmc202.satellite.controllers.sdre.hinfinity.ProportionalNonLinearMRPSDREHInfinityController;
import br.inpe.cmc202.satellite.controllers.sdre.hinfinity.ProportionalNonLinearQuaternionFullSDREHInfinityController;
import br.inpe.cmc202.satellite.sensors.Gyroscope;
import br.inpe.cmc202.satellite.sensors.Magnetometer;
import br.inpe.cmc202.satellite.sensors.SetOfSunSensors;

/**
 * 
 * Class that centralizes the satellite configuration.
 * 
 * 
 * @author alessandro.g.romero
 * 
 */
public class Satellite {

	final private Logger logger = LoggerFactory.getLogger(Satellite.class);

	/**
	 * Inertia Tensor It is related to the centre of mass of the satellite.
	 */
	final private RealMatrix I;

	/**
	 * Inertia Tensor (^-1) - inverse
	 */
	final private RealMatrix I_inverse;

	/**
	 * Inertia Tensor It is related to the centre of mass of the satellite. This
	 * inertia tensor reflects the parametric uncertainty in the "real"
	 * satellite.
	 */
	final private RealMatrix I_mu;

	/**
	 * Inertia Tensor (^-1) - inverse This inertia tensor reflects the
	 * parametric uncertainty in the "real" satellite.
	 */
	final private RealMatrix I_inverse_mu;

	// sensors
	final private SetOfSunSensors setOfSunSensors;
	final private Magnetometer magnetometer;
	final private Gyroscope gyroscope;

	// actuators
	final private SetOfMagnetorquers setOfMagnetorquer;
	final private SetOfReactionWheels setOfReactionWheels;

	// controllers
	final private Controller controller;
	final private SetOfMagnetorquersController setOfMagnetorquerController;

	// references
	final Vector3D angularVelocityReference_body;
	final Vector3D sunReference_body;

	// to store sate for the controller
	private SpacecraftState spacecraftState;

	/**
	 * It defines if the current mode for attitude is to define a target
	 * quarternion or if it computes comparing the solar vector and
	 * the desired direction (it does not know the current attitude - only two
	 * vectors in the body frame).
	 */
	private boolean modeUsingAttitude = true;

	// to store the target attitude in ECI
	private Rotation targetQuaternion = null;

	/**
	 * It is used to indicate that external torques must be accounted using a
	 * random variable with: mean 0, variance 1 and the magnitude defined.
	 * 
	 * It is used in the kinetics.
	 */
	private double externalTorquesMagnitude = 0;

	/**
	 * It is used in the SDRE Controllers parametrized A(x,\alpha)
	 */
	private double alpha1 = 0d;

	/**
	 *  To store the controllerName.
	 */
	final private String reactionWheelControllerName;
	
	/**
	 * Constructor.
	 * 
	 * @param ecef
	 * @param reactionWheelControllerName
	 * @param magnetorqueControllerName
	 * @param satelliteConfiguration
	 */
	public Satellite(Frame ecef, String reactionWheelControllerName,
			String magnetorqueControllerName,
			Properties satelliteConfiguration, Properties inertiaTensor) {
		// sensors
		this.setOfSunSensors = new SetOfSunSensors();
		this.magnetometer = new Magnetometer(ecef);
		this.gyroscope = new Gyroscope();

		// actuators
		if ("SetOfMagnetorquersController".equals(magnetorqueControllerName)) {
			this.setOfMagnetorquer = new SetOfMagnetorquers(
					satelliteConfiguration);
		} else {
			this.setOfMagnetorquer = null;
		}
		this.setOfReactionWheels = new SetOfReactionWheels(
				satelliteConfiguration);

		// ANGULAR VELOCITY - REFERENCE
		// ----------------------------------
		if (satelliteConfiguration == null) {
			this.angularVelocityReference_body = new Vector3D(0, 0, 0);
		} else {
			this.angularVelocityReference_body = new Vector3D(
					Double.valueOf(satelliteConfiguration.getProperty(
							"reference.angularVelocity.x", "0")),
					Double.valueOf(satelliteConfiguration.getProperty(
							"reference.angularVelocity.y", "0")),
					Double.valueOf(satelliteConfiguration.getProperty(
							"reference.angularVelocity.z", "0")));
		}

		// SUN - REFERENCE
		// ----------------------------------
		if (satelliteConfiguration == null) {
			this.sunReference_body = new Vector3D(1, 0, 0);
		} else {
			this.sunReference_body = new Vector3D(
					Double.valueOf(satelliteConfiguration.getProperty(
							"reference.sun.x", "1")),
					Double.valueOf(satelliteConfiguration.getProperty(
							"reference.sun.y", "0")),
					Double.valueOf(satelliteConfiguration.getProperty(
							"reference.sun.z", "0")));
		}
		// INERTIA TENSORS
		// ----------------------------------
		RealMatrix I = null;
		if (satelliteConfiguration == null) {
			I = MatrixUtils.createRealMatrix(new double[][] {
					{ 310, 1.11, 1.01 }, { 1.11, 360, -0.35 },
					{ 1.01, -0.35, 530.7 } });
		} else {
			I = MatrixUtils.createRealMatrix(new double[][] {
					{
							Double.valueOf(satelliteConfiguration.getProperty(
									"inertiaMoment.1.1", "310d")),
							Double.valueOf(satelliteConfiguration.getProperty(
									"inertiaMoment.1.2", "1.11d")),
							Double.valueOf(satelliteConfiguration.getProperty(
									"inertiaMoment.1.3", "1.01d")) },
					{
							Double.valueOf(satelliteConfiguration.getProperty(
									"inertiaMoment.2.1", "1.11d")),
							Double.valueOf(satelliteConfiguration.getProperty(
									"inertiaMoment.2.2", "360d")),
							Double.valueOf(satelliteConfiguration.getProperty(
									"inertiaMoment.2.3", "-0.35d")) },
					{
							Double.valueOf(satelliteConfiguration.getProperty(
									"inertiaMoment.3.1", "1.01d")),
							Double.valueOf(satelliteConfiguration.getProperty(
									"inertiaMoment.3.2", "-0.35d")),
							Double.valueOf(satelliteConfiguration.getProperty(
									"inertiaMoment.3.3", "530.7d")) } });
		}

		//
		if (getSetOfReactionWheels() == null) {
			this.I = I;
		} else {
			// taking into account the set of reaction wheels
			this.I = I.subtract(getSetOfReactionWheels()
					.getInertiaTensorContribution());
		}

		// inverting the inertia tensor
		// it will be used
		// it uses the lower upper decomposition as defined in the
		// hipparchus
		this.I_inverse = new LUDecomposition(this.I).getSolver().getInverse();

		// ********************
		// MODEL UNCERTAINTY
		// ********************
		if (inertiaTensor == null) {
			// MODEL UNCERTAINTY NOT PRESENT
			this.I_mu = this.I;
			this.I_inverse_mu = this.I_inverse;
		} else {
			RealMatrix I_mu = null;
			// MODEL UNCERTAINTY PRESENT
			I_mu = MatrixUtils.createRealMatrix(new double[][] {
					{
							Double.valueOf(inertiaTensor.getProperty(
									"inertiaMoment.1.1")),
							Double.valueOf(inertiaTensor.getProperty(
									"inertiaMoment.1.2")),
							Double.valueOf(inertiaTensor.getProperty(
									"inertiaMoment.1.3")) },
					{
							Double.valueOf(inertiaTensor.getProperty(
									"inertiaMoment.2.1")),
							Double.valueOf(inertiaTensor.getProperty(
									"inertiaMoment.2.2")),
							Double.valueOf(inertiaTensor.getProperty(
									"inertiaMoment.2.3")) },
					{
							Double.valueOf(inertiaTensor.getProperty(
									"inertiaMoment.3.1")),
							Double.valueOf(inertiaTensor.getProperty(
									"inertiaMoment.3.2")),
							Double.valueOf(inertiaTensor.getProperty(
									"inertiaMoment.3.3")) } });

			//
			if (getSetOfReactionWheels() == null) {
				this.I_mu = I_mu;
			} else {
				// taking into account the set of reaction wheels
				this.I_mu = I_mu.subtract(getSetOfReactionWheels()
						.getInertiaTensorContribution());
			}

			// inverting the inertia tensor
			// it will be used
			// it uses the lower upper decomposition as defined in the
			// hipparchus
			this.I_inverse_mu = new LUDecomposition(this.I_mu).getSolver()
					.getInverse();
		}
		// ********************
		// MODEL UNCERTAINTY
		// ********************

		// controller
		this.reactionWheelControllerName = reactionWheelControllerName;
		switch (reactionWheelControllerName) {
		case "ProportionalDerivativeLinearSunVectorController":
			this.controller = new ProportionalDerivativeLinearSunVectorController();
			break;
		case "ProportionalLinearEulerAnglesLQRController":
			this.controller = new ProportionalLinearEulerAnglesLQRController(
					this);
			break;
		case "ProportionalNonLinearEulerAnglesSDREController":
			this.controller = new ProportionalNonLinearEulerAnglesSDREController(
					this);
			break;
		case "ProportionalLinearQuaternionFullLQRController":
			this.controller = new ProportionalLinearQuaternionFullLQRController(
					this);
			break;
		case "ProportionalLinearQuaternionPartialLQRController":
			this.controller = new ProportionalLinearQuaternionPartialLQRController(
					this);
			break;
		case "ProportionalNonLinearQuaternionSDREController_OMEGA":
			// NO CONTROLLABLE
			this.controller = new ProportionalNonLinearQuaternionFullSDREController(
					this, "OMEGA");
			break;
		case "ProportionalNonLinearQuaternionSDREController_XI":
			// NO CONTROLLABLE
			this.controller = new ProportionalNonLinearQuaternionFullSDREController(
					this, "XI");
			break;
		case "ProportionalNonLinearQuaternionSDREController_GIBBS":
			this.controller = new ProportionalNonLinearQuaternionFullSDREController(
					this, "GIBBS");
			break;
		case "ProportionalNonLinearQuaternionSDREController_GIBBS_SECOND":
			this.controller = new ProportionalNonLinearQuaternionFullSDREController(
					this, "GIBBS_SECOND");
			break;
		case "ProportionalNonLinearQuaternionSDREController_ALPHA":
			this.controller = new ProportionalNonLinearQuaternionFullSDREController(
					this, "ALPHA");
			break;
		case "ProportionalNonLinearMRPSDREController_FIRST":
			this.controller = new ProportionalNonLinearMRPSDREController(this,
					"FIRST");
			break;
		case "ProportionalNonLinearMRPSDREController_SECOND":
			this.controller = new ProportionalNonLinearMRPSDREController(this,
					"SECOND");
			break;
		case "ProportionalNonLinearMRPSDREController_ALPHA":
			this.controller = new ProportionalNonLinearMRPSDREController(this,
					"ALPHA");
			break;
		case "ProportionalNonLinearQuaternionFullSDREHInfinityController":
			this.controller = new ProportionalNonLinearQuaternionFullSDREHInfinityController(
				this, "GIBBS");
			break;
		case "ProportionalNonLinearMRPSDREHInfinityController":
			this.controller = new ProportionalNonLinearMRPSDREHInfinityController(this,
				"FIRST");
			break;
			
		default:
			this.controller = new NopeController();
		}

		if (!"SetOfMagnetorquersController".equals(magnetorqueControllerName)) {
			this.setOfMagnetorquerController = null;
		} else {
			this.setOfMagnetorquerController = new SetOfMagnetorquersController();
		}
	}

	/**
	 * Additional constructor for the definition of alpha and external torques.
	 * 
	 * @param ecef
	 * @param reactionWheelControllerName
	 * @param magnetorqueControllerName
	 * @param alpha1
	 * @param externalTorquesMagnitude
	 * @param satelliteProperties
	 */
	public Satellite(Frame ecef, String reactionWheelControllerName,
			String magnetorqueControllerName, double alpha1,
			double externalTorquesMagnitude, Properties satelliteProperties,
			Properties inertiaTensor) {
		this(ecef, reactionWheelControllerName, magnetorqueControllerName,
				satelliteProperties, inertiaTensor);
		this.alpha1 = alpha1;
		this.externalTorquesMagnitude = externalTorquesMagnitude;
	}

	/**
	 * Get the Inertia tensor to be used
	 * 
	 * @return
	 */
	public RealMatrix getI() {
		return this.I_mu;
	}

	/**
	 * Get the inverse of the Inertia tensor ^-1 to be used
	 * 
	 * @return
	 */
	public RealMatrix getI_inverse() {
		return this.I_inverse_mu;
	}

	/**
	 * Get the Inertia tensor to be used
	 * 
	 * @return
	 */
	public RealMatrix getI_nominal() {
		return this.I;
	}

	/**
	 * Get the inverse of the Inertia tensor ^-1 to be used
	 * 
	 * @return
	 */
	public RealMatrix getI_inverse_nominal() {
		return this.I_inverse;
	}
	
	/**
	 * Set of sun sensors exposed.
	 * 
	 * @return
	 */
	public SetOfSunSensors getSetOfSunSensors() {
		return setOfSunSensors;
	}

	/**
	 * Magnetometer exposed.
	 * 
	 * @return
	 */
	public Magnetometer getMagnetometer() {
		return magnetometer;
	}

	/**
	 * Gyroscope exposed.
	 * 
	 * @return
	 */
	public Gyroscope getGyroscope() {
		return gyroscope;
	}

	/**
	 * Set of magnetorquer exposed.
	 * 
	 * @return
	 */
	public SetOfMagnetorquers getSetOfMagnetorquer() {
		return setOfMagnetorquer;
	}

	/**
	 * Set of reaction wheels exposed.
	 * 
	 * @return
	 */
	public SetOfReactionWheels getSetOfReactionWheels() {
		return setOfReactionWheels;
	}

	/**
	 * telemetry methods - sun error.
	 * 
	 * @param currentState
	 * @return
	 * @throws OrekitException
	 */
	public double[] getSunError(SpacecraftState currentState)
			throws OrekitException {

		Rotation errorQuaternions = null;

		if (!modeUsingAttitude) {
			// a rotation (quaternion)
			// from actual reading of the sensors (in the body system) to
			// the reference (in the body system)
			// it was working with that
			errorQuaternions = new Rotation(
					getSetOfSunSensors().read(currentState), sunReference_body);
	
			logger.trace(
					"Quaternion (actual reading to reference): Q1 {} Q2 {} Q3 {} Q4 {} ",
					errorQuaternions.getQ1(), errorQuaternions.getQ2(),
					errorQuaternions.getQ3(), errorQuaternions.getQ0());
	
		} else {
			if (targetQuaternion == null){
				// a rotation (quaternion)
				// from actual reading of the sensors (in the eci system) to
				// the reference (in the eci system)
				// IN THE ECI SYSTEM
				final Vector3D sun = currentState.toTransform().getInverse()
						.transformVector(getSetOfSunSensors().read(currentState));
				final Vector3D sunT = currentState.toTransform().getInverse()
						.transformVector(sunReference_body);
	
				targetQuaternion = new Rotation(sun, sunT);
	
				logger.info(
						"Target quaternion defined - - Q1 {} Q2 {} Q3 {} Q4 {}  ",
						targetQuaternion.getQ1(), targetQuaternion.getQ2(),
						targetQuaternion.getQ3(), targetQuaternion.getQ0() );
				errorQuaternions = targetQuaternion;
			} else {				
				// computing difference from target quaternion to
				// observed quaternion
				// q_error = q_observed^-1 * q_target
				// Wertz 18-43b
				RealMatrix qt = MatrixUtils.createRealMatrix(4, 4);
				qt.setEntry(0, 0, targetQuaternion.getQ0());
				qt.setEntry(0, 1, targetQuaternion.getQ3());
				qt.setEntry(0, 2, -targetQuaternion.getQ2());
				qt.setEntry(0, 3, targetQuaternion.getQ1());
	
				qt.setEntry(1, 0, -targetQuaternion.getQ3());
				qt.setEntry(1, 1, targetQuaternion.getQ0());
				qt.setEntry(1, 2, targetQuaternion.getQ1());
				qt.setEntry(1, 3, targetQuaternion.getQ2());
	
				qt.setEntry(2, 0, targetQuaternion.getQ2());
				qt.setEntry(2, 1, -targetQuaternion.getQ1());
				qt.setEntry(2, 2, targetQuaternion.getQ0());
				qt.setEntry(2, 3, targetQuaternion.getQ3());
	
				qt.setEntry(3, 0, -targetQuaternion.getQ1());
				qt.setEntry(3, 1, -targetQuaternion.getQ2());
				qt.setEntry(3, 2, -targetQuaternion.getQ3());
				qt.setEntry(3, 3, targetQuaternion.getQ0());
	
				RealVector qo_ = new ArrayRealVector(new double[] {
						-currentState.getAttitude().getRotation().getQ1(),
						-currentState.getAttitude().getRotation().getQ2(),
						-currentState.getAttitude().getRotation().getQ3(),
						currentState.getAttitude().getRotation().getQ0() });
	
				RealVector qe = qt.operate(qo_);
	
				// Rotation based on qe
				errorQuaternions = new Rotation(qe.getEntry(3), qe.getEntry(0),
						qe.getEntry(1), qe.getEntry(2), false);
	
				logger.trace(
						"Quaternion (COMPUTED): Q1 {} Q2 {} Q3 {} Q4 {} ",
						errorQuaternions.getQ1(), errorQuaternions.getQ2(),
						errorQuaternions.getQ3(), errorQuaternions.getQ0());
			} 
		}
		return new double[] { errorQuaternions.getQ0(),
				errorQuaternions.getQ1(), errorQuaternions.getQ2(),
				errorQuaternions.getQ3() };
	}

	/**
	 * Kinetic energy in the satellite.
	 * 
	 * @param currentState
	 * @return
	 * @throws OrekitException
	 */
	public double getKineticEnergy(SpacecraftState currentState)
			throws OrekitException {
		RealVector angularVelocity_body = new ArrayRealVector(getGyroscope()
				.read(currentState).toArray(), true);
		double firstComponent = angularVelocity_body.dotProduct(I
				.operate(angularVelocity_body)) / 2;
		if (getSetOfReactionWheels() == null) {
			return firstComponent;
		} else {
			double secondComponent = angularVelocity_body
					.dotProduct(new ArrayRealVector(getSetOfReactionWheels()
							.getInertiaTensorContribution().operate(
									getSetOfReactionWheels().getState()
											.getAngularVelocity().toArray())));

			double thirdComponent = new ArrayRealVector(
					getSetOfReactionWheels().getState().getAngularVelocity()
							.toArray()).dotProduct(new ArrayRealVector(
					getSetOfReactionWheels().getInertiaTensorContribution()
							.operate(
									getSetOfReactionWheels().getState()
											.getAngularVelocity().toArray()))) / 2;
			return firstComponent + secondComponent + thirdComponent;
		}
	}

	/**
	 * Exposes the controller.
	 * 
	 * @return
	 */
	public Controller getController() {
		return controller;
	}

	/**
	 * @return the setOfMagnetorquerController
	 */
	public SetOfMagnetorquersController getSetOfMagnetorquerController() {
		return setOfMagnetorquerController;
	}

	/**
	 * @return the spacecraftState
	 */
	public SpacecraftState getSpacecraftState() {
		return spacecraftState;
	}

	/**
	 * @param spacecraftState
	 *            the spacecraftState to set
	 */
	public void setSpacecraftState(SpacecraftState spacecraftState) {
		this.spacecraftState = spacecraftState;
	}

	/**
	 * @return the externalTorquesMagnitude
	 */
	public double getExternalTorquesMagnitude() {
		return externalTorquesMagnitude;
	}

	/**
	 * @return the alpha1
	 */
	public double getAlpha1() {
		return alpha1;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return "Satellite [\n setOfMagnetorquer=" + setOfMagnetorquer
				+ ",\n setOfReactionWheels=" + setOfReactionWheels
				+ ",\n controller=" + controller
				+ ",\n setOfMagnetorquerController="
				+ setOfMagnetorquerController
				+ ",\n angularVelocityReference_body="
				+ angularVelocityReference_body + ",\n sunReference_body="
				+ sunReference_body + ",\n externalTorquesMagnitude="
				+ externalTorquesMagnitude + ",\n alpha1=" + alpha1 + "]";
	}
	
	
	/**
	 * @return the maximum angular velocity controllable by the reaction wheels (individually)
	 */
	public RealVector getMaximumAngularVelocityControllableByReactionWheels() {
		
		// calculating maximum angular momentum of the reaction wheels
		final SetOfReactionWheels set = this.getSetOfReactionWheels();
		final RealMatrix inertiaSet = set.getInertiaTensorContribution();
		final RealMatrix maxAngularVelocity = MatrixUtils
				.createRealMatrix(new double[][] { { set.getMAX_ANGULAR_VELOCITY() }, { set.getMAX_ANGULAR_VELOCITY() },
						{ set.getMAX_ANGULAR_VELOCITY() } });
		final RealMatrix maxActuatorsAngularMomentum = inertiaSet.multiply(maxAngularVelocity);

		
		// calculating max angular velocity = Iw = L
		final RealMatrix inertia = this.getI();
		final DecompositionSolver solver = new LUDecomposition(inertia).getSolver();
		final RealMatrix maxAngVelocity = solver.solve(maxActuatorsAngularMomentum);

		return new ArrayRealVector(maxAngVelocity.getColumn(0));
	}

	/**
	 * Set mode used for attitude to compute the sun error
	 * false = step by step difference of vectors 
	 * true = quaternion error
	 */
	public void setModeUsingAttitude(boolean mode) {
		this.modeUsingAttitude = mode;
	}
	
	/**
	 * Returns the controller name in the satellite.
	 */
	public String getReactionWheelControllerName() {
		return this.reactionWheelControllerName;
	}
}
