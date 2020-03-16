package br.inpe.cmc202.simulation;

import java.text.DecimalFormat;
import java.util.Map;
import java.util.TreeMap;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.RotationOrder;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.attitudes.Attitude;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.sampling.OrekitFixedStepHandler;
import org.orekit.time.AbsoluteDate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.slf4j.MDC;

import br.inpe.cmc202.orbit.KeplerianOrbitAroundEarth;
import br.inpe.cmc202.satellite.Satellite;
import br.inpe.cmc202.satellite.controllers.BaseController;
import br.inpe.cmc202.satellite.controllers.sdre.hinfinity.ProportionalNonLinearMRPSDREHInfinityController;
import br.inpe.cmc202.satellite.controllers.sdre.hinfinity.ProportionalNonLinearQuaternionFullSDREHInfinityController;
import br.inpe.cmc202.simulation.animation.SatelliteAttitudeVisualizer;
import br.inpe.cmc202.simulation.plotter.Plotter;

/**
 * Step Handler, it stores the results of the simulation and plots at the end.
 * 
 * @author alessandro.g.romero
 * 
 */
public class StepHandler implements OrekitFixedStepHandler {

	final private Logger logger = LoggerFactory.getLogger(StepHandler.class);

	private final Frame ecef;
	private final AbsoluteDate startTime;

	private final Satellite satellite;

	private final boolean plot;

	// SIMULATION
	// to support plotting
	// 1.
	final Map<Double, Double> trueAnomaly = new TreeMap<Double, Double>();
	// 2.
	final Map<Double, double[]> solarVectorBody = new TreeMap<Double, double[]>();
	// 3.
	final Map<Double, double[]> errorSolarVector = new TreeMap<Double, double[]>();
	// 4.
	final Map<Double, double[]> angularVelocityBody = new TreeMap<Double, double[]>();
	// 5.
	final Map<Double, double[]> reactionWheelTorque = new TreeMap<Double, double[]>();
	// 6.
	final Map<Double, double[]> reactionWheelAngularVelocity = new TreeMap<Double, double[]>();
	// 7.
	final Map<Double, Double> reactionWheelNormAngularMomentum = new TreeMap<Double, Double>();
	// 8.
	final Map<Double, double[]> magneticVectorBody = new TreeMap<Double, double[]>();
	// 9.
	final Map<Double, Double> normMagneticVectorBody = new TreeMap<Double, Double>();
	// 10.
	final Map<Double, double[]> magnetorqueControlTorque = new TreeMap<Double, double[]>();
	// 11.
	final Map<Double, double[]> magneticDipole = new TreeMap<Double, double[]>();
	// 12.
	final Map<Double, double[]> eulerAnglesECI = new TreeMap<Double, double[]>();
	// 13.
	final Map<Double, double[]> eulerAnglesECEF = new TreeMap<Double, double[]>();
	// ADDITIONAL - POSITION ECI
	final Map<Double, double[]> positionEci = new TreeMap<Double, double[]>();
	// ADDITIONAL - POSITION ECEF
	final Map<Double, double[]> positionEcef = new TreeMap<Double, double[]>();
	// ADDITIONAL - POSITION ECEF
	final Map<Double, double[]> positionLLA = new TreeMap<Double, double[]>();
	// ADDITIONAL - KINETIC ENERGY
	final Map<Double, Double> kineticEnergy = new TreeMap<Double, Double>();
	// ADDITIONAL - SATELITE ATTITUDE
	final Map<Double, double[]> satelliteAttitude = new TreeMap<Double, double[]>();
	// ADDITIONAL - QUATERNION ERROR
	final Map<Double, double[]> quaternionError = new TreeMap<Double, double[]>();
	// ADDITIONAL - QUATERNION
	final Map<Double, double[]> quaternion = new TreeMap<Double, double[]>();
	// ADDITIONAL - SUN POINTING ERROR
	final Map<Double, Double> sunPointingErrorBody = new TreeMap<Double, Double>();
	// ADDITIONAL - Controllability
	final Map<Double, Double> detControllability = new TreeMap<Double, Double>();
	// ADDITIONAL - STATE SPACE QUATERNIONS
	final Map<Double, double[]> stateSpaceQuaternions = new TreeMap<Double, double[]>();
	// ADDITIONAL - Condition Number of Controllability
	final Map<Double, Double> conditionNumberControllability = new TreeMap<Double, Double>();
	// ADDITIONAL - Poincare Section
	final Map<Double, Double> poincareSection = new TreeMap<Double, Double>();
	private double lastQN = 0d;
	// ADDITIONAL - Gama
	final Map<Double, Double> gama = new TreeMap<Double, Double>();

	// to check intervalToStore
	final long intervalToStore;
	final long totalPointsToBeStored;
	long currentTime = 0;
	final DecimalFormat percent = new DecimalFormat("0%");

	private long startTimeSimuation;

	double lastStoredTime;

	/**
	 * @param ecef
	 * @param startTime
	 * @param satellite
	 * @param simulationTime
	 * @param step
	 * @param intervalToStore
	 */
	public StepHandler(Frame ecef, AbsoluteDate startTime, Satellite satellite,
			double simulationTime, double step, long intervalToStore,
			boolean plot) {
		this.ecef = ecef;
		this.startTime = startTime;
		this.satellite = satellite;
		this.intervalToStore = intervalToStore;
		this.totalPointsToBeStored = (long) (simulationTime / step)
				/ intervalToStore + 1;
		percent.setRoundingMode(java.math.RoundingMode.FLOOR);

		this.plot = plot;
	}

	public void init(SpacecraftState s0, AbsoluteDate t) {
		logger.info("Running the simulation...");
		startTimeSimuation = System.currentTimeMillis();
	}

	public void handleStep(SpacecraftState currentState, boolean isLast)
			throws OrekitException {
		MDC.put("date", currentState.getDate().toString());

		double[] errorQuaternionArray = satellite.getSpacecraftState()
				.getAdditionalState("sunerror");
		Rotation errorQuaternion = new Rotation(errorQuaternionArray[0],
				errorQuaternionArray[1], errorQuaternionArray[2],
				errorQuaternionArray[3], false);

		if (currentTime % intervalToStore == 0 || isLast) {
			// current position and angular cordinates
			Attitude attitude = currentState.getAttitude();

			// to report
			trueAnomaly
					.put(currentState.getDate().durationFrom(startTime),
							FastMath.toDegrees(((KeplerianOrbitAroundEarth) currentState
									.getOrbit()).getTrueAnomaly()));
			angularVelocityBody.put(
					currentState.getDate().durationFrom(startTime), attitude
							.getSpin().toArray());
			solarVectorBody.put(currentState.getDate().durationFrom(startTime),
					currentState.getAdditionalState("setofsunsensors"));

			// recomputing quaternion
			Vector3D previousSunError_body = new Vector3D(
					errorQuaternion.getAngles(RotationOrder.ZYX,
							RotationConvention.VECTOR_OPERATOR));
			errorSolarVector.put(
					currentState.getDate().durationFrom(startTime),
					previousSunError_body.toArray());

			if (satellite.getSetOfReactionWheels() != null) {
				reactionWheelTorque.put(
						currentState.getDate().durationFrom(startTime),
						satellite.getSetOfReactionWheels().getState()
								.getControlTorque().toArray());
				reactionWheelAngularVelocity.put(currentState.getDate()
						.durationFrom(startTime), satellite
						.getSetOfReactionWheels().getState()
						.getAngularVelocity().toArray());
				reactionWheelNormAngularMomentum.put(
						currentState.getDate().durationFrom(startTime),
						satellite
								.getSetOfReactionWheels()
								.getState()
								.getAngularMomentumNorm(
										satellite.getGyroscope().read(
												currentState)));
			}
			magneticVectorBody.put(
					currentState.getDate().durationFrom(startTime),
					currentState.getAdditionalState("magnetometer"));
			normMagneticVectorBody.put(
					currentState.getDate().durationFrom(startTime),
					new Vector3D(currentState
							.getAdditionalState("magnetometer")).getNorm());
			if (satellite.getSetOfMagnetorquer() != null) {
				magnetorqueControlTorque.put(currentState.getDate()
						.durationFrom(startTime), satellite
						.getSetOfMagnetorquer().getState().getControlTorque()
						.toArray());
				magneticDipole.put(
						currentState.getDate().durationFrom(startTime),
						satellite.getSetOfMagnetorquer().getState()
								.getMagneticDipole().toArray());
			}
			eulerAnglesECI.put(
					currentState.getDate().durationFrom(startTime),
					attitude.getRotation().getAngles(RotationOrder.ZYX,
							RotationConvention.VECTOR_OPERATOR));
			eulerAnglesECEF.put(
					currentState.getDate().durationFrom(startTime),
					attitude.withReferenceFrame(ecef)
							.getRotation()
							.getAngles(RotationOrder.ZYX,
									RotationConvention.VECTOR_OPERATOR));

			// ADDITIONALS
			positionEci.put(currentState.getDate().durationFrom(startTime),
					currentState.getPVCoordinates().getPosition().toArray());
			positionEcef.put(
					currentState.getDate().durationFrom(startTime),
					currentState
							.getOrbit()
							.getFrame()
							.getTransformTo(ecef, currentState.getDate())
							.transformPosition(
									currentState.getPVCoordinates()
											.getPosition()).toArray());
			positionLLA.put(
					currentState.getDate().durationFrom(startTime),
					satellite
							.getMagnetometer()
							.transformECEFToLLA(
									currentState
											.getOrbit()
											.getFrame()
											.getTransformTo(ecef,
													currentState.getDate())
											.transformPosition(
													currentState
															.getPVCoordinates()
															.getPosition()))
							.toArray());
			kineticEnergy.put(currentState.getDate().durationFrom(startTime),
					satellite.getKineticEnergy(currentState));

			satelliteAttitude.put(
					currentState.getDate().durationFrom(startTime),
					attitude.getRotation().getAngles(RotationOrder.ZYX,
							RotationConvention.VECTOR_OPERATOR));

			double[] quatErr = new double[] { errorQuaternion.getQ1(),
					errorQuaternion.getQ2(), errorQuaternion.getQ3(),
					errorQuaternion.getQ0() };

			quaternionError.put(currentState.getDate().durationFrom(startTime),
					quatErr);

			double[] quat = new double[] { attitude.getRotation().getQ1(),
					attitude.getRotation().getQ2(),
					attitude.getRotation().getQ3(),
					attitude.getRotation().getQ0() };

			this.quaternion.put(currentState.getDate().durationFrom(startTime),
					quat);

			// sun pointing error
			double x = currentState.getAdditionalState("setofsunsensors")[0];
			double acos = FastMath.acos(x);
			if (Double.isNaN(acos)
					&& (0.99 < FastMath.abs(x) && FastMath.abs(x) < 1.01)) {
				acos = 0;
			}
			this.sunPointingErrorBody.put(
					currentState.getDate().durationFrom(startTime),
					FastMath.toDegrees(acos));

			// controllability det
			double detControllability = 0D;
			if (satellite.getController() instanceof BaseController){
				detControllability = ((BaseController)satellite.getController()).getDetControllability();
			}
			this.detControllability.put(
					currentState.getDate().durationFrom(startTime),
					detControllability);
			// controllability conditionNumber
			double conditionNumberControllability = 0D;
			if (satellite.getController() instanceof BaseController){
				conditionNumberControllability = ((BaseController)satellite.getController()).getConditionNumberControllability();
			}
			this.conditionNumberControllability.put(
					currentState.getDate().durationFrom(startTime),
					conditionNumberControllability);

			// state space quaternions
			double[] quatErrVector = new double[] { errorQuaternion.getQ1(),
					errorQuaternion.getQ2(), errorQuaternion.getQ3()};
			stateSpaceQuaternions.put(currentState.getDate().durationFrom(startTime),
					quatErrVector);

			// gama
			double gama = 0.0D;
			if (satellite.getController() instanceof ProportionalNonLinearQuaternionFullSDREHInfinityController){
				gama = ((ProportionalNonLinearQuaternionFullSDREHInfinityController)satellite.getController()).getGama();
			}
			if (satellite.getController() instanceof ProportionalNonLinearMRPSDREHInfinityController){
				gama = ((ProportionalNonLinearMRPSDREHInfinityController)satellite.getController()).getGama();
			}
			this.gama.put(
					currentState.getDate().durationFrom(startTime),
					gama);

			lastStoredTime = currentState.getDate().durationFrom(startTime);

			System.out.print(percent.format(((float) trueAnomaly.size())
					/ totalPointsToBeStored));
		}

		// MUST BE INDEPENDENT OF INTERVAL TO STORE
		// poincare section
		if (lastQN == 0){
			lastQN = errorQuaternion.getQ3();
		} else{
			if (FastMath.signum(lastQN) != FastMath.signum(errorQuaternion.getQ3())){
				this.poincareSection.put(
						errorQuaternion.getQ1(),
						errorQuaternion.getQ2());
			}
			lastQN = errorQuaternion.getQ3();
		}

		// closing files
		if (isLast) {
			logger.info(""); // to skip a line
			logger.info("Simulation ran in {} s.",
					(System.currentTimeMillis() - startTimeSimuation) / 1000);
			close();
		}
		currentTime++;
	}

	public void close() {
		if (!plot) {
			logger.info("Not ploting data...");
			return;
		}

		logger.info("Ploting data...");

		// plotting RESULTS
		// 1.
		Plotter.plot2DScatter(trueAnomaly, "1. true anomaly - ECI", "degrees");
		// 2.
		Plotter.plot2DLine(angularVelocityBody,
				"2. angular velocity - Body ECI, ECI ("
						+ this.satellite.getController().getClass()
								.getSimpleName() + ")", false, "radians/second");
		// .getSimpleName() + ")", true, "deg/second");
		// TODO
		// IFTOOM

		// 3.
		Plotter.plot2DLine(solarVectorBody, "3. solar vector - BODY ("
				+ this.satellite.getController().getClass().getSimpleName()
				+ ")", false, "");
		// 4.
		Plotter.plot2DLine(errorSolarVector, "4. error solar vector - BODY ("
				+ this.satellite.getController().getClass().getSimpleName()
				+ ")", true, "degrees");
		// 5.
		Plotter.plot2DLine(reactionWheelTorque,
				"5. reaction wheel torque - BODY ("
						+ this.satellite.getController().getClass()
								.getSimpleName() + ")", false, "N.m");
		// 6.
		Plotter.plot2DLine(reactionWheelAngularVelocity,
				"6. reaction wheel anguar velocity - BODY ("
						+ this.satellite.getController().getClass()
								.getSimpleName() + ")", false, "radians/second");
		// .getSimpleName() + ")", true, "deg/second");
		// TODO
		// IFTOOM

		// 7.
		Plotter.plot2DScatter(reactionWheelNormAngularMomentum,
				"7. reaction wheel norm angular momentum - BODY ("
						+ this.satellite.getController().getClass()
								.getSimpleName() + ")", "Kg.m^2.radians/second");
		// 8.
		Plotter.plot2DLine(magneticVectorBody, "8. magnetic Vector - BODY",
				false, "nT");
		// 9.
		Plotter.plot2DScatter(normMagneticVectorBody,
				"9. normMagneticVector -  BODY", "nT");
		// 10.
		Plotter.plot2DLine(magnetorqueControlTorque,
				"10. magnetorque control torque -  BODY", false, "N.m");
		// 11.
		Plotter.plot2DLine(magneticDipole, "11. magnetic Dipole -  BODY",
				false, "A.m^2");
		// 12.
		Plotter.plot2DLine(eulerAnglesECI, "12. euler angles - ECI", true,
				"degrees");
		// 13.
		Plotter.plot2DLine(eulerAnglesECEF, "13. euler angles - ECEF", true,
				"degrees");
		// ADDITIONAL - POSITION ECI
		Plotter.plot3DScatter(positionEci, "14. position - ECI");
		// ADDITIONAL - POSITION ECEF
		Plotter.plot3DScatter(positionEcef, "15. position - ECEF");
		// ADDITIONAL - POSITION LLA
		Plotter.plot2DScatterLLA(positionLLA, "16. position - LLA");
		// ADDITIONAL - KINETIC ENERGY
		Plotter.plot2DScatter(kineticEnergy,
				"17. rotational part of kinetic energy - BODY", "Joule (J)");
		// ADDITIONAL - QUATERNION
		Plotter.plot2DLine(quaternion, "18. quaternion ("
				+ this.satellite.getController().getClass().getSimpleName()
				+ ")", "");
		// ADDITIONAL - QUATERNION ERROR
		Plotter.plot2DLine(quaternionError, "19. quaternion error ("
				+ this.satellite.getController().getClass().getSimpleName()
				+ ")", "quaternion error");

		// ADDITIONAL - SUN POINTING ERROR
		Plotter.plot2DScatter(sunPointingErrorBody,
				"20. sun pointing error - BODY ("
						+ this.satellite.getController().getClass()
								.getSimpleName() + ")", "degrees");
		
		// ADDITIONAL - DET CONTROLLABILITY
		Plotter.plot2DScatter(detControllability,
				"21. determinant of controllability ("
						+ this.satellite.getController().getClass()
								.getSimpleName() + ")", "");

		// ADDITIONAL - STATE SPACE Quaternion
		Plotter.plot3DScatter(stateSpaceQuaternions, "22. state space quaternion - BODY");
		// ADDITIONAL - STATE SPACE Velocity
		Plotter.plot3DScatter(angularVelocityBody, "23. state space velocity - BODY");

		// ADDITIONAL - CONDITION NUMBER CONTROLLABILITY
		Plotter.plot2DScatter(conditionNumberControllability,
				"24. condition number of controllability ("
						+ this.satellite.getController().getClass()
								.getSimpleName() + ")", "");

		// ADDITIONAL - GAMA
		Plotter.plot2DScatter(gama,
				"24. gama ("
						+ this.satellite.getController().getClass()
								.getSimpleName() + ")", "");

		// ADDITIONAL - 3D satellite attitude visualizer
		try {
			logger.trace("Sleeping until all graphs are shown.");
			Thread.sleep(10000);
			logger.info("Starting satellite attitude visualizer...");
			new SatelliteAttitudeVisualizer(satelliteAttitude,
					reactionWheelTorque, satellite, 10).startAnimation();
		} catch (InterruptedException ule) {
			logger.error("Aborted satellite attitude visualizer.", ule);
		} catch (UnsatisfiedLinkError ule) {
			logger.error(
					"It is not possible to start the satellite visualizer without Java 3D. Message: ",
					ule.getMessage());
			logger.info("Possible solutions: \n "
					+ "\n1. Download Java 3D library 1.3.1 or latter http://www.oracle.com/technetwork/java/javasebusiness/downloads/java-archive-downloads-java-client-419417.html#java3d-1.5.1-oth-JPR"
					+ "\n2. Merge bin and lib directories http://stackoverflow.com/questions/7966067/java-howto-resolve-java-lang-unsatisfiedlinkerror-no-j3d-in-java-library-path"
					+ ule.getMessage());
		}
	}
}
