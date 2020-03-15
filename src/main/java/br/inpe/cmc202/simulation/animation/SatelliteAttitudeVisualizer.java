package br.inpe.cmc202.simulation.animation;

import java.applet.Applet;
import java.awt.BorderLayout;
import java.awt.Font;
import java.awt.GraphicsConfiguration;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.DecimalFormat;
import java.util.Map;

import javax.media.j3d.AmbientLight;
import javax.media.j3d.BoundingSphere;
import javax.media.j3d.BranchGroup;
import javax.media.j3d.Canvas3D;
import javax.media.j3d.DirectionalLight;
import javax.media.j3d.Transform3D;
import javax.media.j3d.TransformGroup;
import javax.swing.Timer;
import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import org.apache.commons.lang.ArrayUtils;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import br.inpe.cmc202.satellite.Satellite;

import com.sun.j3d.utils.applet.MainFrame;
import com.sun.j3d.utils.geometry.Box;
import com.sun.j3d.utils.geometry.Text2D;
import com.sun.j3d.utils.universe.SimpleUniverse;

/**
 * SatelliteAtitudeVisualizer. It simulates the body attitude. It is based on
 * Java 3D.
 * 
 * Based on http://www.java3d.org/animationinteraction.html
 * 
 * @author alessandro.g.romero
 */
public class SatelliteAttitudeVisualizer extends Applet implements
		ActionListener {

	final private Logger logger = LoggerFactory
			.getLogger(SatelliteAttitudeVisualizer.class);

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	// for the infrastructure of Java 3D supporting animation
	private TransformGroup objTrans;
	private Transform3D trans = new Transform3D();
	private BranchGroup textBranchGroup;
	private Timer timer;

	// for demo purpose
	private float demo = 0.0f;

	// for satellite attitude visualization
	private final Map<Double, double[]> eulerAngles;
	private final Map<Double, double[]> torque;
	private final double[] times;
	private int currentTime;
	private final DecimalFormat df;
	private final DecimalFormat dfTorque;

	/**
	 * Constructor.
	 */
	public SatelliteAttitudeVisualizer(Map<Double, double[]> eulerAngles,
			Map<Double, double[]> torque, Satellite satellite, long timeScale) {

		// setup layout
		setLayout(new BorderLayout());
		GraphicsConfiguration config = SimpleUniverse
				.getPreferredConfiguration();
		Canvas3D c = new Canvas3D(config);
		add("Center", c);

		// Create a scene and attach it to the virtual universe
		SimpleUniverse u = new SimpleUniverse(c);
		u.getViewingPlatform().setNominalViewingTransform();
		u.addBranchGraph(createSceneGraph(satellite));

		// animation and timer
		if (eulerAngles != null && eulerAngles.size() > 2) {
			this.eulerAngles = eulerAngles;
			this.torque = torque;
			this.times = ArrayUtils.toPrimitive(eulerAngles.keySet().toArray(
					new Double[0]));
			currentTime = 0;
			int animationInterval = (int) ((times[1] - times[0]) * 1000 / (timeScale));
			logger.info("Total simulation time (s): {} " + "\n Time scale: {}"
					+ "\n Total (approximated) visualizer time (s): {}"
					+ "\n Visualizer interval (ms): {}",
					times[times.length - 1], timeScale, times[times.length - 1]
							/ timeScale, animationInterval);
			timer = new Timer(animationInterval, this);
		} else {
			timer = new Timer(100, this);
			this.eulerAngles = null;
			this.torque = null;
			this.times = null;
		}

		// used to round the texts
		df = new DecimalFormat("000000.00");
		df.setRoundingMode(java.math.RoundingMode.CEILING);
		dfTorque = new DecimalFormat("0.000");
		df.setRoundingMode(java.math.RoundingMode.CEILING);
	}

	/**
	 * Create the scene.
	 * 
	 * @return
	 */
	private BranchGroup createSceneGraph(Satellite satellite) {
		// Create the root of the branch graph
		BranchGroup objRoot = new BranchGroup();

		// computing proportional dimensions based on the inertia axis
		Vector3D satelliteDimensionsNormalized = null;
		if (satellite != null) {
			Vector3D satelliteDimensions = new Vector3D(satellite.getI()
					.getEntry(0, 0), satellite.getI().getEntry(1, 1), satellite
					.getI().getEntry(2, 2));
			satelliteDimensionsNormalized = satelliteDimensions.normalize();
		} else {
			satelliteDimensionsNormalized = new Vector3D(0.1, 0.2, 0.3);
			satelliteDimensionsNormalized = satelliteDimensionsNormalized
					.normalize();
		}
		satelliteDimensionsNormalized = satelliteDimensionsNormalized
				.scalarMultiply(.4);

		// Create the satellite shape leaf node, add it to the scene graph.
		Box box = new Box((float) satelliteDimensionsNormalized.getX(),
				(float) satelliteDimensionsNormalized.getY(),
				(float) satelliteDimensionsNormalized.getZ(), null);
		objTrans = new TransformGroup();
		objTrans.setCapability(TransformGroup.ALLOW_TRANSFORM_WRITE);
		objTrans.addChild(box);
		objRoot.addChild(objTrans);

		// lights
		BoundingSphere bounds = new BoundingSphere(new Point3d(0.0, 0.0, 0.0),
				100.0);

		// sun light - yellow
		Color3f sunLightColor = new Color3f(1.0f, 1.0f, .0f);
		Vector3f sunLightDirection = new Vector3f(-0.323116f, -0.868285f,
				-0.376401f);
		DirectionalLight sunLight = new DirectionalLight(sunLightColor,
				sunLightDirection);
		sunLight.setInfluencingBounds(bounds);
		objRoot.addChild(sunLight);

		// Set up the ambient light
		Color3f ambientColor = new Color3f(1f, 1f, 1f);
		AmbientLight ambientLightNode = new AmbientLight(ambientColor);
		ambientLightNode.setInfluencingBounds(bounds);
		objRoot.addChild(ambientLightNode);

		// Create a branchgroup for text
		TransformGroup objTransText = new TransformGroup();
		textBranchGroup = new BranchGroup();
		textBranchGroup.setCapability(BranchGroup.ALLOW_DETACH);
		textBranchGroup.setCapability(BranchGroup.ALLOW_CHILDREN_WRITE);
		textBranchGroup.setCapability(BranchGroup.ALLOW_CHILDREN_EXTEND);
		Transform3D transform1 = new Transform3D();
		Vector3f vector1 = new Vector3f(0, -.6f, 0);
		transform1.setTranslation(vector1);
		objTransText.setTransform(transform1);
		objTransText.setCapability(TransformGroup.ALLOW_TRANSFORM_WRITE);
		objTransText.setCapability(TransformGroup.ALLOW_CHILDREN_WRITE);
		objTransText.addChild(textBranchGroup);
		objRoot.addChild(objTransText);

		objRoot.compile();

		return objRoot;

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * java.awt.event.ActionListener#actionPerformed(java.awt.event.ActionEvent)
	 */
	public void actionPerformed(ActionEvent e) {
		Text2D text2d = null;
		if (eulerAngles == null) {
			demo += .01;
			trans.setEuler(new Vector3d(demo, demo, 0));
			// trans.setEuler(new Vector3d(FastMath.toRadians(-133), FastMath
			// .toRadians(22), FastMath.toRadians(-69)));
			text2d = new Text2D("Time(s): " + df.format(demo)
					+ "  Torque(N.m): " + dfTorque.format(demo), new Color3f(
					0.9f, 1.0f, 1.0f), "Helvetica", 12, Font.ITALIC);
		} else {
			// rotation of the satellite
			trans.setEuler(new Vector3d(eulerAngles.get(times[currentTime])[0],
					eulerAngles.get(times[currentTime])[1], eulerAngles
							.get(times[currentTime])[2]));

			// control torque
			Vector3D torqueN = new Vector3D(torque.get(times[currentTime])[0],
					torque.get(times[currentTime])[1],
					torque.get(times[currentTime])[2]);

			// time and torque
			text2d = new Text2D("Time(s): " + df.format(times[currentTime])
					+ "  Torque(N.m): " + dfTorque.format(torqueN.getNorm()),
					new Color3f(0.9f, 1.0f, 1.0f), "Helvetica", 12, Font.ITALIC);

			currentTime++;
			if (currentTime == eulerAngles.size()) {
				timer.stop();
				logger.info("Visualizer ended.");
			}

		}

		objTrans.setTransform(trans);

		BranchGroup text2 = new BranchGroup();
		text2.addChild(text2d);
		text2.setCapability(BranchGroup.ALLOW_DETACH);
		textBranchGroup.removeAllChildren();
		textBranchGroup.addChild(text2);

	}

	/**
	 * Called to start animation.
	 */
	public void startAnimation() {
		new MainFrame(this, 500, 300).setTitle("Satellite Attitude Visualizer");
		timer.start();
	}

	/**
	 * Main to see the infrastructure of Java 3D.
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		new SatelliteAttitudeVisualizer(null, null, null, 0).startAnimation();
	}

}