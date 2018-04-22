package roadgraph;

import geography.GeographicPoint;

/**
 * A node describing a vertex in a graph.
 * <br><br>
 * @author Lane Chasteen
 */
public class Node {
	//-- properties --//
	private final double latitude;
	private final double longitude;

	//-- constructors --//
	/**
	 * Creates a new Node.
	 * @param latitude - Latitude.
	 * @param longitude - Longitude.
	 */
	public Node(double latitude, double longitude) {
		this.latitude = latitude;
		this.longitude = longitude;
	}
	
	/**
	 * Creates a new Node.
	 * @param point - {@link GeographicPoint} to create from.
	 */
	public Node(GeographicPoint point) {
		this(point.getX(), point.getY());
	}
	
	/**
	 * Copy constructor. Creates a new Node.
	 * @param n - {@link Node} to copy.
	 */
	public Node(Node n) {
		this(n.getLatitude(), n.getLongitude());
	}

	//-- Node methods --//
	/**
	 * Calculates the geographic distance in km between this point and 
	 * the other point. 
	 * @param other 
	 * @return The distance between this lat, lon point and the other point
	 */
	public static double distance(Node n1, Node n2) {
		return GeographicPoint.getDist(n1.getLatitude(), n1.getLongitude(),
				n2.getLatitude(), n2.getLongitude());     
	}
	
	/**
	 * Returns the latitude.
	 * @return The latitude.
	 */
	public double getLatitude() {
		return this.latitude;
	}

	/**
	 * Returns the longitude.
	 * @return The longitude.
	 */
	public double getLongitude() {
		return this.longitude;
	}
	
	public GeographicPoint getGeographicPoint() {
		return new GeographicPoint(latitude, longitude);
	}

	//-- Object methods --//
	@Override
	public String toString() {
		return "Lat: " + getLatitude() + ", Lon: " + getLongitude();
	}
	
	@Override
	public int hashCode() {
		long bits = java.lang.Double.doubleToLongBits(latitude);
		bits ^= java.lang.Double.doubleToLongBits(longitude) * 31;
		return (((int) bits) ^ ((int) (bits >> 32)));
	}
	
	@Override
	public boolean equals(Object o) {
		if (o == this) {
			return true;
		}
		if (!(o instanceof Node)) {
			return false;
		}
		
		Node n = (Node) o;
		return (Double.compare(n.latitude, latitude) == 0) && (Double.compare(n.longitude, longitude) == 0);
	}
}
