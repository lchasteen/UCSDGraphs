package roadgraph;

import geography.GeographicPoint;

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

	//-- Object methods --//
	@Override
	public String toString() {
		return "Lat: " + getLatitude() + ", Lon: " + getLongitude();
	}
	
	@Override
	public int hashCode() {
		int result = 37;
		result = 31 * result + Double.hashCode(latitude);
		result = 31 * result + Double.hashCode(longitude);
		return result;
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
		return n.latitude == latitude && n.longitude == longitude;
	}

}
