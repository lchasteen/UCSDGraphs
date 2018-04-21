package roadgraph;

import geography.GeographicPoint;

/**
 * Identified an edge in a {@link GeographicPoint} point graph.
 * Contains a geographic point which is the ending location from
 * the point of reference which contains this object.
 * <br><br>
 * @author Lane Chasteen
 */
public final class Edge {

	//-- properties --//
	private final GeographicPoint vertex;
	private final String roadName;
	private final String roadType;
	private final double length;

	//-- constructors --//
	/**
	 * Creates a new Edge.
	 * @param vertex - The ending reference point.
	 * @param roadName - The name of the road.
	 * @param roadType - The road type.
	 * @param length - The length from the starting point to the internal vertex.
	 * @throws IllegalArgumentException if vertex, roadName, or roadType is {@code null}.
	 */
	public Edge(GeographicPoint vertex, String roadName, String roadType, double length) throws IllegalArgumentException {
		if (vertex == null) {
			throw new IllegalArgumentException("Unable to create a new " 
					+ Edge.class.getSimpleName() 
					+ " with a null " 
					+ GeographicPoint.class.getSimpleName());
		}
		if (roadName == null || roadType == null) {
			throw new IllegalArgumentException("Unable to create a new " 
					+ Edge.class.getSimpleName() 
					+ " with a null road name or road type.");
		}
		this.vertex = vertex;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}

	//-- Edge methods --//
	/**
	 * Returns the {@link GeographicPoint} as the vertex reference.
	 * @return {@link GeographicPoint} as the vertex reference.
	 */
	public GeographicPoint getVertex() {
		return vertex;
	}

	/**
	 * Returns the road name.
	 * @return The road name.
	 */
	public String getRoadName() {
		return roadName;
	}

	/**
	 * Returns the road type.
	 * @return The road type.
	 */
	public String getRoadType() {
		return roadType;
	}

	/**
	 * Returns the distance in KM between the {@code vertex} and
	 * the parent.
	 * @return The distance in KM between the {@code vertex} and 
	 * the parent.
	 */
	public double getLength() {
		return length;
	}

	//-- Object methods --//
	@Override
	public boolean equals(Object o) {
		if (o == this) {
			return true;
		}
		if (!(o instanceof Edge)) {
			return false;
		}
		Edge e = (Edge) o;

		return e.vertex.equals(vertex)
				&& e.roadName.equals(roadName) 
				&& e.roadType.equals(roadType);
	}

	@Override
	public int hashCode() {
		int result = 37;
		result = 31 * result + vertex.hashCode();
		result = 31 * result + roadName.hashCode();
		result = 31 * result + roadType.hashCode();
		return result;
	}
}