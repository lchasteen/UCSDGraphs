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
	private final PriorityNode vertex;
	private final String roadName;
	private final String roadType;

	//-- constructors --//
	/**
	 * Creates a new Edge.
	 * @param vertex - {@link PriorityNode} ending reference point.
	 * @param roadName - The name of the road.
	 * @param roadType - The road type.
	 * @throws IllegalArgumentException if vertex, roadName, or roadType is {@code null}.
	 */
	public Edge(PriorityNode vertex, String roadName, String roadType) throws IllegalArgumentException {
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
	}

	//-- Edge methods --//
	/**
	 * Returns the {@link Node} as the vertex reference.
	 * @return {@link Node} as the vertex reference.
	 */
	public PriorityNode getVertex() {
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