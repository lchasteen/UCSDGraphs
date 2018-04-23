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
	private final Node node;
	private final String roadName;
	private final String roadType;
	private final double length;

	//-- constructors --//
	/**
	 * Creates a new Edge.
	 * @param node - {@link Node} reference point.
	 * @param roadName - The name of the road.
	 * @param roadType - The road type.
	 * @param length - The length from the starting point to the internal vertex.
	 * @throws IllegalArgumentException if vertex, roadName, or roadType is {@code null}.
	 */
	public Edge(Node node, String roadName, String roadType, double length) throws IllegalArgumentException {
		if (node == null) {
			throw new IllegalArgumentException("Unable to create a new " 
					+ Edge.class.getSimpleName() 
					+ " with a null vertex.");
		}
		if (roadName == null || roadType == null) {
			throw new IllegalArgumentException("Unable to create a new " 
					+ Edge.class.getSimpleName() 
					+ " with a null road name or road type.");
		}
		if (length < 0) {
			throw new IllegalArgumentException("Unable to create a new " 
					+ Edge.class.getSimpleName() 
					+ ". The road length must be greater than or equal to zero");
		}
		
		this.node = node;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	/**
	 * Copy constructor.
	 * @param edge - {@link Edge} to copy.
	 */
	public Edge(Edge edge) {
		this(new Node(edge.getNode()), edge.getRoadName(), edge.getRoadType(), edge.getLength());
	}

	//-- Edge methods --//
	/**
	 * Returns the {@link Node} reference point.
	 * @return The {@link Node} reference point.
	 */
	public Node getNode() {
		return node;
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

		return e.getNode().equals(node)
				&& e.roadName.equals(roadName) 
				&& e.roadType.equals(roadType);
	}

	@Override
	public int hashCode() {
		int result = 37;
		result = 31 * result + node.hashCode();
		result = 31 * result + roadName.hashCode();
		result = 31 * result + roadType.hashCode();
		return result;
	}
}