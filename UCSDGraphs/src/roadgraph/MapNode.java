/**
 * A class to represent a node in the map
 */
package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

/**
 * @author UCSD MOOC development team
 * 
 * Class representing a vertex (or node) in our MapGraph
 *
 */
public class MapNode {
	/** The list of edges out of this node */
	private final Set<MapEdge> edges;

	/** the latitude and longitude of this node */
	private final GeographicPoint location;
	
	private volatile double startDistance;
	
	private volatile double possibleDistanceToEnd;

	/** 
	 * Create a new MapNode at a given Geographic location
	 * @param loc the location of this node
	 */
	public MapNode(GeographicPoint loc) {
		location = loc;
		edges = new HashSet<MapEdge>();
	}

	/**
	 * Add an edge that is outgoing from this node in the graph
	 * @param edge The edge to be added
	 */
	public void addEdge(MapEdge edge) {
		edges.add(edge);
	}

	/**  
	 * Return the neighbors of this MapNode 
	 * @return a set containing all the neighbors of this node
	 */
	public Set<MapNode> getNeighbors()	{
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for (MapEdge edge : edges) {
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}

	/**
	 * Get the geographic location that this node represents
	 * @return the geographic location of this node
	 */
	public GeographicPoint getLocation() {
		return location;
	}

	/**
	 * return the edges out of this node
	 * @return a set contianing all the edges out of this node.
	 */
	public Set<MapEdge> getEdges() {
		return edges;
	}
	
	/**
	 * Sets the distance from the start node.
	 * @param startDistance - The distance from the start node.
	 */
	public void setStartDistance(Double startDistance) {
		this.startDistance = startDistance;
	}
	
	/**
	 * Returns the distance from the start node.
	 * @return the distance from the start node.
	 */
	public Double getStartDistance() {
		return this.startDistance;
	}
	
	/**
	 * Sets the possible distance of the point to the end point.
	 * @param possibleDistance - The possible distance to the end point.
	 */
	public void setPossibleDistanceToEnd(double possibleDistance) {
		this.possibleDistanceToEnd = possibleDistance;
	}
	
	/**
	 * Returns the possible distance of the point to the end point.
	 * @return The possible distance of the point to the end point.
	 */
	public double getPossibleDistanceToEnd() {
		return this.possibleDistanceToEnd;
	}
	
	// For debugging, output roadNames as a String.
	public String roadNamesAsString() {
		String toReturn = "(";
		for (MapEdge e: edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += ")";
		return toReturn;
	}

	/** Returns whether two nodes are equal.
	 * Nodes are considered equal if their locations are the same, 
	 * even if their street list is different.
	 * @param o the node to compare to
	 * @return true if these nodes are at the same location, false otherwise
	 */
	@Override
	public boolean equals(Object o)	{
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode)o;
		return node.location.equals(this.location);
	}

	/** Because we compare nodes using their location, we also 
	 * may use their location for HashCode.
	 * @return The HashCode for this node, which is the HashCode for the 
	 * underlying point
	 */
	@Override
	public int hashCode() {
		return location.hashCode();
	}

	/** ToString to print out a MapNode object
	 *  @return the string representation of a MapNode
	 */
	@Override
	public String toString() {
		String toReturn = "[NODE at location (" + location + ")";
		toReturn += " intersects streets: ";
		for (MapEdge e: edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += "]";
		return toReturn;
	}
}