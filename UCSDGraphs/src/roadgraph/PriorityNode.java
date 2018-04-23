package roadgraph;

import geography.GeographicPoint;

/**
 * Wrapper for the {@link Node} to provide weighted value.
 * <br><br>
 * @author Lane Chasteen
 */
public class PriorityNode extends Node {
	//-- properties --//
	private volatile double weight;
	/**
	 * Lock for {@link #weight}
	 */
	private final Object lock = new Object();

	//-- constructors --//
	/**
	 * Creates a new PriorityNode.
	 * @param node - {@link Node}.
	 * @param weight - Calculated value from previous point up to this point.
	 */
	public PriorityNode(Node node, double weight) {
		super(node);
		this.weight = weight;
	}
	
	/**
	 * Creates a new PriorityNode.
	 * @param point - {@link GeographicPoint} to create from.
	 */
	public PriorityNode(GeographicPoint point) {
		super(point.getX(), point.getY());
		this.weight = -1;
	}
	
	/**
	 * Copy constructor.
	 * @param pn - {@link PriorityNode} to copy.
	 */
	public PriorityNode(PriorityNode pn) {
		this(pn, pn.getWeight());
	}

	/**
	 * Returns the value for the distance calculated from previous 
	 * reference point to this internal {@link Node} point.
	 * @return The value for the distance calculated from previous 
	 * reference point to this internal {@link Node} point.
	 */
	public double getWeight() {
		synchronized (lock) {
			return weight;
		}
	}
	
	/**
	 * Sets the value for the distance calculated from previouse
	 * reference point to this internal {@link Node} point.
	 * @param newWeight - The new weight.
	 */
	public void setWeight(double newWeight) {
		synchronized (lock) {
			this.weight = newWeight;
		}
	}

//	/**
//	 * Returns the {@link Node}.
//	 * @return The {@link Node}.
//	 */
//	public Node getNode() {
//		return node;
//	}

	//-- Object methods --//
	@Override
	public String toString() {
		String nodeStr = super.toString();
		String result = "node: " + nodeStr + ", weight: " + weight;
		return result;
	}
}