package roadgraph;


/**
 * Wrapper for the {@link Node} to provide weighted value.
 * <br><br>
 * @author Lane Chasteen
 */
public class PriorityNode {
	//-- properties --//
	private final Node node;
	private volatile double weight;
	private final double length;
	/**
	 * Lock for {@link #weight}
	 */
	private final Object lock = new Object();

	//-- constructors --//
	/**
	 * Creates a new PriorityNode.
	 * @param node - {@link Node}.
	 * @param weight - Calculated value from previous point up to this point.
	 * @param length - The length from the starting point to the internal vertex.
	 */
	public PriorityNode(Node node, double weight, double length) {
		this.node = node;
		this.weight = weight;
		this.length = length;
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
	 * Returns the distance in KM between the {@code vertex} and
	 * the parent.
	 * @return The distance in KM between the {@code vertex} and 
	 * the parent.
	 */
	public double getLength() {
		return length;
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

	/**
	 * Returns the {@link Node}.
	 * @return The {@link Node}.
	 */
	public Node getNode() {
		return node;
	}

	//-- Object methods --//
	@Override
	public String toString() {
		String result = "node: " + node + ", length: " + String.valueOf(length) + ", weight: " + weight;
		return result;
	}
	
	@Override
	public int hashCode() {
		long bits = java.lang.Double.doubleToLongBits(getWeight());
		bits ^= java.lang.Double.doubleToLongBits(getLength()) * 31;
		return (((int) bits) ^ ((int) (bits >> 32))) + node.hashCode();
	}

	@Override
	public boolean equals(Object o) {
		if (o == this) {
			return true;
		}
		if (!(o instanceof PriorityNode)) {
			return false;
		}

		PriorityNode n = (PriorityNode) o;
		return (Double.compare(n.getWeight(), getWeight()) == 0) 
				&& (Double.compare(n.getLength(), getLength()) == 0)
				&& node.equals(n.node);
	}
}