package roadgraph;

import java.util.Comparator;

/**
 * Created by JeffZhang on 1/17/18.
 */
public class DijkstraComparator implements Comparator<MapNode> {
    @Override
    public int compare(MapNode node1, MapNode node2) {
        return Double.compare(node1.getShortDistance(), node2.getShortDistance());
    }
}
