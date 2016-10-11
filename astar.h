#ifndef ASTAR
#define ASTAR

#include <vector>
#include <queue>
#include <unordered_map>
#include <functional>
#include "graph.h"

#define NULL_PTR_ID -1

/* #ThingsToRememberAboutTheDriver
- Graph nodes are depicted as IDs
	- Instead of parent pointer or whatever, store ids
- Heuristics come with their own cost calculations
- Edge format = (StartingID, EndingID, weight)
	- Weights are the costs of the edges
*/

//callback object for Astar
template <typename GraphType, typename AstarType>
class Callback {
    protected:
        GraphType const& g;
    public:
        Callback( GraphType const& _g) : g(_g) {}
        virtual ~Callback() {}
        virtual void OnIteration( AstarType const&) {}
        virtual void OnFinish( AstarType const&)    {}
};

enum NodeStatus {
	InOpenList,
	InClosedList,
	Undiscovered
};

// Node representation of each vertex
template<typename Heuristic>
struct CustomNode {
	typedef typename Heuristic::ReturnType CostUnit;

	size_t parent;
	CostUnit givenCost;
	CostUnit heuristicCost;
	NodeStatus status;

	CustomNode() : parent(0), givenCost(std::numeric_limits<CostUnit>::max()), status(Undiscovered) {
	};
};

// Priority queue
template<typename Heuristic>
class PriorityQueue {

public:
	typedef size_t VertexID;
	typedef typename Heuristic::ReturnType GCost;
	typedef std::pair<GCost, VertexID> QueueElement; //std::pair compares with respect to the first element
													 // Insert cost value to the given id
	void Insert(GCost cost, VertexID vertexID) {
		myQueue.emplace(cost, vertexID);
	}

	VertexID FirstElement() {
		// The first element is guaranteed to be the smallest
		// compared with respect to the first element of std::pair
		QueueElement element = myQueue.top();
		myQueue.pop();
		return element.second; //second is the id of the vertex
	}

	bool IsEmpty() {
		return myQueue.empty();
	}

	unsigned int Size() {
		return myQueue.size();
	}

	void Clear() {
		myQueue = PriorityQueueType();
	}

private:
	typedef std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> PriorityQueueType;
	PriorityQueueType myQueue;

};

template <typename GraphType, typename Heuristic> 
class Astar {
    public:
		typedef CustomNode<Heuristic> Node;
		typedef size_t ID;
		typedef typename Heuristic::ReturnType DistanceUnit;
		
        ////////////////////////////////////////////////////////////
		Astar(GraphType const& _graph, Callback<GraphType, Astar> & cb) :
			graph(_graph),
			callback(cb),
			nodelist(),
			solution(),
			orderedContainer(),
			start_id(0),
			goal_id(0)
		{}

		typedef PriorityQueue<Heuristic> OrderedContainer; //just used for ordering
		typedef std::unordered_map<ID, Node> NodeContainer;
		typedef std::vector<typename GraphType::Edge> SolutionContainer;
		typedef typename GraphType::Vertex Vertex;

        ////////////////////////////////////////////////////////////
        // this function should not be used in the actual code
        void sample_function(size_t s, size_t g) {
            start_id = s;
            goal_id  = g;
			orderedContainer.Clear();
			nodelist.clear();
			solution.clear();
            Heuristic heuristic;
            // note "const&", since Graph returns const references, we save a 
            // temporary
			Vertex const& vertex_start = graph.GetVertex(start_id);
			Vertex const& vertex_goal  = graph.GetVertex(goal_id);
            //heuristic from start to goal
            typename Heuristic::ReturnType h = heuristic( graph, vertex_start, vertex_goal );
            std::cout << "Heuristic at start " << h << std::endl;

            // note "const&", since Graph returns const references, we save a 
            // temporary
			std::vector<typename GraphType::Edge> const& outedges = graph.GetOutEdges( vertex_goal );
			size_t outedges_size = outedges.size();
			for (size_t i = 0; i < outedges_size; ++i) {
				std::cout << "goal has a neighbor " << outedges[i].GetID2() << " at distance " << outedges[i].GetWeight() << std::endl;
			}

        }

		// #SearchAlgorithm
        ////////////////////////////////////////////////////////////
		SolutionContainer search(size_t s, size_t g) {
			start_id = s;
			goal_id = g;
			orderedContainer.Clear();
			nodelist.clear();
			solution.clear();
			Heuristic heuristic;
			//heuristic from start to goal
			DistanceUnit h = heuristic(graph, graph.GetVertex(start_id), graph.GetVertex(goal_id));
			size_t currentNodeId;
			DistanceUnit travelCost;

			// Create first node
			Node homeNode;
			homeNode.parent = -1;
			homeNode.givenCost = 0;
			homeNode.heuristicCost = h;

			// Register the node
			RegisterNode(homeNode, start_id);

			// TODO: Infinite loop if goal is not found
			while (orderedContainer.Size() > 0) {
				callback.OnIteration(*this);

				// Get the lowest cost element's ID
				currentNodeId = orderedContainer.FirstElement();

				if (currentNodeId == goal_id) {
					//fill solution and return
					FillReturnPath();
					break;
				}

				Node& currentNode = nodelist[currentNodeId];
				// Get the neighbors
				SolutionContainer const & edgeList = graph.GetOutEdges(currentNodeId);
				for (unsigned int i = 0; i < edgeList.size(); ++i) {
					// Get the ID of the neighborNode
					size_t neighborNodeID = edgeList[i].GetID2();
					Node& neighborNode = nodelist[neighborNodeID];

					// TODO: I'm assuming heuristic is always consistent
					// If not, this condition would fail so return to this if tests don't work
					// Because we might re-visit closed nodes
					if (neighborNode.status == InClosedList) {
						continue;
					}

					
					travelCost = currentNode.givenCost + edgeList[i].GetWeight();

					// New node
					if (neighborNode.status == Undiscovered) {
						DistanceUnit neighborHeuristic = heuristic(graph, graph.GetVertex(neighborNodeID), graph.GetVertex(goal_id));
						neighborNode.givenCost = travelCost;
						neighborNode.heuristicCost = neighborHeuristic;
						neighborNode.parent = currentNodeId;
						neighborNode.status = InOpenList;
						RegisterNode(neighborNode, neighborNodeID);
					}else if (travelCost < neighborNode.givenCost) {
						neighborNode.givenCost = travelCost;
						neighborNode.parent = currentNodeId;
						RegisterNode(neighborNode, neighborNodeID);
					}
					
				}

				
				currentNode.status = InClosedList;
			}

			callback.OnFinish(*this);
			return solution;
		}
        ////////////////////////////////////////////////////////////////////////
    private:
        // do not modify the next 2 lines
        const GraphType &            graph;
        Callback<GraphType,Astar>  & callback;
        // the next 4 lines are just suggestions
        // OpenListContainer, ClosedListContainer, SolutionContainer are typedef'ed
        NodeContainer            nodelist;
        SolutionContainer            solution;
		OrderedContainer			orderedContainer;
        size_t                       start_id,goal_id;

		// HELPER FUNCTIONS
		void RegisterNode(Node const & node, size_t id) {
			orderedContainer.Insert(node.givenCost, id);
			nodelist[id] = node;
		}

		void FillReturnPath() {
			size_t currentID = goal_id;
			size_t nextNodeID = goal_id;
			do {
				Node& currentNode = nodelist[currentID];
				SolutionContainer const & edgeList = graph.GetOutEdges(currentID);
				for (unsigned int i = 0; i < edgeList.size(); ++i) {
					nextNodeID = edgeList[i].GetID2();
					// If the node is currentNode's registered parent than that is the way I should be going
					if (nextNodeID == currentNode.parent) {
						solution.push_back(edgeList[i]);
						break;
					}
				}
				currentID = nextNodeID;
			} while (currentID != start_id);
		}
};

#endif
