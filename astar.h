#ifndef ASTAR
#define ASTAR

//callback object for Astar
template <typename GraphType, typename AstarType>
class Callback {
    protected:
        GraphType const& g;
    public:
        Callback( GraphType const& _g) : g(_g) {}
        virtual ~Callback() {}
        virtual void OnIteration( AstarType const& ) { }
        virtual void OnFinish( AstarType const& )    { }
};

template <typename GraphType, typename Heuristic> 
class Astar {
    public:
        ////////////////////////////////////////////////////////////
        Astar( GraphType const& _graph, Callback<GraphType,Astar> & cb ) : 
            graph(_graph),
            openlist(),
            closedlist(),
            solution(),
            callback(cb),
            start_id(0),
            goal_id(0)
        {}
        ////////////////////////////////////////////////////////////
        // this function should not be used in the actual code
        void sample_function(size_t s, size_t g) {
            start_id = s;
            goal_id  = g;
            openlist.clear();
            closedlist.clear();
            solution.clear();
            Heuristic heuristic;
            // note "const&", since Graph returns const references, we save a 
            // temporary
            typename GraphType::Vertex const& vertex_start = graph.GetVertex(start_id);
            typename GraphType::Vertex const& vertex_goal  = graph.GetVertex(goal_id);
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
        ////////////////////////////////////////////////////////////
        std::vector<typename GraphType::Edge> search(size_t s, size_t g) {
            start_id = s;
            goal_id  = g;
            openlist.clear();
            closedlist.clear();
            solution.clear();
            Heuristic heuristic;
            //heuristic from start to goal
            typename Heuristic::ReturnType h = heuristic( graph,graph.GetVertex(start_id),graph.GetVertex(goal_id) );






            while ( openlist.size() > 0 ) {
                callback.OnIteration( *this );






            }




            callback.OnFinish( *this );
            return solution;
        }
        ////////////////////////////////////////////////////////////////////////
    private:
        // do not modify the next 2 lines
        const GraphType &            graph;
        Callback<GraphType,Astar>  & callback;
        // the next 4 lines are just sugestions
        // OpenListContainer, ClosedListContainer, SolutionContainer are typedefed
        OpenListContainer            openlist;
        ClosedListContainer          closedlist;
        SolutionContainer            solution;
        size_t                       start_id,goal_id;
};

#endif
