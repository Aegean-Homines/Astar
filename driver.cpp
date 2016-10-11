#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <cstring> //memset memcpy
#include <ctime>
#include "graph.h"
#include "astar.h"

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
    struct Test0Vertex {
        Test0Vertex (size_t _id=0, float _x = 0.0f, float _y = 0.0f, float _z = 0.0f) 
            : id(_id),x(_x),y(_y),z(_z)   {  
                //std::cout << "Test0Vertex("<<id<<") called\n";
            }
        size_t ID() const { return id; }
        float X() const { return x; }
        float Y() const { return y; }
        float Z() const { return z; }
        bool operator< ( const Test0Vertex& rhs) const {
            return id<rhs.id;
        }
        private:
        size_t id;
        float x,y,z;
    };

struct Test0Edge {
    Test0Edge(size_t id1, size_t id2, float weight) : 
        id1(id1),id2(id2),weight(weight) {}
    size_t GetID1() const { return id1; }
    size_t GetID2() const { return id2; }
    float  GetWeight() const { return weight; }
    private:
    size_t id1;
    size_t id2;
    float weight;
};

template <typename GraphType>
struct Test0Heuristic {
    typedef float ReturnType;
    float operator () (const GraphType &, 
            const typename GraphType::Vertex & v1,  
            const typename GraphType::Vertex &) const 
    {
        float h = 0.0f;
        switch ( v1.ID() ) {
            case 1: h = 6.0f; break;
            case 2: h = 7.0f; break;
        };
        return h;
    }
};

//#FirstTest
void test0() {
    /*       (B)
             /     \
             1       1
             /         \
             (A)-1-(C)-2-(E)--5--(F)
             \         /
             1       3
             \     /
             (D)

             h(B)=6
             h(C)=7
             h(all other nodes)=0
     */

    enum V {A,B,C,D,E,F};

    //make sure algorithm works with all permutation of edges AB AC AD
    //that is correctness doesn't depend on tie-breaking
    std::vector<int> order;
    order.push_back(1);
    order.push_back(2);
    order.push_back(3);
    float worst_solution = 0;
    //long worst_time = 0;
    do {
        //std::cout << order[0] << " "  << order[1] << " " << order[2] << std::endl;
        typedef Graph< Test0Vertex,Test0Edge >        GraphType;
        typedef Astar< GraphType,Test0Heuristic<GraphType> >  AstarSearchType;
        typedef Callback< GraphType,AstarSearchType >        CallbackType;

        GraphType graph;
        graph.InsertVertex( GraphType::Vertex(A) );
        graph.InsertVertex( GraphType::Vertex(B) );
        graph.InsertVertex( GraphType::Vertex(C) );
        graph.InsertVertex( GraphType::Vertex(D) );
        graph.InsertVertex( GraphType::Vertex(E) );
        graph.InsertVertex( GraphType::Vertex(F) );
        graph.InsertEdge( GraphType::Edge(A,static_cast<V>(order[0]),1));
        graph.InsertEdge( GraphType::Edge(static_cast<V>(order[0]),A,1));
        graph.InsertEdge( GraphType::Edge(A,static_cast<V>(order[1]),1));
        graph.InsertEdge( GraphType::Edge(static_cast<V>(order[1]),A,1));
        graph.InsertEdge( GraphType::Edge(A,static_cast<V>(order[2]),1));
        graph.InsertEdge( GraphType::Edge(static_cast<V>(order[2]),A,1));
        graph.InsertEdge( GraphType::Edge(B,E,1) );
        graph.InsertEdge( GraphType::Edge(E,B,1) );
        graph.InsertEdge( GraphType::Edge(C,E,2) );
        graph.InsertEdge( GraphType::Edge(E,C,2) );
        graph.InsertEdge( GraphType::Edge(D,E,3) );
        graph.InsertEdge( GraphType::Edge(E,D,3) );
        graph.InsertEdge( GraphType::Edge(E,F,5) );
        graph.InsertEdge( GraphType::Edge(F,E,5) );

        CallbackType callback(graph);
        AstarSearchType astar(graph,callback);
        std::vector<Test0Edge> solution = astar.search(A,F);
        float cost = 0.0f;
        std::vector<GraphType::Edge>::reverse_iterator rit;
        for(rit=solution.rbegin();rit!=solution.rend();++rit) {
            cost += rit->GetWeight();
        }
        if (worst_solution < cost) worst_solution = cost;
    } while (next_permutation(order.begin(),order.end()));
    std::cout << "solution cost " << worst_solution << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
struct Test1Heuristic {
    typedef float ReturnType;
    float operator () (const GraphType &, 
            const typename GraphType::Vertex & v1,  
            const typename GraphType::Vertex &) const {
        float h = 0.0f;
        switch ( v1.ID() ) {
            case 1: h = 10.0f; break;
        };
        return h;
    }
};


void test1() {
    /*        (B)
              /     \
              2       1
              /         \
              (A)         (D)-1-(E)--10--(F)
              \         /
              1       4
              \     /
              (C)
              h(B)=10
              h(all other nodes)=0 
     */
    typedef Graph< Test0Vertex,Test0Edge >        GraphType;
    typedef Astar< GraphType,Test0Heuristic<GraphType> >  AstarSearchType;
    typedef Callback< GraphType,AstarSearchType >        CallbackType;

    GraphType graph;

    enum V {A,B,C,D,E,F};

    {
        graph.InsertVertex( GraphType::Vertex(A) );
        graph.InsertVertex( GraphType::Vertex(B) );
        graph.InsertVertex( GraphType::Vertex(C) );
        graph.InsertVertex( GraphType::Vertex(D) );
        graph.InsertVertex( GraphType::Vertex(E) );
        graph.InsertVertex( GraphType::Vertex(F) );
        graph.InsertEdge( GraphType::Edge(A,B,2));
        graph.InsertEdge( GraphType::Edge(B,A,2));
        graph.InsertEdge( GraphType::Edge(A,C,1));
        graph.InsertEdge( GraphType::Edge(C,A,1));
        graph.InsertEdge( GraphType::Edge(B,D,1));
        graph.InsertEdge( GraphType::Edge(D,B,1));
        graph.InsertEdge( GraphType::Edge(C,D,4) );
        graph.InsertEdge( GraphType::Edge(D,C,4) );
        graph.InsertEdge( GraphType::Edge(D,E,1) );
        graph.InsertEdge( GraphType::Edge(E,D,1) );
        graph.InsertEdge( GraphType::Edge(E,F,10) );
        graph.InsertEdge( GraphType::Edge(F,E,10) );
    }

    CallbackType callback(graph);
    AstarSearchType astar(graph,callback);
    std::vector<Test0Edge> solution = astar.search(A,F);
    float cost = 0.0f;
    std::vector<GraphType::Edge>::reverse_iterator rit;
    for(rit=solution.rbegin();rit!=solution.rend();++rit) {
        cost += rit->GetWeight();
    }
    std::cout << "solution cost " << cost << std::endl;
}
//////////////////////////////////////////////////////////////////////////////
// END OF SIMPLE TESTS
//////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
template <typename Grid>
void createGridFromBMP(
        Grid& g,      //graph - empty
        char const * bmp_filename,
        int & width, int & height
        )
{
    std::ifstream f( bmp_filename, std::ifstream::binary );
    if ( !f) { return; }

    char buffer[4];
    f.seekg (f.beg+10);
    //read offset
    f.read (buffer,4);
    //int offset = *reinterpret_cast<int*>( buffer ); //warning: dereferencing type-punned pointer will break strict-aliasing rules
    int offset = 0;
    offset |= 0xff000000 & (buffer[3]<<24);
    offset |= 0x00ff0000 & (buffer[2]<<16);
    offset |= 0x0000ff00 & (buffer[1]<<8);
    offset |= 0x000000ff & (buffer[0]);

    f.seekg (f.beg+18);
    //read width
    f.read (buffer,4);
    width = 0;
    width |= 0xff000000 & (buffer[3]<<24);
    width |= 0x00ff0000 & (buffer[2]<<16);
    width |= 0x0000ff00 & (buffer[1]<<8);
    width |= 0x000000ff & (buffer[0]);

    //read height
    f.read (buffer,4);
    height = 0;
    height |= 0xff000000 & (buffer[3]<<24);
    height |= 0x00ff0000 & (buffer[2]<<16);
    height |= 0x0000ff00 & (buffer[1]<<8);
    height |= 0x000000ff & (buffer[0]);

    //skip to data
    f.seekg ( f.beg+offset );

    //calculate buffer size for data, must be a multiple of 4
    int size = 3*width;
    if ( size%4 ) size += (4 - (size%4) );
    char *buffer2 = new char[size];

    //std::cout << std::endl;
    bool** field = new bool*[height]; //row pointers
    for ( int h=0; h<height; ++h ) {
        field[h] = new bool[width];
        f.read( buffer2,size );
        //read 
        for ( int w=0; w<width; ++w ) {
            int color = ( *(buffer2+3*w+2)<<16 ) + ( *(buffer2+3*w+1)<<8 ) + ( *(buffer2+3*w) );
            //if ( color != 0 ) std::cout << "."; else std::cout << "#";
            field[h][w] = ( color != 0 ); //true=not black = exists
        }
        //std::cout << std::endl;
    }

#ifdef DOT_OUT
    std::cout << "graph G {\n";
#endif
    //y=h,x=w
    for ( int h=0; h<height; ++h ) {
        for ( int w=0; w<width; ++w ) {
            if ( field[h][w] ) {
                g.InsertVertex( typename Grid::Vertex( width * h+w ,w,h,0.0f) );
#ifdef DOT_OUT
                std::cout << width*h+w 
                    << " [label=\"" << width*h+w 
                    //<< "(" << x << "," << y << ")"
                    <<"\","
                    << "pos=\"" << w << "," << h << "\""
                    << "pin=true, fontsize=9];" << std::endl;
#endif
            }
        }
    }

    //check right and down edges
    for ( int h=0; h<height-1; ++h ) {
        for ( int w=0; w<width-1; ++w ) {
            if ( field[h][w] ) {
                if ( field[h][w+1] ) {
                    g.InsertEdge( typename Grid::Edge( width * h+w,      width * h + (w+1), 1.0) );
                    g.InsertEdge( typename Grid::Edge( width * h+(w+1),  width * h + w, 1.0) );
#ifdef DOT_OUT
                    std::cout << width*h+w << "--" << width*h+w+1 << ";" << std::endl;
#endif
                }
                if ( field[h+1][w] ) {
                    g.InsertEdge( typename Grid::Edge( width * (h+1)+w,  width * h + w, 1.0) );
                    g.InsertEdge( typename Grid::Edge( width * h+w,  width * (h+1) + w, 1.0) );
#ifdef DOT_OUT
                    std::cout << width*h+w << "--" << width*(h+1)+w << ";" << std::endl;
#endif
                }
            }
        }
    }

    //chck down in right-most 
    for ( int h=0; h<height-1; ++h ) {
        int w = width-1;
        if ( field[h][w] && field[h+1][w] ) {
            g.InsertEdge( typename Grid::Edge( width * (h+1)+w,  width * h + w, 1.0) );
            g.InsertEdge( typename Grid::Edge( width * h+w,  width * (h+1) + w, 1.0) );
#ifdef DOT_OUT
            std::cout << width*h+w << "--" << width*(h+1)+w << ";" << std::endl;
#endif
        }
    }

    //check right at bottome
    for ( int w=0; w<width-1; ++w ) {
        int h = height-1;
        if ( field[h][w] && field[h][w+1] ) {
            g.InsertEdge( typename Grid::Edge( width * h+w,      width * h + (w+1), 1.0) );
            g.InsertEdge( typename Grid::Edge( width * h+(w+1),  width * h + w, 1.0) );
#ifdef DOT_OUT
            std::cout << width*h+w << "--" << width*h+w+1 << ";" << std::endl;
#endif
        }
    }

    delete [] buffer2;
    //std::cout << std::endl;

    for ( int h=0; h<height; ++h ) { delete [] field[h]; }
    delete [] field;
#ifdef DOT_OUT
    std::cout << "}\n";
#endif
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Heuristics
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//#include "cycle.h" //ticks and elapsed
struct EdgeGrid {
    EdgeGrid(size_t id1, size_t id2, float weight) : id1(id1),id2(id2),weight(weight) {}
    size_t GetID1() const { return id1; }
    size_t GetID2() const { return id2; }
    float  GetWeight() const { return weight; }
    private:
    size_t id1;
    size_t id2;
    float weight; 
};

struct VertexGrid {
    VertexGrid (size_t _id=0, float _x=0.0f, float _y=0.0f, float _z = 0.0f) : id(_id),x(_x),y(_y),z(_z) 	{	}
    size_t ID() const { return id; }
    bool operator< ( const VertexGrid& rhs) const { return id<rhs.id; }
    float X() const { return x; }
    float Y() const { return y; }
    float Z() const { return z; }
    private:
    size_t id;
    float x,y,z;
};


// #HEURISTICDEFINITIONS
//standard Euclidian heuristic, admissible and consistent when num/den==1. num
//and den allow a multiplier which may speed up the search when num/den>1,
//notice that search may become suboptimal
template <typename GraphType, int num =1, int den=1>
// num / den  is a heuristic multiplier (inadmissible heuristic if num > den )
struct EuclidianHeuristic {
    typedef float ReturnType;
    ReturnType operator () (const GraphType &, 
            const typename GraphType::Vertex & v1,  
            const typename GraphType::Vertex & v2) const {
        float x1 = v1.X();
        float y1 = v1.Y();
        float x2 = v2.X();
        float y2 = v2.Y();
        return static_cast<float>(num)/den * std::sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
    }
};

//standard Manhattan heuristic, admissible and consistent
//see ManhattanHeuristic comment
template <typename GraphType, int num=1, int den=1>
struct ManhattanHeuristic {
    typedef float ReturnType;
    ReturnType operator () (const GraphType &, 
            const typename GraphType::Vertex & v1,  
            const typename GraphType::Vertex & v2) const {
        //Manhattan distance
        return static_cast<float>(num)/den * std::abs (v1.X()-v2.X()) + std::abs (v1.Y()-v2.Y());
    }
};

//define a special std::less for "unordered" pairs
namespace std {
    template <typename T>
        struct less<std::pair<T,T> > {
            //a non-const version - may need to swap vertices
            bool operator() ( std::pair<T,T> p1, std::pair<T,T> p2) const {
                //first order each pair - smaller id first
                if ( p1.first < p1.second ) { std::swap(p1.first,p1.second); }
                if ( p2.first < p2.second ) { std::swap(p2.first,p2.second); }

                //now use lexicographical order
                return ( (p1.first < p2.first) || 
                        ( !(p2.first < p1.first ) && p1.second < p2.second ) );
            }
        };
}

//heuristic calculates Manhattan distance and than multiplies it by a random
//number between o and 1. Obviously it's still an admissible heuristic, but
//there is a very high chance that it's not consistent.
template <typename GraphType>
struct ManhattanRandomHeuristic {
    typedef float ReturnType;
    typedef std::pair<typename GraphType::Vertex,typename GraphType::Vertex> VertexPair;
    //keep track of already calculated heuristic values,
    //this way if client resubmits a query, we return exactly the same 
    //value as previous time. Otherwise SOME implementations of A* may fail.

    std::map<VertexPair,ReturnType> heuristic_values;
    typename GraphType::Vertex goal;

    ManhattanRandomHeuristic() : heuristic_values(), goal() {
        std::srand ( std::time(NULL) );
    }

    ReturnType operator () (const GraphType &, 
            const typename GraphType::Vertex & v1,  
            const typename GraphType::Vertex & v2) {

        VertexPair p = std::pair<typename GraphType::Vertex,typename GraphType::Vertex>(v1,v2);

        typename std::map<VertexPair,ReturnType>::const_iterator 
            it = heuristic_values.find( p );

        if ( it != heuristic_values.end() ) {
            return it->second; //return previously calculated value
        } else { 
            return //new pair, so need to calculate heuristic
                heuristic_values[ p ] = 
                static_cast<float>( std::abs (v1.X()-v2.X()) + std::abs (v1.Y()-v2.Y()) ) * (std::rand () % 1000) / 1000; //1000 is radom generator resolution
        }
        //cassert (1); //should never get to this point
        //return  -1;
    }
};

//constant zero heuristic, admissible and consistent
template <typename GraphType>
struct ZeroHeuristic {
    typedef float ReturnType;
    ReturnType operator () (const GraphType &, 
            const typename GraphType::Vertex &,  
            const typename GraphType::Vertex &) const {
        return 0;
    }
};



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Callbacks
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//callback object for Astar that prints open and closed lists
template <typename GraphType, typename AstarType>
class Callback2 : public Callback<GraphType,AstarType> {
    public:
        Callback2( GraphType const& _g) : Callback<GraphType,AstarType>(_g) {
			
		}
        virtual void OnIteration( AstarType const& search ) { 
            typename AstarType::OpenListContainer::const_iterator it_op_b = search.GetOpenListBegin(), it_op_e = search.GetOpenListEnd();
            std::cout << "===============================================\n";
			std::cout << "Openlist: " << std::endl;
            for ( ; it_op_b != it_op_e; ++it_op_b) {
                typename GraphType::Vertex const& v = this->g.GetVertex( (*it_op_b)->);
				std::cout << it_op_b->id << std::endl;
            }
            std::cout << std::endl;
        }
        virtual void OnFinish( AstarType const& search )    {
            //get top element and trace it back to start of the search
            //size_t top_id = search.GetGoalID();

            /*std::vector<size_t> trace = search.TraceToStart( top_id );
            std::cout << "Solution ";
            std::vector<size_t>::const_iterator 
                it_trace_b = trace.begin(),
                           it_trace_e = trace.end();
            for ( ; it_trace_b != it_trace_e; ++it_trace_b) {
                typename GraphType::Vertex const& v = this->g.GetVertex( *it_trace_b );
                std::cout << "("<<v.X()<<","<<v.Y()<<")--";
            }
            std::cout << std::endl;*/
        }
};

//callback object for Astar that writes open and closed lists into bmp
template <typename GraphType, typename AstarType>
class Callback3 : public Callback<GraphType,AstarType> {
    char * bmp_prefix; // all bytes that are NOT data
    char * bmp_data;   // all bytes that are     data
    char * bmp_suffix; // all bytes that are NOT data
    int bmp_prefix_size, bmp_suffix_size;
    int width, height, padding/*pad each row to 4 bytes*/;
    int counter;
    Callback3( Callback3 const& );
    Callback3& operator=( Callback3 const& );
    public:
    ~Callback3() {
        delete [] bmp_prefix;
        delete [] bmp_data;
        delete [] bmp_suffix;
    }
    Callback3( GraphType const& _g, char const * bmp_filename ) : 
        Callback<GraphType,AstarType>(_g), 
        bmp_prefix(),bmp_data(),bmp_suffix(),
        bmp_prefix_size(),bmp_suffix_size(),width(),height(),padding(),counter()
    {
        std::ifstream f( bmp_filename, std::ifstream::binary );
        if ( !f) { return; }

        char buffer[4];
        f.seekg (f.beg+10);
        //read offset
        f.read (buffer,4);
        int offset = 0;
        offset |= 0xff000000 & (buffer[3]<<24);
        offset |= 0x00ff0000 & (buffer[2]<<16);
        offset |= 0x0000ff00 & (buffer[1]<<8);
        offset |= 0x000000ff & (buffer[0]);
        //std::cout << "offset = " << offset << std::endl;

        //create prefix for future writes
        bmp_prefix_size = offset;
        bmp_prefix = new char[bmp_prefix_size];
        f.seekg( f.beg );
        f.read( bmp_prefix, bmp_prefix_size );


        //read width
        f.seekg (f.beg+18);
        f.read (buffer,4);
        width = 0;
        width |= 0xff000000 & (buffer[3]<<24);
        width |= 0x00ff0000 & (buffer[2]<<16);
        width |= 0x0000ff00 & (buffer[1]<<8);
        width |= 0x000000ff & (buffer[0]);
        //std::cout << "width = " << width << std::endl;

        //read height
        f.read (buffer,4);
        height |= 0xff000000 & (buffer[3]<<24);
        height |= 0x00ff0000 & (buffer[2]<<16);
        height |= 0x0000ff00 & (buffer[1]<<8);
        height |= 0x000000ff & (buffer[0]);
        //std::cout << "height = " << height << std::endl;

        //skip to data
        f.seekg ( f.beg+offset );

        //calculate buffer size for data, must be a multiple of 4
        int size = 3*width;
        padding = 4 - (size%4);
        if ( size%4 ) { size += padding; }
        else          { padding = 0;     }

        //std::cout << "padding = " << padding << std::endl;
        //std::cout << "size = " << size << std::endl;

        //read map data
        bmp_data = new char[size*height];
        f.read( bmp_data, size*height );

        /*
        //calculate total filesize 
        f.seekg(0, std::ifstream::end);
        int filesize = f.tellg();

        //write suffix
        //skip to the end of data
        bmp_suffix_size = filesize-size*height;
        bmp_suffix = new char[bmp_suffix_size];
        f.seekg ( f.beg+size*height );
        f.read( bmp_suffix, bmp_suffix_size );
         */
    }

    ////////////////////////////////////////////////////////////////////////
    virtual void OnIteration( AstarType const& search ) {
		std::cout << "OnIteration of Callback3" << std::endl;
        //create next "frame_counter.bmp"
        char current_frame[128];
        sprintf(current_frame,"frame_%06u.bmp",static_cast<unsigned>(counter++) );
        std::ofstream frame( current_frame, std::ifstream::out | std::ifstream::binary );

        //write prefix
        frame.write( bmp_prefix, bmp_prefix_size );

        char** frame_data  = new char*[height];
        char*  frame_data2 = new char[(3*width+padding)*height];
        for ( int h=0; h<height; ++h) {
            frame_data[h] = frame_data2+(3*width+padding)*h;
        }

        //std::cout << "writing into frame_data " << (3*width+padding)*height << std::endl;
        memcpy( frame_data2, bmp_data, (3*width+padding)*height );

        char unsigned openlistelement[3]   = {255,0,0};
        char unsigned closedlistelement[3] = {0,255,0};
        char unsigned pathelement[3]       = {0,0,255};
        //write openlist
        typename AstarType::OpenListContainer::const_iterator it_op_b = search.GetOpenListBegin(), it_op_e = search.GetOpenListEnd();
        for ( ; it_op_b != it_op_e; ++it_op_b) {
            typename GraphType::Vertex const& v = this->g.GetVertex( it_op_b->id );
            int w=v.X();
            int h=v.Y();
            memcpy( frame_data[h]+3*w, openlistelement, 3);
        }

        typename AstarType::ClosedListContainer::const_iterator it_cl_b = search.GetClosedListBegin(), it_cl_e = search.GetClosedListEnd();
        for ( ; it_cl_b != it_cl_e; ++it_cl_b) {
            typename GraphType::Vertex const& v = this->g.GetVertex( it_cl_b->second.id );
            int w=v.X();
            int h=v.Y();
            memcpy( frame_data[h]+3*w, closedlistelement, 3);
        }


        typename AstarType::Node const& top = search.GetTopElement();
        typename GraphType::Vertex const& vertex_top = this->g.GetVertex( top.id );
        //color top element
        int w_top = vertex_top.X();
        int h_top = vertex_top.Y();
        memcpy( frame_data[h_top]+3*w_top, pathelement, 3);

        size_t parent_id = top.parent; //get parent - it's on closed list
        std::vector<size_t> trace = search.TraceToStart( parent_id );
        std::vector<size_t>::const_iterator 
            it_trace_b = trace.begin(), it_trace_e = trace.end();
        for ( ; it_trace_b != it_trace_e; ++it_trace_b) {
            typename GraphType::Vertex const& v = this->g.GetVertex( *it_trace_b );
            int w=v.X();
            int h=v.Y();
            memcpy( frame_data[h]+3*w, pathelement, 3);
        }


        //write data and delete
        frame.write( frame_data2, height*(3*width+padding) );
        delete [] frame_data;
        delete [] frame_data2;
    }

    virtual void OnFinish( AstarType const& search )    {
        //get top element and trace it back to start of the search
        size_t top_id = search.GetGoalID();

        std::vector<size_t> trace = search.TraceToStart( top_id );
        std::cout << "Solution ";
        std::vector<size_t>::const_iterator 
            it_trace_b = trace.begin(),
                       it_trace_e = trace.end();
        for ( ; it_trace_b != it_trace_e; ++it_trace_b) {
            typename GraphType::Vertex const& v = this->g.GetVertex( *it_trace_b );
            std::cout << "("<<v.X()<<","<<v.Y()<<")--";
        }
        std::cout << std::endl;
    }
};





template< typename GraphType, typename AstarSearchType >
void run( char const * bmp_name ) {
    int width=0, height=0;
    GraphType graph;
    createGridFromBMP( graph, bmp_name, width, height );

    size_t start_id=0+0*width, goal_id=(width-1)+(height-1)*width;
    //             x0 y0                 x1         y1

    //change to 1 to generate BMP for each iteration (SLOW)
    //check out Callback2 - may be used to insert your own debug stuff
#if 0
    typedef Callback3< GraphType,AstarSearchType >       CallbackType;
    CallbackType callback(graph, bmp_name);
#else
    typedef Callback< GraphType,AstarSearchType >        CallbackType;
    CallbackType callback(graph);
#endif

    AstarSearchType astar(graph, callback);

    std::cout << "Start search (" << start_id << " to " << goal_id << ")" << std::endl;
    //clock_t start = std::clock();
    std::vector<typename GraphType::Edge> solution = astar.search(start_id,goal_id);
    //clock_t finish = std::clock();

    float cost = 0.0f;

    typename std::vector<typename GraphType::Edge>::reverse_iterator rit;
    for(rit=solution.rbegin();rit!=solution.rend();++rit) {
        cost += rit->GetWeight();
    }
    std::cout << "Solution cost " << cost /*<< "    algorithm time " << finish-start */<< std::endl;
}



void test2() {
    typedef Graph< VertexGrid,EdgeGrid >              Grid;
    typedef Astar< Grid, ManhattanRandomHeuristic<Grid> >   AstarSearchType;
    run< Grid,AstarSearchType>( "map1.bmp" ); 
}

void test3() {
    typedef Graph< VertexGrid,EdgeGrid >              Grid;
    typedef Astar< Grid, EuclidianHeuristic<Grid> >   AstarSearchType;
    run< Grid,AstarSearchType>( "map1.bmp" ); 
}

void test4() {
    typedef Graph< VertexGrid,EdgeGrid >              Grid;
    typedef Astar< Grid, ManhattanHeuristic<Grid> >   AstarSearchType;
    run< Grid,AstarSearchType>( "map1.bmp" ); 
}

void test5() {
    typedef Graph< VertexGrid,EdgeGrid >              Grid;
    typedef Astar< Grid, ManhattanRandomHeuristic<Grid> >   AstarSearchType;
    run< Grid,AstarSearchType>( "map2.bmp" ); 
}

void test6() {
    typedef Graph< VertexGrid,EdgeGrid >              Grid;
    typedef Astar< Grid, EuclidianHeuristic<Grid> >   AstarSearchType;
    run< Grid,AstarSearchType>( "map2.bmp" ); 
}

void test7() {
    typedef Graph< VertexGrid,EdgeGrid >              Grid;
    typedef Astar< Grid, ManhattanHeuristic<Grid> >   AstarSearchType;
    run< Grid,AstarSearchType>( "map2.bmp" ); 
}

void test8() {
    typedef Graph< VertexGrid,EdgeGrid >              Grid;
    typedef Astar< Grid, ManhattanHeuristic<Grid> >   AstarSearchType;
    run< Grid,AstarSearchType>( "map3.bmp" ); 
}

void test9() {
    typedef Graph< VertexGrid,EdgeGrid >              Grid;
    typedef Astar< Grid, ManhattanHeuristic<Grid> >   AstarSearchType;
    run< Grid,AstarSearchType>( "map4.bmp" ); 
}

void test10() {
    typedef Graph< VertexGrid,EdgeGrid >              Grid;
    typedef Astar< Grid, ManhattanHeuristic<Grid> >   AstarSearchType;
    run< Grid,AstarSearchType>( "map5.bmp" ); 
}

void (*pTests[])() = {
    test0,test1,test2,test3,test4,
    test5,test6,test7,test8,test9,test10
};


int main(int argc, char *argv[] ) {
    if (argc >1) {
        int test = 0;
        std::sscanf(argv[1],"%i",&test);
        try {
            pTests[test]();
        } catch( const char* msg) {
            std::cerr << msg << std::endl;
        }
    }

    return 0;
}
