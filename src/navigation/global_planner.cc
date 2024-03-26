#include <string>
#include "voronoi/voronoi_builder.hpp"
#include "voronoi/voronoi_diagram.hpp"
#include "vector_map/vector_map.h"
#include "shared/math/line2d.h"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"


using std::string;
using namespace math_util;
using namespace ros_helpers;

using namespace boost;
using namespace polygon;
using namespace detail;

namespace navigation {

class GlobalPlanner {
    public:
    // maybe constructor and just put the initialization stuff in that

    GlobalPlanner(const vector_map::VectorMap& map)  
    {
        vb_(new voronoi_builder);

        // construct the diagram builder 

        // insert map geometry into the diagram builder
        for (size_t i = 0; i < map.lines.size(); i++) {
            const line2f map_line = map.lines[i];
            vb_.insert_segment(
                map_line.p0.x(),
                map_line.p0.y(),
                map_line.p1.x(),
                map_line.p1.y(),
            )
        }
        

        // instantiate any other global variables we're using?

    };

    

    // overarching function to handle global planning process
    void run_global_planner() {
        



    }

    // function to build voronoi diagram
    void build_diagram() {

        



    }

    // function to run a*, smooth the path to be kinematically feasible?
    void plan_global_path() {

    }

    void visualize_voronoi(amrl_msgs::VisualizationMsg & viz_msg) {
        // draw the line segments on the viz_msg
        
    }

};

} // namespace