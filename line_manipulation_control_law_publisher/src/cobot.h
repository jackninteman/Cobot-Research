#ifndef COBOT_H
#define COBOT_H

//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------

#include <cstdint>
#include <vector>

//------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------

// #define CIRCLE

#define HYBRID

#define NUM_GEO_MODES (6)

#define FREE_MODE_IDX (0)
#define LINE_MODE_IDX (1)
#define PLANE_2D_MODE_IDX (2)
#define CIRCLE_MODE_IDX (3)
#define SPLINE_MODE_IDX (4)
#define PLANE_3D_MODE_IDX (5)

#define DEFAULT_MODE std::vector<uint8_t>({1, 0, 0, 0, 0, 0})
#define MODE_STRINGS std::vector<std::string>({"FREE", "LINE", "PLANE", "CIRCLE", "SPLINE", "3D PLANE"})

#define NUM_ROT_MODES (2)

#define UPRIGHT_IN_WS_IDX (0)
#define TANGENT_TO_SHAPE_IDX (1)

#define DEFAULT_ROT_MODE std::vector<uint8_t>({1, 0})
#define ROT_MODE_STRINGS std::vector<std::string>({"PARALLEL TO WS", "TANGENT TO SHAPE"})

#define NUM_CONTROL_MODES (2)

#define CROSSTRACK_IDX (0)
#define WALL_IDX (1)

#define DEFAULT_CONTROL_MODE std::vector<uint8_t>({1, 0})
#define CONTROL_MODE_STRINGS std::vector<std::string>({"CROSSTRACK", "WALL"})

//------------------------------------------------------------------------------
// VARIABLES
//------------------------------------------------------------------------------

#endif