#ifndef WV3_XPER_PERCEPTION_H_INCLUDE
#define WV3_XPER_PERCEPTION_H_INCLUDE

#include "wv3_xper_datatypes.h"
#include <cstddef>

// Macros for segmentation
#define XPER_LANE_MAX_N                   ((vfc::int32_t)5)
#define XPER_LANE_MAX_SEGMENT_N           ((vfc::int32_t)24)
#define XPER_ARROW_MAX_N                  ((vfc::int32_t)10)
#define XPER_STOPLINE_MAX_N               ((vfc::int32_t)5)
#define XPER_DECELERATE_MAX_N             ((vfc::int32_t)3)
#define XPER_CROSSWALK_MAX_N              ((vfc::int32_t)1)
#define XPER_CROSSWALK_POLYGON_MAX_N      ((vfc::int32_t)4)
#define XPER_ROADEDGE_MAX_N               ((vfc::int32_t)4)

// Macros for detection
#define XPER_OBJ_2D_DIMENSION_N           ((vfc::int32_t)2)
#define XPER_OBJ_3D_DIMENSION_N           ((vfc::int32_t)3)
#define XPER_OBJ_3D_VELOCITY_N            ((vfc::int32_t)2)
#define XPER_PARKSLOT_POINTS_N            ((vfc::int16_t)4)
#define XPER_OBJ_2D_MAX_N                 ((vfc::int16_t)100)
#define XPER_OBJ_3D_MAX_N                 ((vfc::int16_t)100)
#define XPER_PARKSLOT_MAX_N               ((vfc::int16_t)100)

constexpr size_t GRID_MAX_WIDTH  = 1920;
constexpr size_t GRID_MAX_HEIGHT = 1080;


namespace per {

//********************** Road Landmarks ********************//
enum class EXPerLaneType : vfc::uint8_t 
{
    lane_type_unknown = 0,
    lane_type_solid,
    lane_type_dashed,
    //solid-dot lane marker, dot-part at the left side of solid-part
    lane_type_solid_left_dashed,
    //solid-dot lane marker, dot-part at the right side of solid-part
    lane_type_solid_right_dashed,
    lane_type_merge,
    lane_type_dual_solid,
    lane_type_dual_dashed,
    lane_type_decelerate_middle,
    lane_type_decelerate_right,
    lane_type_decelerate_left,
    lane_type_fishbone
};

enum class EXPerLaneColor : vfc::uint8_t 
{
    lane_color_unknown = 0,
    lane_color_white,
    lane_color_yellow,
    lane_color_green
};

enum class EXPerLaneState : vfc::uint8_t 
{
    lane_state_off = 0,
    lane_state_on,
    lane_state_reduced,
    lane_state_hardware_err,
    lane_state_software_err,
    lane_state_fid_active,
    lane_state_blindness,
    lane_state_reserved
};

enum class EXPerLaneID : vfc::uint8_t 
{
    lane_id_unhnow = 0,
    lane_id_ego_left,
    lane_id_ego_right,
    lane_id_left_left,
    lane_id_right_right,
    lane_id_left_left_left,
    lane_id_right_right_right,
    lane_id_reserved
};

enum class EXPerLaneATT : vfc::uint8_t
{
    lane_att_normal = 0,
    lane_att_guide_area,
    lane_att_waiting_zoo,
    lane_att_reserved
};

struct CXPerLaneSegment
{
    bool                m_valid;
    EXPerLaneID         m_lane_id;
  
    vfc::float32_t             m_param_coff[4]; // y = c0*x + c1*x^2 + c2*x^3 + c3*x^3, 0->c0, 1->c1, 2->c2, 3->c3
    CXPerPoint2f        m_start_pt;
    CXPerPoint2f        m_end_pt;

    vfc::float32_t             m_width;
    EXPerLaneType       m_type;
    EXPerLaneColor      m_color;
    EXPerLaneATT        m_att;

    EXPerLaneState      m_state; 
}; 

//! describe events relevant with vehicle algorithm
enum class EXPerRoadEdgeType : vfc::uint8_t 
{
    roadedge_type_unknown = 0,
    roadedge_type_curbstone,
    roadedge_type_parking_car,
    roadedge_type_traffic_cone,
    roadedge_type_reserved
};

struct CXPerRoadEdgeSegment
{
    bool                m_valid;
    EXPerLaneID         m_lane_id;
  
    vfc::float32_t      m_param_coff[4]; // y = c0*x + c1*x^2 + c2*x^3 + c3*x^3, 0->c0, 1->c1, 2->c2, 3->c3
    CXPerPoint2f        m_start_pt;
    CXPerPoint2f        m_end_pt;

    vfc::float32_t      m_height;
    EXPerRoadEdgeType   m_type;
};


//! describe events relevant with vehicle algorithm
enum class EXPerArrowType : vfc::uint8_t 
{
    arrow_type_unknown = 0,
    arrow_type_left,
    arrow_type_right,
    arrow_type_straight,
    arrow_type_turnback,
    arrow_type_turn_left,
    arrow_type_turn_right,
    arrow_type_straight_left,
    arrow_type_straight_right,
    arrow_type_turnback_left,
    arrow_type_turnback_right,
    arrow_type_other
};

//! describe events relevant with vehicle algorithm
struct CXPerArrow 
{
    bool            m_valid;
    EXPerArrowType  m_type;
    CXPerPoint2f    m_centrioid;
    CXPerBBox2f     m_bbox;
};

//! describe events relevant with vehicle algorithm
struct CXPerCrosswalk 
{
    bool            m_valid;
    CXPerPoint2f    m_centrioid;
    vfc::int32_t    m_polygon_num;
    CXPerPolygon2f  m_polygon[XPER_CROSSWALK_POLYGON_MAX_N];
};


//! describe events relevant with vehicle algorithm
struct CXPerStopline 
{
    bool            m_valid;
    CXPerPoint2f    m_centrioid;
    CXPerBBox2f     m_bbox;
};

struct CXPerDecelerate 
{
    bool            m_valid;
    CXPerPoint2f    m_centrioid;
    CXPerBBox2f     m_bbox;
};

//********************** parking slot ********************//
enum class EXPerSlotMaterialType : vfc::uint8_t
{
    paint_concrete = 0,
    stereo,
    grass, 
    brick,
    slot_material_others
};

enum class EXPerSlotTargetType : vfc::uint8_t
{
    woman = 0,
    disabled, 
    normal
};

enum class EXPerSlotShapeType : vfc::uint8_t
{
    vertical = 0,
    parallel,
    slant
};

enum class EXPerSlotOccupiedType : vfc::uint8_t
{
    no_occupied = 0,
    self_occupied,
    vehicle_occupied,
    closed_locker_occupied,
    open_locker_occupied,
    bucket_occupied,
    other_occupied
};

enum class EXPerSlotEntranceLineType : vfc::uint8_t
{
    right_angle_anticlock = 0,
    slanted_anticlock_acute,
    slanted_anticlock_obtuse,
    right_angle_clock,
    slanted_clock_acute,
    slanted_clock_obtuse,
    invalid
};

struct CXPerSlotAttr
{
    EXPerSlotMaterialType m_eMaterialType;
    EXPerSlotTargetType   m_eTargetType;
    EXPerSlotShapeType    m_eShapeType;
    EXPerSlotOccupiedType m_eOccupiedType;
    EXPerSlotEntranceLineType m_eEntraneLineType;
};

struct CXPerSlotOut
{
    CXPerPoint2f   m_atPoints[XPER_PARKSLOT_POINTS_N]; // p1,p2,p3,p4
    CXPerSlotAttr  m_tAttribute;  // parking slot attributes
    vfc::int16_t   m_idPS;        // parking slot ID
};


struct CXPerParkingSlotsData 
{
    vfc::uint32_t  m_number{0};
    vfc::uint32_t  m_ara_sequence_number;
    vfc::uint64_t  m_timeStamp;
    vfc::int32_t   m_destSlotId;
    vfc::float32_t m_veloctiy;
    vfc::float32_t m_steerAng;
    vfc::float32_t m_position[3];
    vfc::float32_t m_orientation[4];
    vfc::int32_t   m_tracked_parking_slots; // number of tracked parking slot

    CXPerTPParkingSlot3D       m_parkingSlots3Dx[XPER_PARKSLOT_MAX_N];
    CXPerParkingSlotInfo       m_parkingSlotsInfo[XPER_PARKSLOT_MAX_N];
    CXPerTPParkingSlontUV      m_parkingSlotsUV[XPER_PARKSLOT_MAX_N];
    CXPerTPParkingSlotValid    m_parkingSlotValid[XPER_PARKSLOT_MAX_N];
};
///
struct CXPerEgoPoseData 
{
    vfc::uint32_t  m_number{0};
    vfc::uint32_t  m_ara_sequence_number;

    vfc::uint64_t  m_timeStamp;
    vfc::float32_t m_position[3];
    vfc::float32_t m_orientation[4];
};

struct CXPerBevFreeSpaceGridData
{
    vfc::uint32_t  m_number{0};
    vfc::uint32_t  m_ara_sequence_number;

    vfc::uint64_t  m_timeStamp;
    vfc::float32_t m_resolution;
    vfc::int32_t   m_width;
    vfc::int32_t   m_height;
    vfc::float32_t m_position[3];
    vfc::float32_t m_orientation[4];
    vfc::int8_t	   m_data[GRID_MAX_WIDTH*GRID_MAX_HEIGHT];
};

enum class EXPerObjCatagory : vfc::uint8_t 
{
    car = 0,
    pedestrain,
    cyclist,
    van,
    others
};

// ******************** 2D detection *****************//
enum class EXPer2DDimensionType : vfc::uint8_t
{
    height_2d = 0,
    width_2d
};

struct CXPer2DAtt
{
    bool m_truncated;
    bool m_occluded; 
  // vfc::float32_t truncated;
  // vfc::float32_t occluded; 
};

struct CXPer2DBBox
{ 
    CXPerPoint2f        m_center2D;  // 2D center points (x,y)
    vfc::float32_t      m_dimension2D[XPER_OBJ_2D_DIMENSION_N]; //hight, width
    CXPer2DAtt          m_att2D;
    vfc::float32_t      m_score2D;   // 2D detection probability
    EXPerObjCatagory    m_type2D;
    vfc::int16_t        m_id2D;
};


//*******************3D detection*******************//
enum class EXPer3DDimensionType : vfc::uint8_t
{
    length_3d = 0,
    height_3d,
    width_3d
};

enum class EXPer3DVelocityType : vfc::uint8_t
{  
    velocity_x_3d = 0,
    velocity_y_3d
};


struct CXPer3DAtt
{
    bool m_truncated;
    bool m_occluded; 
    // vfc::float32_t truncated;
    // vfc::float32_t occluded; 
};


struct CXPer3DBBox
{
    vfc::int16_t       m_id;
    EXPerObjCatagory   m_type;
    vfc::float32_t     m_score;
    vfc::float32_t     m_orientation; // global rotation angel
    vfc::float32_t     m_dimension[XPER_OBJ_3D_DIMENSION_N]; //length, hight, width
    CXPerPoint3f       m_location;  //(x,y,z) - object center in camera coordinate system
    CXPerPoint2f       m_center;    // projected object center in image coordinate system
    vfc::float32_t     m_velocity[XPER_OBJ_3D_VELOCITY_N]; //(vx, vy) in camera coordinate system
    CXPer3DAtt         m_att;       // occlusion / truncated status.
};


struct CXPerSystemOut 
{
    //frame number
    vfc::int32_t  m_frame_id;
    vfc::uint64_t m_timestamp;

    //ego-vehicle
    CXPerLaneSegment       m_lane_segment[XPER_LANE_MAX_SEGMENT_N]; // 6*3+6=24, each lane have max 3 segments.

    //ego-vehicle
    CXPerRoadEdgeSegment   m_roadedge_segment[XPER_ROADEDGE_MAX_N]; // 6*3+6=24, each roadedge have max 3 segments.

    //ego-vehicle
    CXPerArrow             m_arrow[XPER_ARROW_MAX_N];
  
    //ego-vehicle
    CXPerCrosswalk         m_crosswalk[XPER_CROSSWALK_MAX_N];

    //ego-vehicle
    CXPerStopline          m_stopline[XPER_STOPLINE_MAX_N];
  
    //ego-vehicle
    CXPerDecelerate        m_decelerate[XPER_DECELERATE_MAX_N];


    /* the semantic grid map include 3 channels, chan1:cls, chan2: height, chan3: cls_confidence, chan4: height_confidence
     *  data type: similar to CV_8UC4
     *  chan0 for cls: 0-drivable road area, 1-visible road area( supplement to drivable road area) 3-obstacle(include curb)
     *  chan1 for height: value range 0~255, representative 0~5m, height interval(m) = 5m/255=0.0196m
     *  chan2 for cls confidence: value range 0~100, higher is better
     *  chan3 for height estimation confidence: value range 0~100, higher is better
     *  grid resolution:
     *  range:
     *  ego-vehicle
    */
    //XPerMat               semantic_grid_map;  // daiding? cross_walk polygon, curb 
    CXPerPolygon2f         m_freespace;  

    CXPerSlotOut           m_slot[XPER_PARKSLOT_MAX_N];
    CXPer2DBBox            m_obj_2d[XPER_OBJ_2D_MAX_N];
    CXPer3DBBox            m_obj_3d[XPER_OBJ_3D_MAX_N];
};

}

#endif
