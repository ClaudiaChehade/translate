//VHM Abstract Output struct

#ifndef VHM_ABST_OUTPUT_VFC_HPP_INCLUDE
#define VHM_ABST_OUTPUT_VFC_HPP_INCLUDE

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_siunits_convenienttypes.hpp"
#include "vfc/container/vfc_carray.hpp"
#include "vfc/linalg/vfc_linalg_matrix33.hpp"

namespace pub_vhm_abst_vfc
{
    constexpr static vfc::int32_t CONTAINER_SIZE = 64;

    /*****************************CVhmAbstBufferOutput***********************************/
    struct CPose
    {
        vfc::CSI::si_milli_metre_i32_t m_positionX{0};                //!< X coordinate of the pose
        vfc::CSI::si_milli_metre_i32_t m_positionY{0};                //!< Y coordinate of the pose
        vfc::CSI::si_radian_f32_t m_yawAngle{0.f};                    //!< Yaw angle of the pose
    };  // CPose

    struct CSVhmAbstOdometry
    {
        vfc::CSI::si_milli_metre_i32_t m_positionX;
        vfc::CSI::si_milli_metre_i32_t m_positionY;
        vfc::CSI::si_radian_f32_t m_yawAngle;
        vfc::CSI::si_radian_per_second_f32_t m_yawAngleRate;     
        vfc::CSI::si_milli_second_ui32_t m_wicCanTime;
        vfc::CSI::si_micro_second_ui32_t m_r5TimeStamp;
        vfc::CSI::si_metre_per_second_f32_t m_velocity;
    };  // SVhmAbstOdometry

    struct CSVhmAbstBuffer
    {
        vfc::TCArray<CSVhmAbstOdometry, CONTAINER_SIZE> m_buffer;
        vfc::int32_t m_bufferSize;
        vfc::int32_t m_bufferIndex;

        vfc::float32_t m_offsetR5toWicTimeStamp;
        vfc::float32_t m_factorR5toWicTimeStamp;

        struct CSKalmanState 
        {
            vfc::float32_t m_position;
            vfc::float32_t m_velocity; 
            vfc::float32_t m_acceleration;
        } m_kalmanStateX, m_kalmanStateY, m_kalmanStateYaw;

        vfc::linalg::TMatrixMN<vfc::float32_t, 3, 3> m_kalmanPosterioriMatrixX;
        vfc::linalg::TMatrixMN<vfc::float32_t, 3, 3> m_kalmanPosterioriMatrixY;
        vfc::linalg::TMatrixMN<vfc::float32_t, 3, 3> m_kalmanPosterioriMatrixYaw;

        CPose m_reInitCoordinateSystemOrigin;
    };  // SVhmAbstBuffer

    struct CVhmAbstBufferOutput
    {
        enum{NUM_MTA_CHUNKS = 64};

        vfc::TCArray<vfc::uint8_t, 8> m_startMarker;

        CSVhmAbstBuffer m_vhmAbstBuffer;
        vfc::uint16_t   m_sequenceNumber;

        vfc::CSI::si_micro_second_ui32_t m_r5TimeStamp;
        vfc::int32_t  m_vhmAbstDebugCounter;
        vfc::uint64_t m_pma_sensorOput_nsTime_u64;
        vfc::TCArray<vfc::uint8_t, 8> m_endMarker;
    };  // CVhmAbstBufferOutput


    /*****************************CVhmAbstOutput***********************************/
    enum class EVehMoveDir: vfc::uint8_t 
    {
        forward     = 0,
        backward    = 1,
        unknown     = 2,
        standstill  = 3,
    };  // EVehMoveDir

    enum class EVhmUpdateState : vfc::uint8_t 
    {
        init        = 0,
        updated     = 1,
        not_updated = 2,
        reinit      = 3,
    }; // EVhmUpdateState

    struct CVhmAbstOutput //: daddy::CInterfaceBase_v3
    {
        //enum { ID = scom_ids::VHM_ABST_OUT };
        //enum { VERSION = 14 };
        struct CDrivingDirection 
        { 
            enum class EDrivingDirection : vfc::uint8_t
            { 
                forward = 0, 
                backward, 
                unknown, 
                standstill 
            }; 
        };
        typedef CDrivingDirection::EDrivingDirection EDrivingDirection;

        enum {
            NUM_MTA_CHUNKS = 64
        };

        struct CPose
        {
            vfc::CSI::si_milli_metre_i32_t m_positionX{0};                                 //!< X coordinate of the pose
            vfc::CSI::si_milli_metre_i32_t m_positionY{0};                                 //!< Y coordinate of the pose
            vfc::CSI::si_radian_f32_t      m_yawAngle{0.f};                                //!< Yaw angle of the pose
        };  // CPose

        vfc::char_t m_start[8];

        CPose                                       m_reInitFreePose;                   //!< VHM odometry pose (without coordinate system re-initialization)
        CPose                                       m_reInitCoordinateSystemOrigin;     //!< The current VHM coordinate system origin in the coordinate system after VHM_Init
        CPose                                       m_latestFilteredReInitFreePose;     //!< Latest (Kalman-filtered) re-init free vhm pose from vhm_abst_buffer

        vfc::CSI::si_metre_per_second_f32_t         m_velocity;         //!< Current velocity. Combined vectorial sum of Vx, Vy
        vfc::CSI::si_radian_per_second_f32_t        m_yawAngleRate;     //!< Current yaw angle rate
        vfc::uint32_t                               m_isDataSet;        //!< Is any data set? (To distinguish non-set data from zero data)
        vfc::int32_t                                m_debugCounter;     //!< Just for debugging reasons
        vfc::int32_t                                m_vhmDebugCounter;  //!< debug counter output from vhm modul (for debugging)
        vfc::uint32_t                               m_R5TimestampRun;   //!< R5 timestamp call of run method of runnable (for debugging)
        vfc::uint32_t                               m_D3Timestamp;      //!< D3 timestamp related to the odometry signal for debugging (currently not available)
        vfc::uint32_t                               m_WIC_CanTime;      //!< WIC Can Time (32bit), this is a Global Timestamp
        vfc::int32_t                                m_posXRaw;          //!< raw position output delivered in mm (with resets)
        vfc::int32_t                                m_posYRaw;          //!< raw position output delivered in mm (with resets)
        vfc::CSI::si_radian_f32_t                   m_yawAngleRaw;      //!< raw yaw angle from vhm output (with resets)

        //!< Curvature value from vhm output in current driving direction. Signed: Positive sign for left turn, negative for right turn.
        vfc::CSI::si_per_metre_f32_t                m_kappaRaw;
        //!< Curvature value from vhm output in backward driving direction. Signed: Positive sign for left turn, negative for right turn.
        vfc::CSI::si_per_metre_f32_t                m_kappaBwd;
        //!< Curvature value from vhm output in forward driving direction. Signed: Positive sign for left turn, negative for right turn.
        vfc::CSI::si_per_metre_f32_t                m_kappaFwd;
        //!< the largest curvature the vehicle can do in a left or right turn in forward motion
        vfc::CSI::si_per_metre_f32_t                m_kappaMaxForward;
        //!< The largest curvature the vehicle can do in a left or right turn in backward motion
        vfc::CSI::si_per_metre_f32_t                m_kappaMaxBackward;

        vfc::CSI::si_radian_f32_t                   m_steeringAngleRear;    //!< Steering angle rear in the current direction (always 0 for 2WS systems)
        vfc::CSI::si_radian_f32_t                   m_steeringAngleRearBwd; //!< Steering angle rear while going backward (always 0 for 2WS systems)
        vfc::CSI::si_radian_f32_t                   m_steeringAngleRearFwd; //!< Steering angle rear while going forward (always 0 for 2WS systems)
        vfc::CSI::si_metre_per_square_second_f32_t  m_vehLongitudinalAcceleration;  //!< Longitudinal acceleration value from vhm output
        vfc::CSI::si_metre_per_square_second_f32_t  m_vehLateralAcceleration;       //!< Lateral acceleration value from vhm output
        vfc::CSI::si_metre_per_second_f32_t         m_vehLongitudinalVelocity;      //!< Velocity to longitudinal direction from vhm output
        vfc::CSI::si_metre_per_second_f32_t         m_vehLateralVelocity;           //!< Velocity to lateral direction from vhm output

        EVhmUpdateState                             m_odometryUpdateState;      //!< Contains status information regarding odometry reset
        vfc::CSI::si_metre_f32_t                    m_curOdometry_X;            //!< Current PMA odometry X position (without resets)
        vfc::CSI::si_metre_f32_t                    m_curOdometry_Y;            //!< Current PMA odometry Y position (without resets)
        vfc::CSI::si_radian_f32_t                   m_curOdometry_YawAngle;     //!< Current PMA odometry Yaw Angle (without resets)
        vfc::CSI::si_metre_f32_t                    m_drivenDist;               //!< Absolute driven distance
        vfc::CSI::si_milli_metre_i32_t              m_signedDrivenDist;         //!< The signedly accumulated driven distance of the vehicle (with resets)
        vfc::uint32_t                               m_drivenDistRaw;            //!< Absolute driven distance [mm]
        vfc::uint8_t                                m_drivingDirection_e;       //!< Current driving direction, see EDrivingDirection for possible values
        EVehMoveDir                                 m_vehMoveDir;               //!< VHM vehicle motion direction based on WIC directions
        EVehMoveDir                                 m_vehDriveDir;              //!< VHM vehicle motion direction based on gear signal

        //!< Additional features which are available only for customers, where VHM2 is used:
        //!< Number of rotations around axis z ( overflow of m_curOdometry_YawAngle). Positive counterclockwise. VHM2 only
        vfc::int8_t                                 m_numberOfRotations;

        //!< This covariance values have to be interpreted as delta uncertainties between this and the previous odometry message. VHM2 only
        vfc::linalg::TMatrix33<vfc::float32_t>      m_poseCovarianceMatrix;

        //!< Pitch angle theta (rotation around y-axis; see roll-pitch-yaw definition in DIN 70001. VHM2 only
        vfc::CSI::si_radian_f32_t                   m_theta;
        vfc::uint64_t                               m_pma_sensorOput_nsTime_u64;
        vfc::char_t                                 m_end[8];
    };  // CVhmAbstOutput
}
#endif