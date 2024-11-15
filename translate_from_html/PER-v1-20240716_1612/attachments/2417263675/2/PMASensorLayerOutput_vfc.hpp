//PAM Sensor Layer Output struct

#ifndef PMAAENSORLAYEROUTPUT_VFC_HPP_INCLUDED
#define PMAAENSORLAYEROUTPUT_VFC_HPP_INCLUDED

//-----------------------------------------------------------------------------
// vfc includes
//-----------------------------------------------------------------------------
#include "vfc/core/vfc_types.hpp"

/*--------------------------------------------------------------------------*/
/*!\addtogroup RBP_MP_DEF           MP_DEF definitions
 * \ingroup RBP_COMPONENT_MP
 @{ *//*--------------------------------------------------------------------*/

#define rbp_MP_MAX_TIMING_GENERATORS_du8 ((vfc::uint8_t)(2))

/*--------------------------------------------------------------------------*/
/*- RBP_MES_MP                                                             -*/
/*--------------------------------------------------------------------------*/


/*!\defgroup RBP_MES_MP_M02           exported symbolic constants
 * \ingroup RBP_COMPONENT_RBP_MES_MP
 @{ */
#define rbp_MP_MAX_NUMBER_OF_SENSORS_du8 ((vfc::uint8_t)(14))
#define rbp_MP_MAX_USED_SENSORS_du8 ((vfc::uint8_t)(12))
#define rbp_MP_NUMBER_OF_ECHOS_ON_ADI_du8 ((vfc::uint8_t)(8))


// maximum size of position buffer ring memory
#define rbp_vhm_VehPosHistRMSize_du8   ((vfc::uint8_t)20)
#define rbp_vhm_common_vehPosHistRMSize_du8 ((vfc::uint8_t)20)

#define RBP_NUM_WADESENSORS ((vfc::uint8_t)(2))

#define RBP_MAX_WADEASSIST_ECHOES ((vfc::uint8_t)(10))


namespace pmasens_vfc
{
  
    /************************************************Enumeration Struct Definition*******************************
    ************************************************************************************************************/
    /**
     * Typedef for EPS Status
     */
    enum class ERbpTypeVSMEpsStatus : vfc::int32_t         // rbp_Type_vsm_EpsStatus_en
    {
        rbp_EpsNotPresent_enm = 0x0,
        rbp_EpsInit_enm = 0x1,
        rbp_EpsInActive_enm = 0x3,
        rbp_EpsActivatable_enm = 0x4,
        rbp_EpsActive_enm = 0x6,
        rbp_EpsReady_enm = 0x8,
        rbp_EpsRestart_enm = 0x9,
        rbp_EpsError_enm = 0xA
    };

    enum class ERbpTypeVHMRollRecogState : vfc::int32_t  // rbp_Type_vhm_RollRecogState_en
    {
        rbp_vhm_MOV_FORW_enm = 0,
        rbp_vhm_MOV_BACKW_enm = 1,
        rbp_vhm_MOV_TURNED_FORW_enm = 2,
        rbp_vhm_MOV_TURNED_BACKW_enm = 3,
        rbp_vhm_MOV_UNKNOWN_enm = 4,
        rbp_vhm_STOP_enm = 5,
        rbp_vhm_MOV_UNDER_INVESTIGATION_enm = 6
    };

    //vehicle moving direction identification
    enum class ERbpTypeVHMVehMoveDir : vfc::int32_t  // rbp_Type_vhm_VehMoveDir_en
    {
        //vehicle moves forward
        rbp_vhm_MoveDirForward_enm = 0,

        //vehicle moves backward
        rbp_vhm_MoveDirBackward_enm = 1,

        //vehicle moving direction is not known
        rbp_vhm_MoveDirUnknown_enm = 2,

        //vehicle stands still
        rbp_vhm_MoveDirStop_enm = 3
    };

    enum class ERbpTypeVHMPositionQuality : vfc::int32_t // rbp_Type_vhm_PositionQuality_en
    {
        rbp_vhm_HIGH_PRECISION_enm = 0,
        rbp_vhm_GOOD_PRECISION_enm = 1,
        rbp_vhm_MEDIUM_PRECISION_enm = 2,
        rbp_vhm_LOW_PRECISION_enm = 3,
        rbp_vhm_SOFT_DEGRADATION_enm = 4,
        rbp_vhm_FATAL_ERROR_enm = 5
    };

    enum class ERbpTypeVHMCalibState : vfc::int32_t  // rbp_Type_vhm_CalibState_en
    {
        rbp_vhm_CalibState_INITIAL_enm = 0,
        rbp_vhm_CalibState_UNSTABLE_enm = 1,
        rbp_vhm_CalibState_STABLE_enm = 2
    };

    //Update states
    enum class ERbpTypeVHMUpdateState : vfc::int32_t // rbp_Type_vhm_UpdateState_en
    {
        // VHM init
        rbp_vhm_StateInit_enm = 0,

        //position updated
        rbp_vhm_StateUpdated_enm = 1,

        //position not updated
        rbp_vhm_StateNotUpdated_enm = 2,

        //position reinitialized
        rbp_vhm_StateReInit_enm = 3
    };

    enum class ERbpTypeVHMCommonVehMoveDir : vfc::int32_t   // rbp_Type_vhm_common_vehMoveDir_en
    {
        rbp_vhm_common_forward_enm = 0,
        rbp_vhm_common_backward_enm = 1,
        rbp_vhm_common_unknown_enm = 2,
        rbp_vhm_common_stop_enm = 3
    };

    enum class ERbpTypeVHMCommonOutputState : vfc::int32_t  // rbp_Type_vhm_common_outputState_en
    {
        rbp_vhm_common_init_enm = 0,
        rbp_vhm_common_updated_enm = 1,
        rbp_vhm_common_notUpdated_enm = 2,
        rbp_vhm_common_reInit_enm = 3
    };

    enum class ERbpTypeVHMCommonQuality : vfc::int32_t  // rbp_Type_vhm_common_quality_en
    {
        rbp_vhm_common_high_enm = 0,
        rbp_vhm_common_good_enm = 1,
        rbp_vhm_common_medium_enm = 2,
        rbp_vhm_common_low_enm = 3,
        rbp_vhm_common_softDegradation_enm = 4,
        rbp_vhm_common_fatalError_enm = 5
    };

    //Typedef for Gear
    enum class ERbpTypeVSMGear : vfc::int32_t    // rbp_Type_vsm_Gear_en
    {
        rbp_Park_enm = 0,
        rbp_Reverse_enm = 1,
        rbp_Neutral_enm = 2,
        rbp_Drive_enm = 3,
        rbp_NoSignal_enm = 4
    };

    enum class ERbpTypeVSMSideMirrorstate : vfc::int32_t // rbp_Type_vsm_SideMirrorstate_en
    {
        rbp_SMS_DoNotConsider_enm = 0,
        rbp_SMS_Folded_enm = 1,
        rbp_SMS_Unfolded_enm = 2
    };

    //Typedef for Wheel Direction
    enum class ERbpTypeVSMWheelDir : vfc::int32_t    // rbp_Type_vsm_WheelDir_en
    {
        rbp_WheelDirForward_enm = 0,
        rbp_WheelDirBackward_enm = 1,
        rbp_WheelDirUnknown_enm = 2,
        rbp_WheelDirStop_enm = 3
    };

    //Typedef for Engine Start
    enum class ERbpTypeVSMEngineStart : vfc::int32_t // rbp_Type_vsm_EngineStart_en
    {
        rbp_EngineNoStart_enm = 0,
        rbp_EngineFirstStart_enm = 1,
        rbp_EngineRestart_enm = 2
    };

    enum class ERbpTypeVSMWheelChange : vfc::int32_t // rbp_Type_vsm_WheelChange_en
    {
        rbp_vsm_Unknown_enm = 0,
        rbp_vsm_WheelChanged_enm = 1,
        rbp_vsm_WheelNotChanged_enm = 2
    };

    enum class ERbpTypeSwcpmaSystemState : vfc::int32_t  // rbp_Type_swcpma_SystemState_en
    {
        // Initialization phase of swcpma
        rbp_swcpma_InitMode_enm,

        // Processing phase of swcpma
        rbp_swcpma_NormalMode_enm,

        // Standby mode of swcpma
        rbp_swcpma_StandByMode_enm,

        // swcpma ready for system shutdown
        rbp_swcpma_ShutDown_enm
    };

    struct CRbpTypeAcsPlWaUssDataEcho
    {
        vfc::uint32_t    m_DataId_bf2:2;
        vfc::uint32_t    m_CodeId_bf5:5;
        vfc::uint32_t    m_HeightInfoH_bf2:2;
        vfc::uint32_t    m_RunTime_bf13:13;
        vfc::uint32_t    m_AmplitudeA_bf6:6;
        vfc::uint32_t    m_CorrelationFactorR_bf4:4;
    };   // rbp_Type_acs_pl_wa_uss_DataEcho_st

    /**
     * describes quality criteria for an echo
     * - Significance = Reliability of the echo
     * - HeightSignificance = Probability that echo is reflected from a high object
     */
    struct CRbpTypeEchoQuality
    {
        // Measure for Reliability of the echo
        vfc::uint8_t m_Significance_bf2:2;

        // Height Significance of the echo (-> GroupID)
        vfc::uint8_t m_HeightSignificance_bf4:4;

        // Marks whether echo is filtered by appearance filter
        vfc::uint8_t m_EchoFiltered_bf1:1;

        // Echo is potentially coming from side
        vfc::uint8_t m_PotentialSideEcho_bf1:1;

    }; // rbp_Type_EchoQuality_st


    /**
     * provides echo information of the sensor
     */
    struct CRbpTypeEchoInfo
    {
        // Echo distance (mm)
        vfc::uint16_t m_EchoDist_u16;
        /**
         * Contains quality criteria of the echo
         * - Significance: Reliability of the echo
         * - HeightSignificance: GroupID
         * - Filtered: Marks whether echo is filtered
         */
        CRbpTypeEchoQuality m_EchoQuality_st;
    }; // rbp_Type_EchoInfo_st


    /**
     * contains all information of one measurement of a specific sensor
     */
    /**
     * Additional Attributes of an Echo
     * - Amplitude
     * - Matched Filter
     * (- R-value possible)
     */
    struct CRbpTypeMeasEchoAttributes
    {
        // amplitude of the measured echo
        vfc::uint32_t m_Amplitude_bf6:6;

        // Matched Filter of the echo (codeID)
        vfc::uint32_t m_MatchedFilter_bf5:5;

        // Indicates that echo was stored in trace with corresponding trace id
        vfc::uint32_t m_StoredInTrace_bf1:1;

        // Potential multiple reflection
        vfc::uint32_t m_MultipleRefl_bf1:1;

        // Indicates maximum echo distance at the echo wald
        vfc::uint32_t m_MaxDiffEchoWald_mm_bf8:8;

        // indicator for small wall bits of height significances (e.g. hs3 hs4)
        vfc::uint32_t m_HeightSigSmallWallBit_bf1:1;

        // Provides R value (correlation)
        vfc::uint32_t m_R_bf4:4;

        // to be used in the future
        vfc::uint32_t m_reserved_bf6:6;

    }; // rbp_Type_MeasEchoAttributes_st


    /**
     * contains all information of one measured echo
     */
    struct CRbpTypeMeasEchoInfo
    {
        // Attributes of measured echo
        CRbpTypeMeasEchoAttributes m_EchoAttributes_st;

        // Echo information
        CRbpTypeEchoInfo m_EchoInfo_st;

        // ID of corresponding trace
        vfc::uint16_t m_TraceID_u16;

        // ID of receiving sensor
        vfc::uint8_t m_RxSensorID_u8;

        // Weighting Factor based on Clutter Parameter
        vfc::uint8_t m_ClutterWeightFactor_u8;
    }; // rbp_Type_MeasEchoInfo_st


    struct CRbpTypeMeas
    {
        // provides direct echo information
        CRbpTypeMeasEchoInfo m_DE_pst[rbp_MP_NUMBER_OF_ECHOS_ON_ADI_du8];

        // provides cross echo information on the left sensor
        CRbpTypeMeasEchoInfo m_CEleft_pst[rbp_MP_NUMBER_OF_ECHOS_ON_ADI_du8];

        // provides cross echo information on the right sensor
        CRbpTypeMeasEchoInfo m_CEright_pst[rbp_MP_NUMBER_OF_ECHOS_ON_ADI_du8];

        // bitmask of receivers that have been participating in this measurement
        vfc::uint16_t m_CrossEchoRcvs_bf16;

        // Clutter value that was detected during the measurement
        vfc::uint8_t  m_Clutter_u8;

        // ID of sending sensor
        vfc::uint8_t  m_TxSensor_u8;

        // Code ID used by sending sensor
        vfc::uint8_t  m_TxSensorCodeID_u8;
    }; // rbp_Type_Meas_st

    struct CRbpTypeMpMeasurement
    {
        /**
         * time when measurement was started (transmit time), in [ms] CPU time of the CPU core that runs MP.
         * Contains values from 0 - 65535.
         */
        vfc::uint32_t m_ecuTime_pu32[rbp_MP_MAX_TIMING_GENERATORS_du8];
        vfc::uint8_t  m_measurementID_pu8[rbp_MP_MAX_TIMING_GENERATORS_du8];
        /**
         * each of the 16 available Sensors in the system have a bit in this variable. If the bit is set
         * (value = 1) the Sensor was sensing in the actual measurement
         */
        vfc::uint16_t m_sensorMask_pbf16[rbp_MP_MAX_TIMING_GENERATORS_du8];
        vfc::uint16_t m_RxMask_pbf16[rbp_MP_MAX_TIMING_GENERATORS_du8];
        vfc::uint16_t m_mpRxImpMeas_bf16[rbp_MP_MAX_TIMING_GENERATORS_du8];
        vfc::uint8_t  m_mpDisturbCount_pu8[rbp_MP_MAX_TIMING_GENERATORS_du8];
    }; // rbp_Type_mp_Measurement_st


    /**
     * describes minimal and maximal detection range of a measurement
     */
    struct CRbpTypeEchoRange
    {
        //Minimal possible distance of measurement
        vfc::uint16_t m_DistMin_u16;

        // Maximal possible distance of measurement
        vfc::uint16_t m_DistMax_u16;
    }; // rbp_Type_EchoRange_st

    // This structure contains all data that is returned by MP API functions as a data
    // packet that can either be monitored by a measurement interface or used in an
    // Upper layer partition (see mp_stubs.c!)
    struct CRbpTypeMpMonitorData
    {
        CRbpTypeMpMeasurement    m_Measurement_st;
        CRbpTypeMeas             m_MeasData_st[rbp_MP_MAX_NUMBER_OF_SENSORS_du8];
        bool                     m_SensorBlind_pb[rbp_MP_MAX_NUMBER_OF_SENSORS_du8];
        CRbpTypeEchoRange        m_SensorMeasRange_pst[rbp_MP_MAX_NUMBER_OF_SENSORS_du8]; // Maximum Measurement Range

        //! required to stub the MP API function rbp_mp_GetBuffIndex_u8(vfc::uint8_t f_sensID_u8)
        vfc::uint8_t             m_vehID2SensBuffTbl_pu8[rbp_MP_MAX_NUMBER_OF_SENSORS_du8+1];
        vfc::uint8_t             m_NoiseLevel_u8[rbp_MP_MAX_USED_SENSORS_du8];

        //! coefficients for temperature compensation
        vfc::uint16_t            m_TempCompCoeff_pu16[2];
        //! actual temperature
        vfc::int8_t              m_ActualTemp_s8;

        //! data provided by bool rbp_mp_getMpOutputValid_b()
        bool                     m_MpOutputValid_b;
      
        //! data provided by bool rbp_mp_IsMPinHaltState_b()
        bool                     m_MpInHaltState_b;

        //! active sensor information
        vfc::uint16_t            m_ActiveDESens_bf16;
        vfc::uint16_t            m_ActiveCESens_bf16;
    }; // rbp_Type_mp_MonitorData_st

    /**
     * current vehicle motion state, computed based on the steering angles, WICs, and/or external
     * information. The vehicle motion state is updated in every VHM cycle, usually every 10 ms.
     */
    struct CRbpTypeVHMVehMotionState
    {
        /**
         * vehicle speed, signed (negative = backwards motion), longitudinal axis. If vehMoveDir is STOP, this is set to 0.
         * Source for this signal is the VHM internal speed calculation, the CAN vehicle speed signal, or an
         * average of both, depending on the quality of the signal source. Unit: [m/s]
         */
        vfc::float32_t m_vx_f32;

        /**
         * vehicle speed, y-axis. signed (negative = motion to right side). If vehMoveDir is STOP, this is set to 0.
         * Source for this signal is the VHM model speed calculation. Unit: [m/s]
         */
        vfc::float32_t m_vy_f32;

        // longitudinal acceleration a_x [m/s^2] calculated from WIC.
        vfc::float32_t m_ax_f32;

        // lateral acceleration (a_y) [m/s^2] calculated from yaw rate and model
        vfc::float32_t m_ay_f32;

        /**
         * Curvature of the current vehicle course. Reciprocal value of the distance from the momentary center
         * of rotation (Momentanpol) to the center of the rear axle. Signed: Positive sign for left turn,
         * negative for right turn. Zero for straight driving. This curvature is based on the VHM S-delay for
         * steering angle changes (see SteerAngleFront). The curvature calculation is not restricted by
         * vehicle velocity. Even above 90 km/h the current curvature will be calculated. If the vehicle is
         * equipped with four wheel steering, the calculation is based on steering angle front and alpha_rear.
         * Unit 1/m
         */
        vfc::float32_t m_kappa_f32;

        // Front wheel angle which takes autocalib data into consideration. Unit [rad]
        vfc::float32_t m_alpha_front_f32;
        /**
         * the side slip angle (Deutsch: Schraeglaufwinkel) at the center point of the ego vehicle's rear axle.
         * For slow driving with low lateral acceleration, and especially during active guidance, this is
         * equal to the average steering angle of the rear axle delta_rear. For a fixed rear axle (2WS) system,
         * this will always be zero in active guidance (and most likely in free maneuvering, too). Unit: [rad].
         */
        vfc::float32_t m_alpha_rear_f32;
        /**
         * "Delta_front", the average steering angle of the wheels at the front axle. Unit = rad. This value
         * is the equivalent front wheel steering angle of the single track model. When the vehicle is moving,
         * the steering angle is delayed by an S-position delay of a few cm, to consider the influence of the
         * tyre (see VHM documentation). Delta_front is already corrected by the SWA offset autocalibration
         * (if it's based on the SWA signal and autocalibration is enabled).
         */
        vfc::float32_t m_steerAngleFront_f32;

        // VHM vehicle motion direction based on WIC directions
        ERbpTypeVHMVehMoveDir m_vehMoveDir_en;

        // Information on rolling direction state, used for ADTF record and replay functionality
        ERbpTypeVHMRollRecogState m_rollingDirRecogState_en;

        // VHM vehicle motion direction based on gear signal
        ERbpTypeVHMVehMoveDir     m_vehDriveDir_en;
    }; // rbp_Type_vhm_VehMotionState_st

    // Point or Vector in a Euclidean 2D space in 32bit coordinates
    struct CRbpTypePoint32
    {
        // !< x-coordinate or x-component of vector
        vfc::int32_t m_X_s32;
        // !< y-coordinate or y-component of vector
        vfc::int32_t m_Y_s32;
    }; // rbp_Type_Point32_st


    /**
     * The vehicle state (position and motion state) is the main output of VHM. Usually refers to the
     * current vehicle state. Supports both 2WS and 4WS vehicles, external or VHM internal odometry, mini
     * or premium VHM...
     */
    struct CRbpTypeVHMVehState
    {
        /**
         * the momentary (current) vehicle position, in the VHM coordinate system, computed by the VHM
         * odometry. All length values have unit [mm]. VehPos is reset via vhm_ReInit.
         * Note: The VehPos has some special properties:
         * - X/Y does not change while the rolling direction is unknown.
         * - Yaw angle can change in standstill, if the steering wheel is turned a lot.
         * For more info, see VHM documentation.
         */
        CRbpTypePoint32    m_VehPos_mm_st;

        /**
         * the accumulated driven distance of the vehicle (forward: positive values, backward: negative values),
         * in millimeters, since the last vehicle coordinate reset with driven-distance reset (e.g. KL15 on)
         */
        vfc::int32_t       m_SPos_mm_s32;

        /**
         * Sum of the driven distance since the initialization of the system independent of a detected driving
         * direction. Is not reset on vhm_ReInit. Unit [mm]
         */
        vfc::uint32_t      m_DrivenDistAbs_mm_u32;

        /**
         * yaw angle psi of vehicle in [rad]: Rotation around z-axis, orientation in 2D space.
         * We use the convention for land-driven vehicles: y-axis is 90 deg left of x-axis, and z-axis points upwards.
         * therefore, a positive yaw angle change means a left turn.
         */
        vfc::float32_t     m_psi_f32;

    }; // rbp_Type_vhm_VehState_st

    // Additional output of extra features, that might not be necessarily available in both versions of VHM.
    struct CRbpTypeVHMAdditionalFeatureOutputs
    {
        // Quality of the output posision taking into account sensor failures, driving scenario and calibration status
        ERbpTypeVHMPositionQuality    m_quality_en;

        // Variance and covariance values of the position and orientation, x,y,yaw-angle
        vfc::float32_t m_PoseCovarianceMatrix_ppf32[3][3];

        // Number of rotations around z axis(overflow of psi_f32). Positive counterclockwise
        vfc::int8_t    m_NumberOfRotations_s8;

        /**
         * pitch angle theta (rotation around y-axis; see roll-pitch-yaw definition in DIN 70001 or
         * https://de.wikipedia.org/wiki/Roll-Nick-Gier-Winkel). Unit: [rad]
         */
        vfc::float32_t m_theta_f32;

        /**
         * In case of 4WS, VHM1 calculations are based on a virtual wheelbase which is changing depending on the rear wheel's angle
         * The wheelbase shall be calculated by subtracting this value from the parameterized real wheelbase.
         * Otherwise it's 0. Unit: [mm]
         */
        vfc::int16_t   m_DeltaVirtualWheelbase_mm_s16;

        /**
         * The X and Y coordinates output of VHM1 mapped to instantaneous center in case 4WS is active.
         * Otherwise it's 0. Unit: [mm]
         */
        CRbpTypePoint32 m_VehPosInstantaneousCenter_mm_st;

        /**
         * The driven distance output of VHM1 mapped to instantaneous center in case 4WS is active.
         * Otherwise it's 0. Unit: [mm]
         */
        vfc::int32_t    m_SPosInstantaneousCenter_mm_s32;

    }; // rbp_Type_vhm_AdditionalFeatureOutputs_st

    struct CRbpTypeVHMSteeringLimits
    {
        /**
         * the largest curvature the vehicle can do in a left or right turn in forward motion, when the rear
         * axle steering angle (delta_rear) is zero. Unit: [1/m]
         */
        vfc::float32_t m_kappa_max_fwd_2ws_f32;
        /**
         * the largest curvature the vehicle can do in a left or right turn in backward motion, when the rear
         * axle steering angle (delta_rear) is zero. Unit: [1/m]
         */
        vfc::float32_t m_kappa_max_bwd_2ws_f32;
        /**
         * Maximum possible steering angle at the rear axle. This refers the steering angle of a "virtual
         * wheel" at the center of the rear axle (single track model), which is an average of the left and
         * right rear wheel steering angles. The rear steering system is assumed to be symmetric. No
         * difference is made between forward and backward driving. Unit: [F12 rad]. Typical values range
         * within [-10� ... +10�].
         * Note: For a two-wheel steering system, this will contain value 0. For a four-wheel steering system,
         * the maximum curvature with maximum delta_rear has to be computed by the function
         * vc_getKappaMax4WS(). Unit: 2^-12 radian
         */
        vfc::float32_t m_steer_angle_rear_max_f32;
    }; // rbp_Type_vhm_SteeringLimits_st

    // Data from VHM internal calibration. Was called "rbp_Type_DevelopmentData_st" in VHM API v1
    struct CRbpTypeVHMCalibData
    {
        /**
         * The steering wheel angle when the vehicle is driving perfectly straight (Motion.kappa = 0,
         * delta_front = 0). This is called the SWA offset, and it may exists, as the steering wheel might not
         * be calibrated perfectly (EOL tolerance is usually < 3�, though). See autocalibration feature design
         * for details. If the autocalibration state is not yet stable, the project does not use an SWA-based
         * steering interface, or the steering wheel has no offset, this value is zero. Unit: tbd (0.04� or
         * F11 rad).
         */
        vfc::int16_t m_SWAOffset_s16;

        // State of steering wheel offset calibration.
        ERbpTypeVHMCalibState m_SWAState_en;
 
        // Yaw rate offset, in an unknown quantity
        vfc::int16_t m_YROffset_s16;

        // State of yaw rate offset calibration.
        ERbpTypeVHMCalibState m_YRState_en;
    }; // rbp_Type_vhm_CalibData_st

    /**
     * All 10ms-vhm-runnable output data in one structure.
     * Note: The VHM position history buffer and VHM transformation info is
     * kept separately. To get absolutely everything in one structure, use the VHM monitor
     */
    struct CRbpTypeVHMOutput
    {
        ///Additional outputs containing extra features which are not necessarily available in both versions of VHM.
        CRbpTypeVHMVehMotionState    m_MotionState_st;
        CRbpTypeVHMVehState          m_VehicleState_st;
        CRbpTypeVHMAdditionalFeatureOutputs m_AdditionalFeatureOutputs_st;
        CRbpTypeVHMSteeringLimits    m_SteeringLimits_st;
        CRbpTypeVHMCalibData         m_CalibData_st;

        /// timestamp associated with the VHM vehicle state, using common time base.
        /// (required by OLO measurement object time)
        /// ST chorus: ECU time stamp [ms] at the moment of the last WIC message reception (usually sometime in the last 20 ms)
        /// NRCS2: System time stamp in milliseconds or microseconds at the moment of the last WIC message reception, possibly
        /// also corrected by flexray time provided to match the sending time of the last WIC message.
        vfc::uint32_t  m_WICmsg_time_u32;

        /**
         * Indicates whether the vehicle position (VehicleState_st) was updated in the current cycle,
         * or if the VHM coordinate system
         * was re-initialized, resulting in a completely new VHM vehicle position.
         */
        ERbpTypeVHMUpdateState m_VHMUpdateState_en;
    }; // rbp_Type_vhm_Output_st

    struct CRbpTypePointFl
    {
        vfc::float32_t X_f32;
        vfc::float32_t Y_f32;
    };  // rbp_Type_PointFl_st

    struct CRbpTagAngleFl
    {
        vfc::float32_t Angle_f32;
        vfc::float32_t Sin_f32;
        vfc::float32_t Cos_f32;
    }; // rbp_Type_AngleFl_st

    struct CRbpTypeLineSinCosFl
    {
        CRbpTypePointFl m_P_st;
        CRbpTagAngleFl  m_Phi_st;
    }; // rbp_Type_LineSinCosFl_st

    /// Information for VHM coordinate system re-initialization and transformation
    /// to previous VHM coordinates.
    struct CRbpTypeVHMCoordSysTrafo
    {
        /// the coordinate system transformation (or new coordinate system origin)
        /// that was given in the last VHM ReInit.
        /// If no ReInit was ever performed, this is (0/0/0).
        /// For the re-init #n, this contains the location of the n-th coordinate
        /// system origin in the coordinate system that was used before (n-1).
        /// To transform from the previous VHM coordinate system to the new coordinate system,
        /// use rbp_geo_TransformToLocal_vd()
        CRbpTypeLineSinCosFl    m_NewCSTrafo_st;

        /// The current VHM coordinate system origin in the coordinate system after VHM_Init
        /// (ECU initialization, the "base" coordinate system used in external subsystems)
        /// After initialization, this is (0/0/0).
        CRbpTypeLineSinCosFl    m_CurrentCoordSys_st;

        /// true if a the coordinate system change of the VHM vehicle position (with transformation)
        /// was done in the last call to rbp_vhm_10ms_vd, false otherwise.
        bool m_ReInitFlag_b;
    }; // rbp_Type_vhm_CoordSysTrafo_st

    /// position history for USS position mapping
    /// this data type is required here for VHM_monitor
    struct CRbpTypeVHMVehPosHist
    {
        CRbpTypeVHMVehState  m_VehState_st[rbp_vhm_VehPosHistRMSize_du8];
        vfc::uint32_t        m_WIC_CanTime_pu32[rbp_vhm_VehPosHistRMSize_du8];       // in case of a new WIC message

        // ring-buffer management data
        vfc::uint8_t  m_IdMax_u8;                    // maximum number of currently stored values
        vfc::uint8_t  m_IdNewest_u8;                 // ID of newest entry
        vfc::uint8_t  m_IdOldest_u8;                 // ID of oldest entry
        vfc::uint8_t  m_PosRMSize_u8;                // size of position buffer ring memory
    }; // rbp_Type_vhm_VehPosHist_st

    struct CRbpTypeVHMPrjOutput
    {
        // Valid actual driving position
        ERbpTypeVSMGear    m_ActualDrvPos_en;
      
        // Valid target driving position
        ERbpTypeVSMGear    m_TargetDrvPos_en;

        // Driving position P securely engaged
        bool m_PSecure_b;

        // Vehicle rolling direction, taking into account only the WIC directions.
        ERbpTypeVHMVehMoveDir m_vehRollDir_en;

        // Vehicle driving direction, taking into account the WIC directions and Gear.At standstill the direction
        // is obtained based on Gear.
        ERbpTypeVHMVehMoveDir m_vehDriveDir_en;

        // appliqued default wheel circumference
        vfc::float32_t m_WhlCircDef_f32;

        // estimated wheel circumference
        vfc::float32_t m_WhlCircEst_f32;

        // appliqued default track width
        vfc::float32_t m_TrackDef_f32;

        // estimated track width
        vfc::float32_t m_TrackEst_f32;

        // wheelbase
        vfc::float32_t m_WhlBase_f32;

        // Roll angle
        vfc::float32_t m_Phi_f32;

        // Evaluation of gear position
        ERbpTypeVSMGear m_TargetGear_en;

      // add further output signals here
    };  // rbp_Type_vhm_PrjOutput_st

    // If you want to provide customer projects with the ability
    // to monitor project specific data please add this data structure
    // to rbp_Type_vhm_MonitorData_st 
    struct CRbpTypeVHMPrjMonitorData
    {
        //! Example output value. This one returns 1 if VHM1 is used, and 2 if VHM2 is used
        // Only use project specific components if VHM2 is active
        // #ifdef RBP_VHM2_ENVIRONMENT_FOR_ODOMETRY
        // #if (RBP_VHM2_ENVIRONMENT_FOR_ODOMETRY != RBP_VHM2_OFF)
        //! data returned by the rbp_vhm_getPrjOutput_vd function
        CRbpTypeVHMPrjOutput    m_Output_st;
        // #endif
        // #endif
        vfc::uint16_t           m_vhm_version_u16;
    }; // rbp_Type_vhm_PrjMonitorData_st

    struct CRbpTypeVHMMonitorData
    {
        //! state of rolling direction recognition, used for debugging and MES record & replay
        ERbpTypeVHMRollRecogState    m_RollingDirRecogState_en;

       /*************************************************************************
       *     VHM interface API v2 data
       *************************************************************************/

        //! output: computation results of cyclic VHM2 model call.
        CRbpTypeVHMOutput            m_Output_st;

        //! current valid coordinate system transformation info. From PMA-internal VHM position to
        //! global (reinit-free) coordinates
        CRbpTypeVHMCoordSysTrafo     m_Trafo_st;

        //! Special service to NRCS2 PF: The VHM position without any coordinate system re-initialization
        CRbpTypePoint32              m_VehPos_ReInitFree_st;
        vfc::float32_t               m_VehPsi_ReInitFree_f32;

        //! full content of the VHM history buffer (used for GetEstimatedPosition VHM interface function)
        //! This is with re-init. Use the Trafo_st data to transform into re-init free realm
        CRbpTypeVHMVehPosHist        m_VehPosHist_st;

        CRbpTypeVHMCalibData         m_CalibData_st; // TODO: this is a duplicate !!!!!!!!

        //! Project-specific ADI data
        CRbpTypeVHMPrjMonitorData    m_PRJ_st;
    }; // rbp_Type_vhm_MonitorData_st

    struct CRbpTypeVHMCommonVehicleState
    {
        //Position of the center of the rear axle in the current coordinate system
        CRbpTypePoint32    m_Position_mm_st;

        //Heading angle of the vehicle (angle around axle Z) [-pi, +pi]
        vfc::float32_t     m_YawAngle_rad_f32;

        //Accumulated driven distance without considering the moving direction
        vfc::uint32_t      m_AbsoluteDrivenDistance_mm_u32;

        //Accumulated driven distance which considers the moving direction,
        //increments for forward decrements for backward movements
        vfc::int32_t       m_SignedDrivenDistance_mm_s32;

    }; // rbp_Type_vhm_common_vehicleState_st

    struct CRbpTypeVHMCommonSteering
    {
        //Actual steering angle of the front virtual middle wheel [-pi, +pi]
        //Calculated using the FWD or BWD steering polinom depending on the actual moving direction
        vfc::float32_t    m_FrontWheelAngle_rad_f32;

        //VHM1: Actual steering angle of the rear virtual middle wheel [-pi, +pi]
        //VHM2: Actual moving angle (steering angle + slip) of the rear virtual middle wheel [-pi, +pi]
        //Calculated based on FWD or BWD steering polinom depending on the actual moving direction
        vfc::float32_t    m_RearWheelMovingAngle_rad_f32;

        //VHM1: Actual steering angle of the rear virtual middle wheel [-pi, +pi]
        //VHM2: Actual moving angle (steering angle + slip) of the rear virtual middle wheel [-pi, +pi]
        //Calculated always using the FWD steering polinom
        vfc::float32_t    m_RearWheelMovingAngleFwd_rad_f32;

        //VHM1: Actual steering angle of the rear virtual middle wheel [-pi, +pi]
        //VHM2: Actual moving angle (steering angle + slip) of the rear virtual middle wheel [-pi, +pi]
        //Calculated always using the BWD steering polinom
        vfc::float32_t    m_RearWheelMovingAngleBwd_rad_f32;

        //Curvature of the current vehicle course
        //Calculated based on FWD or BWD steering polinom depending on the actual moving direction
        vfc::float32_t    m_ActualCurvature_1pm_f32;

        //Curvature of the current vehicle course
        //Calculated always using the FWD steering polinom
        vfc::float32_t    m_ActualCurvatureFwd_1pm_f32;

        //Curvature of the current vehicle course
        //Calculated always using the BWD steering polinom
        vfc::float32_t    m_ActualCurvatureBwd_1pm_f32;

        //Largest curvature the vehicle can do in a left or right turn in forward motion,
        //without considering rear axle steering
        vfc::float32_t    m_MaxCurvatureFwd2ws_1pm_f32;

        //Largest curvature the vehicle can do in a left or right turn in backward motion,
        //without considering rear axle steering
        vfc::float32_t    m_MaxCurvatureBwd2ws_1pm_f32;

        //Maximum possible steering angle at the rear virtual middle wheel. [-pi, +pi]
        vfc::float32_t    m_MaxRearWheelAngle_rad_f32;

        //In case of 4WS, VHM1 calculations are based on a virtual wheelbase
        //which is changing depending on the rear wheel's angle
        //The wheelbase shall be calculated by subtracting this value
        //from the parameterized real wheelbase.
        vfc::int16_t      m_VHM1_4WS_DeltaVirtualWheelbase_mm_s16;
    }; // rbp_Type_vhm_common_steering_st

    struct CRbpTypeVHMCommonMotionState
    {
        //Velocity in vehicle coordinate system
        CRbpTypePointFl    m_Velocity_mps_st;

        //Acceleration in vehicle coordinate system
        CRbpTypePointFl    m_Acceleration_mpSqrS_st;

        //Current vehicle movement direction
        ERbpTypeVHMCommonVehMoveDir m_MovementDirection_en;

    };  // rbp_Type_vhm_common_motionState_st

    //VHM_COMMON output api signals
    struct CRbpTypeVHMCommonOutputSignals
    {
        CRbpTypeVHMCommonVehicleState    m_ActualVehicleState_st;
        CRbpTypeVHMCommonSteering        m_Steering_st;
        CRbpTypeVHMCommonMotionState     m_MotionState_st;

        //State of output for the current cycle
        ERbpTypeVHMCommonOutputState     m_OutputState_en;

        //Timestamp based on wic message timestamps
        vfc::uint32_t                    m_TimeStamp_ms_ux;

        //Current quality of position estimation
        ERbpTypeVHMCommonQuality         m_Quality_en;

    };  // rbp_Type_vhm_common_outputSignals_st

    struct CRbpTypeVHMCommonCoordSysTrafo
    {
        //the coordinate system transformation (or new coordinate system origin)
        //that was given in the last VHM ReInit.
        //If no ReInit was ever performed, this is (0/0/0).
        //For the re-init #n, this contains the location of the n-th coordinate
        //system origin in the coordinate system that was used before (n-1).
        CRbpTypeLineSinCosFl    m_NewCSTrafo_st;

        //The current VHM coordinate system origin in the coordinate system after VHM_Init
        //(ECU initialization, the "base" coordinate system used in external subsystems)
        //After initialization, this is (0/0/0).
        CRbpTypeLineSinCosFl    m_CurrentCoordSys_st;

        //true if a the coordinate system change of the VHM vehicle position (with transformation)
        //was done in the last call to rbp_vhm_10ms_vd, false otherwise.
        bool                    m_ReInitFlag_b;
    };  // rbp_Type_vhm_common_coordSysTrafo_st

    struct CRbpTypeVHMCommonVehPosHist
    {
        CRbpTypeVHMCommonVehicleState    m_VehState_st[rbp_vhm_common_vehPosHistRMSize_du8];
        vfc::uint32_t m_Time_pux[rbp_vhm_common_vehPosHistRMSize_du8];

        //ring-buffer management data

        //maximum number of currently stored values
        vfc::uint8_t  m_IdMax_u8;
        //ID of newest entry
        vfc::uint8_t  m_IdNewest_u8;
        //ID of oldest entry
        vfc::uint8_t  m_IdOldest_u8;
        //size of position buffer ring memory
        vfc::uint8_t  m_PosRMSize_u8;
    };  // rbp_Type_vhm_common_vehPosHist_st

    //VHM_COMMON output data for multicore data communication
    struct CRbpTypeVHMCommonOutputData
    {
        CRbpTypeVHMCommonOutputSignals    m_VHMData_st;
        CRbpTypeVHMCommonCoordSysTrafo    m_TrafoData_st;
        //TODO: restructure vehposhist and use the new type (rbp_Type_vhm_common_vehPosHist_st)
        CRbpTypeVHMCommonVehPosHist       m_VehPosHistData_st;
    };  // rbp_Type_vhm_common_outputData_st

    // Typedef for WIC signal and Ticks
    struct CRbpTypeVSMWICandTime
    {
        /**
        * Wheel Impulse Counter Signal,  Resolution (1Bit = 1 pulse)
        *   Range =  0 -  65535 pulse
        */
        vfc::uint16_t    m_WIC_u16;
        /**
        * Wheel Impulse Counter tick Signal
        * Resolution (1Bit = 1ms)          (ms derived from OS)
        * Range in USS PF (ST chorus):     0 -  65535 ms
        * Max allowed range: 32 bit.
        */
        vfc::uint32_t    m_WIC_Tick_u32;
    };  // rbp_Type_vsm_WICandTime_st

    struct CRbpTypeVSMSWAandTime
    {
        /**
        * SWA Signal Resolution (1Bit = 1 pulse)
        * Range  0 -  65535 pulse
        */
        vfc::int16_t    m_SWA_s16;
        /**
        * SWA tick Signal
        * Range in USS PF (ST chorus):     0 -  65535 ms
        * Max allowed range: 32 bit.
        */
        vfc::uint32_t   m_SWA_Tick_u32;
    };  // rbp_Type_vsm_SWAandTime_st

    struct CRbpTypeVSMGPSandTime
    {
        /**
        * GPS Signal,  Resolution (1Bit = 1 pulse)
        *   Range =  0 -  65535 pulse
        */
        vfc::int32_t   m_GPS_s32;
        /**
        * GPS tick Signal
        * Resolution (1Bit = 1ms)          (ms derived from OS)
        * Range               0 -  65535 ms
        */
        vfc::uint32_t   m_GPS_Tick_u32;
    };  // rbp_Type_vsm_GPSandTime_st

    /** This structure contains all input signals which are required by UP6 platform components,
    * and are provided by functions in rbp_vsm_api.h. Adapt this whenever you adapt rbp_vsm_api.h.
    * This structure is immutable for customer projects. The projects can adapt the
    * function to fill this structure, rbp_vsm_getMonitorData_PF_vd(). The rbp_vsm_monitor.c_tplpp
    * is a template.
    */
    struct CRbpTypeVSMMonitorPF
    {
        vfc::uint16_t                  m_AdiVehSpeed_u16;
        bool                           m_AdiVehSpeedReturn_b;

        vfc::int8_t                    m_AdiOutsideTemp_s8;
        bool                           m_AdiOutsideTempReturn_b;

        vfc::uint8_t                   m_AdiUBatt_u8;
        bool                           m_AdiUBattReturn_b;

        ERbpTypeVSMGear                m_AdiGear_en;
        bool                           m_AdiGearReturn_b;

        bool                           m_AdiTrailerHitchPresent_b;
        bool                           m_AdiTrailerHitchPresentReturn_b;

        bool                           m_AdiTrailerAttached_b;
        bool                           m_AdiTrailerAttachedReturn_b;

        ERbpTypeVSMSideMirrorstate     m_AdiSideMirrorLeftPresent_en;
        bool                           m_AdiSideMirrorLeftPresentReturn_b;

        ERbpTypeVSMSideMirrorstate     m_AdiSideMirrorRightPresent_en;
        bool                           m_AdiSideMirrorRightPresentReturn_b;

        CRbpTypeVSMWICandTime          m_AdiWICRRandTime_st;
        bool                           m_AdiWICRRReturn_b;

        CRbpTypeVSMWICandTime          m_AdiWICRLandTime_st;
        bool                           m_AdiWICRLReturn_b;

        CRbpTypeVSMWICandTime          m_AdiWICFRandTime_st;
        bool                           m_AdiWICFRReturn_b;

        CRbpTypeVSMWICandTime          m_AdiWICFLandTime_st;
        bool                           m_AdiWICFLReturn_b;

        CRbpTypeVSMSWAandTime          m_AdiSWAandTime_st;
        bool                           m_AdiSWAReturn_b;

        ERbpTypeVSMWheelDir            m_AdiWheelDirRR_en;
        bool                           m_AdiWheelDirRRReturn_b;

        ERbpTypeVSMWheelDir            m_AdiWheelDirRL_en;
        bool                           m_AdiWheelDirRLReturn_b;

        ERbpTypeVSMWheelDir            m_AdiWheelDirFR_en;
        bool                           m_AdiWheelDirFRReturn_b;

        ERbpTypeVSMWheelDir            m_AdiWheelDirFL_en;
        bool                           m_AdiWheelDirFLReturn_b;

        vfc::uint16_t                  m_AdiWheelRotationRR_u16;
        bool                           m_AdiWheelRotationRRReturn_b;

        vfc::uint16_t                  m_AdiWheelRotationRL_u16;
        bool                           m_AdiWheelRotationRLReturn_b;

        vfc::uint16_t                  m_AdiWheelRotationFR_u16;
        bool                           m_AdiWheelRotationFRReturn_b;

        vfc::uint16_t                  m_AdiWheelRotationFL_u16;
        bool                           m_AdiWheelRotationFLReturn_b;

        vfc::int16_t                   m_AdiYawRate_s16;
        bool                           m_AdiYawRateReturn_b;

        vfc::int16_t                   m_AdiLongAcc_s16;
        bool                           m_AdiLongAccReturn_b;

        vfc::int16_t                   m_AdiLatAcc_s16;
        bool                           m_AdiLatAccReturn_b;

        CRbpTypeVSMGPSandTime          m_AdiGPSLatitudeandTime_st;
        bool                           m_AdiGPSLatitudeReturn_b;

        CRbpTypeVSMGPSandTime          m_AdiGPSElevationandTime_st;
        bool                           m_AdiGPSElevationReturn_b;

        CRbpTypeVSMGPSandTime          m_AdiGPSLongitudeandTime_st;
        bool                           m_AdiGPSLongitudeReturn_b;

        vfc::uint8_t                   m_AdiGPSPDOP_u8;
        bool                           m_AdiGPSPDOPReturn_b;

        vfc::uint16_t                  m_AdiGPSHeading_u16;
        bool                           m_AdiGPSHeadingReturn_b;

        vfc::float32_t                 m_AdiGPSVelOvrGround_f32;
        bool                           m_AdiGPSVelOvrGroundReturn_b;

        vfc::uint16_t                  m_AdiGPSDateTimeMs_u16;
        bool                           m_AdiGPSDateTimeMsReturn_b;

        vfc::uint8_t                   m_AdiGPSDateTimeS_u8;
        bool                           m_AdiGPSDateTimeSReturn_b;
      
        vfc::int16_t                   m_AdiEPS_CurrentSA_Rear_s16;
        bool                           m_AdiEPS_CurrentSA_RearReturn_b;

        ERbpTypeVSMEngineStart         m_AdiEngineStart_en;
        bool                           m_AdiEngineStartReturn_b;

        vfc::uint16_t                  m_AdiTyrePressureFL_u16;
        bool                           m_AdiTyrePressureFLReturn_b;
      
        vfc::uint16_t                  m_AdiTyrePressureFR_u16;
        bool                           m_AdiTyrePressureFRReturn_b;
      
        vfc::uint16_t                  m_AdiTyrePressureRL_u16;
        bool                           m_AdiTyrePressureRLReturn_b;
      
        vfc::uint16_t                  m_AdiTyrePressureRR_u16;
        bool                           m_AdiTyrePressureRRReturn_b; 
    };  // rbp_Type_vsm_Monitor_PF_st

    /**
    * data structure that contains all input data (including signal status / validity flag)
    * that is required by PROJECT-SPECIFIC components or project-specific parts of a component
    * (here: DAI P5 specific)
    */
    struct CRbpTypeVSMMonitorPRJ
    {
        ERbpTypeVSMEpsStatus   m_AdiEpsStatus_en;
        bool                   m_AdiEpsStatusReturn_b;

        bool                   m_EpsActive_b;
        bool                   m_EpsActiveReturn_b;

        bool                   m_EpsActiveRas_b;
        bool                   m_EpsActiveRasReturn_b;
      
        vfc::uint8_t           m_AccPedalPos_u8;
        bool                   m_AccPedalPosReturn_b;
        
        vfc::uint8_t           m_TowBarType_u8;
        bool                   m_TowBarTypeReturn_b;
        
        bool                   m_SparewheelPresent_b;
        bool                   m_SparewheelPresentReturn_b;

        bool                   m_VehicleBlocked_b;
        bool                   m_VehicleBlockedReturn_b;
      
        ERbpTypeVSMGear        m_GearActual_en;
        bool                   m_GearActualReturn_b;

        vfc::float32_t         m_LongAccOffset_f32;
        bool                   m_LongAccOffsetReturn_b;

        vfc::float32_t         m_LatAccOffset_f32;
        bool                   m_LatAccOffsetReturn_b;

        bool                   m_AccXInputFailure_b;
        bool                   m_AccXInputFailureReturn_b;
      
        bool                   m_AccYInputFailure_b;
        bool                   m_AccYInputFailureReturn_b;

        bool                   m_YawRateInputFailure_b;
        bool                   m_YawRateInputFailureReturn_b;

        bool                   m_AngleRearInputFailure_b;
        bool                   m_AngleRearInputFailureReturn_b;

        bool                   m_AngleFrontInputFailure_b;
        bool                   m_AngleFrontInputFailureReturn_b;

        bool                   m_WheelInputFailureFR_b;
        bool                   m_f_WheelInputFailureFRReturn_b;

        bool                   m_WheelInputFailureFL_b;
        bool                   m_f_WheelInputFailureFLReturn_b;

        bool                   m_WheelInputFailureRR_b;
        bool                   m_f_WheelInputFailureRRReturn_b;

        bool                   m_WheelInputFailureRL_b;
        bool                   m_f_WheelInputFailureRLReturn_b;
      
        bool                   m_ESPIntervention_b;
        bool                   m_ESPInterventionReturn_b;
      
        bool                   m_IgnitionOn_b;
        bool                   m_IgnitionOnReturn_b;
        ERbpTypeVSMWheelChange m_WheelChangeStatus_enm;
        bool                   m_WheelChangeStatusReturn_b;
    };  // rbp_Type_vsm_Monitor_PRJ_st

    struct CRbpTypeVSMMonitor
    {
        CRbpTypeVSMMonitorPF  m_vsm_PF_st;
        CRbpTypeVSMMonitorPRJ m_vsm_PRJ_st;
    };  // rbp_Type_vsm_Monitor_st

    //! SWCPMA main status variables
    struct CRbpTypeSwcpmaState
    {
        //! the activation time (with OS-time or ECU-global time) of the current 10 ms task (sensor layer)
        //! used also for sequence call check
        vfc::uint32_t  m_Cyclic10msActivationTime_ms;

        /// count the 10ms task invocations of this instance of SWCPMA (in status "running")
        vfc::uint32_t  m_CycleCounter10ms_u32;

        /// execution manager status of this instance of the PMA-component
        ERbpTypeSwcpmaSystemState  m_state_en;

        //! RTE-start-up event - failsave initialization
        bool  m_isSoftResetTriggered_b;

        //! Save status of get1msTime function - for debugging and FLT purpose
        bool  m_getECUTimeResult_b;

    };  // rbp_Type_swcpmaState_st

    struct CRbpTypeSwcpmaMonitorData
    {
        CRbpTypeSwcpmaState m_swcpmaState_st;
        bool m_IsEol_b;
        bool m_IsActiveEol_b;
    };  // rbp_Type_swcpma_MonitorData_st

    struct CRbpTypeMpPlWaEcho
    {
        CRbpTypeAcsPlWaUssDataEcho m_rawEcho_st;
    };  // rbp_Type_mp_pl_wa_Echo_st

    struct CRbpTypeMpPlWaMonitorData
    {
        CRbpTypeMpPlWaEcho   m_mp_wade_echoes[RBP_NUM_WADESENSORS][RBP_MAX_WADEASSIST_ECHOES];
        bool                 m_mp_wade_blind[RBP_NUM_WADESENSORS];
        bool                 m_mp_wade_diag_request;
    };  // rbp_Type_mp_pl_wa_MonitorData_st

    struct CPMASensorLayerOutput
    {
        vfc::uint64_t m_pma_sensorOput_nsTime_u64; 
        vfc::uint32_t m_m_cycleCounter;
        CRbpTypeMpMonitorData           m_mp_output;
        CRbpTypeVHMMonitorData          m_vhm_output;
        CRbpTypeVHMCommonOutputData     m_vhm_output_new;
        CRbpTypeVSMMonitor              m_vsm_output;
        CRbpTypeSwcpmaMonitorData       m_swcpma_output;
        CRbpTypeMpPlWaMonitorData       m_mp_wade_output;
    };  // CPMASensorLayerOutput

    // Point or Vector in a Euclidean 2D space.
    struct CRbpTypePoint
    {
        // !< x-coordinate or x-component of vector
        vfc::int16_t m_X_s16;

        // !< y-coordinate or y-component of vector
        vfc::int16_t m_Y_s16;
    };  // rbp_Type_Point_st

}// namespace pmasens

#endif // PMAAENSORLAYEROUTPUT_VFC