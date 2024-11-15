#pragma once

#include "vfc/container/vfc_fixedvector.hpp"

namespace viper
{

constexpr vfc::uint32_t GW_LANE_MAX_NUMBER = 8;

struct CXPerProcessedLaneHeader
{
    vfc::uint32_t VFC_LineHdr_TimeStamp;
    vfc::uint8_t  VFC_LineHdr_State;
    
    vfc::uint8_t  VFC_LineHdr_Blindness;
    vfc::uint8_t  VFC_LineHdr_FrameNo;
    vfc::uint8_t  VFC_LineHdr_LineCount;
    vfc::uint8_t  VFC_LineHdr_PEPSIRuntime;
    vfc::uint8_t  VFC_LineHdr_Report;
    vfc::uint32_t VFC_LineHdr_TimeGap;
};


struct CXPerProcessedLane
{
    vfc::float32_t VFC_Line_Curv;
    vfc::float32_t VFC_Line_CurvChange;
    vfc::float32_t VFC_Line_DxEnd;
    vfc::float32_t VFC_Line_DxStart;
    vfc::float32_t VFC_Line_Dy;
    vfc::float32_t VFC_Line_DyStdDevDxStart;
    vfc::float32_t VFC_Line_HeadingAngle;
    vfc::float32_t VFC_Line_ExistProb;
    vfc::uint8_t   VFC_Line_ColorClass;
    vfc::uint8_t   VFC_Line_LineID;
    vfc::uint8_t   VFC_Line_LineWidth;
    vfc::uint8_t   VFC_Line_nextID;
    vfc::uint8_t   VFC_Line_Type;

    vfc::uint8_t   VFC_Line_HypoConfidence;
    vfc::uint8_t   VFC_Line_LaneRelation;
    vfc::uint8_t   VFC_Line_MeasureType;
    vfc::uint8_t   VFC_Line_prevID;
    vfc::uint8_t   VFC_Line_ProtOrderIndex;
    vfc::float32_t VFC_Line_TypeProb;
    vfc::float32_t VFC_Line_CourseDeviation;
    vfc::float32_t VFC_Line_DyStdDevDxEnd;
    vfc::float32_t VFC_Line_DyStdDevDxMid;
};

// 0:left
// 1:right
// 2:left left
// 3:right right
// 4、5:horizontal road edge
// 6、7:vertical road edge
struct CXPerProcessedLaneInterface
{
    CXPerProcessedLaneHeader                                  m_header;
    vfc::TFixedVector<CXPerProcessedLane, GW_LANE_MAX_NUMBER> m_lanes;
};

} // namespace viper