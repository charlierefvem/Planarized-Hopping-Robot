#include "rte_PHRControl_parameters.h"
#include "PHRControl.h"
#include "PHRControl_cal.h"

extern PHRControl_cal_type PHRControl_cal_impl;
namespace slrealtime
{
  /* Description of SEGMENTS */
  SegmentVector segmentInfo {
    { (void*)&PHRControl_cal_impl, (void**)&PHRControl_cal, sizeof
      (PHRControl_cal_type), 2 }
  };

  SegmentVector &getSegmentVector(void)
  {
    return segmentInfo;
  }
}                                      // slrealtime
