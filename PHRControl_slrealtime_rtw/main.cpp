/* Main generated for Simulink Real-Time model PHRControl */
#include <ModelInfo.hpp>
#include <utilities.hpp>
#include "PHRControl.h"
#include "rte_PHRControl_parameters.h"

/* Task descriptors */
slrealtime::TaskInfo task_1( 0u, std::bind(PHRControl_step), slrealtime::TaskInfo::PERIODIC, 0.005, 0, 40);

/* Executable base address for XCP */
#ifdef __linux__
extern char __executable_start;
static uintptr_t const base_address = reinterpret_cast<uintptr_t>(&__executable_start);
#else
/* Set 0 as placeholder, to be parsed later from /proc filesystem */
static uintptr_t const base_address = 0;
#endif

/* Model descriptor */
slrealtime::ModelInfo PHRControl_Info =
{
    "PHRControl",
    PHRControl_initialize,
    PHRControl_terminate,
    []()->char const*& { return PHRControl_M->errorStatus; },
    []()->unsigned char& { return PHRControl_M->Timing.stopRequestedFlag; },
    { task_1 },
    slrealtime::getSegmentVector()
};

int main(int argc, char *argv[]) {
    slrealtime::BaseAddress::set(base_address);
    return slrealtime::runModel(argc, argv, PHRControl_Info);
}
