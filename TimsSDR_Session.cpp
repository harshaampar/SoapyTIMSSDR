#include "SoapyTimsSDR.hpp"
#include <SoapySDR/Logger.hpp>
#include <mutex>
#include <cstddef>

static std::mutex sessionMutex;
static size_t sessionCount = 0;

SoapyTimsSDRSession::SoapyTimsSDRSession(void)
{
    std::lock_guard<std::mutex> lock(sessionMutex);

    if (sessionCount == 0)
    {
        int ret = timssdr_init();
        if (ret != TIMSSDR_SUCCESS)
        {
            SoapySDR::logf(SOAPY_SDR_ERROR, "hackrf_init() failed -- %s", timssdr_error_name(timssdr_error(ret)));
        }
    }
    sessionCount++;
}

SoapyTimsSDRSession::~SoapyTimsSDRSession(void)
{
    std::lock_guard<std::mutex> lock(sessionMutex);

    sessionCount--;
    if (sessionCount == 0)
    {
        int ret = timssdr_exit();
        if (ret != TIMSSDR_SUCCESS)
        {
            SoapySDR::logf(SOAPY_SDR_ERROR, "hackrf_exit() failed -- %s", timssdr_error_name(timssdr_error(ret)));
        }
    }
}
