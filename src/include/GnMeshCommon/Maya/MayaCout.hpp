#pragma once

#include <maya/MStreamUtils.h>

#ifndef GNM_MAYA_COUT
#define GNM_MAYA_COUT MStreamUtils::stdOutStream()
#endif

#ifndef GNM_MAYA_CERR
#define GNM_MAYA_CERR MStreamUtils::stdErrStream()
#endif

#define GNM_MAYA_LOG_TO_STDCERR std::cerr.rdbuf(GNM_MAYA_COUT.rdbuf())
#define GNM_MAYA_LOG_TO_STDCOUT std::cout.rdbuf(GNM_MAYA_COUT.rdbuf())