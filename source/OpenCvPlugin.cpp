// ============================================================================
// OpenCvPlugin — Top-level exports
// ============================================================================

#include "OpenCvPlugin.h"

// Version as a single integer: major * 10000 + minor * 100 + patch
// e.g. 1.0.0 = 10000
#define OCP_VERSION 10000

extern "C" {

    /// Returns the plugin version as an integer (major*10000 + minor*100 + patch).
    /// Useful for C# to verify DLL compatibility at load time.
    OCP_EXPORT int OCP_GetVersion() {
        return OCP_VERSION;
    }

}