#pragma once

#include <string>

namespace FastlioLocalizerRuntimeOverrides
{
inline std::string selectFrame(
    const std::string &configured_frame,
    const std::string &frame_override)
{
    return frame_override.empty() ? configured_frame : frame_override;
}
} // namespace FastlioLocalizerRuntimeOverrides
