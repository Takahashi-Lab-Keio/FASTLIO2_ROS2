#pragma once

#include <string>

namespace FastlioRuntimeOverrides
{
inline std::string selectFrame(
    const std::string &configured_frame,
    const std::string &frame_override)
{
    return frame_override.empty() ? configured_frame : frame_override;
}

inline bool requirePointTime(
    bool configured_requirement,
    bool allow_missing_point_time)
{
    return allow_missing_point_time ? false : configured_requirement;
}
} // namespace FastlioRuntimeOverrides
