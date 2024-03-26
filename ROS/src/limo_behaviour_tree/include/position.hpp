#include "behaviortree_cpp_v3/bt_factory.h"

struct Position3D
{
    double x;
    double y;
    double z;
};

namespace BT
{
    template <> inline Position3D convertFromString(StringView str)
    {
        auto parts = splitString(str,';');
        if(parts.size() !=3)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            Position3D output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            output.z = convertFromString<double>(parts[2]);
            return output;
        }
    }
}