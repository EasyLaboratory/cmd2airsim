#ifndef AIR_EASYTRACKRPCCLIENT_HPP
#define AIR_EASYTRACKRPCCLIENT_HPP

// #include "common/common_utils/StrictMode.hpp"
#include "common/Common.hpp"

// STRICT_MODE_OFF
// #undef NOUSER
// #undef FLOAT
// #undef check
// #include "rpc/client.h"
// //TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
// #ifndef check
// #define check(expr) (static_cast<void>((expr)))
// #endif
// STRICT_MODE_ON

#include "rpc/client.h"
#include "common/Common.hpp"
#include <functional>
#include <memory>
#include "common/CommonStructs.hpp"

namespace msr
{
namespace airlib
{

    class easyTrackRpcClient
    {
    public:
        easyTrackRpcClient(const std::string& ip_address = "127.0.0.1", uint16_t port = 50502, float timeout_sec = 60);

        Eigen::Vector3f getPlayerPosition(int32_t player_index);

    private:
        
        rpc::client client_; 
    };

}
} // namespace msr::airlib




#endif // AIR_EASYTRACKRPCCLIENT_HPP