#ifndef AIR_EASYTRACKRPCSERVER_HPP
#define AIR_EASYTRACKRPCSERVER_HPP

#include "common/common_utils/StrictMode.hpp"
#include "common/Common.hpp"

STRICT_MODE_OFF
#undef NOUSER
#undef FLOAT
#undef check
#include "rpc/server.h"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
STRICT_MODE_ON


namespace msr
{
namespace airlib
{

    class easyTrackRpcServer
    {
    public:
        // 构造函数，设置监听端口
        easyTrackRpcServer(const std::string& ip_address, uint16_t port);

        // 启动服务器的函数
        void startServer(int num_threads = 2);

        // 停止服务器的函数
        void stopServer();

        // 声明一个纯虚函数，供子类实现
        virtual std::vector<float> GetPlayerPosition(int32_t player_index) = 0;

        virtual ~easyTrackRpcServer() = default;

    private:
        rpc::server server_; // RPC 服务器对象
    };

} // namespace airlib
} // namespace msr

#endif // AIR_EASYTRACKRPCSERVER_HPP
