#pragma once


#if not defined(AEROX_IS_WINDOWS)

#include <atomic>
#include <mutex>
#include <optional>

#include <plugins/rc_passthrough/PluginRC.h>
#include "plugin_impl_base.h"

#include <mavsdk.h>
#include "plugins/mavlink_passthrough/mavlink_passthrough.h"
#include <system.h>

namespace mavsdk {

class System;

class PluginRCImpl : public PluginImplBase {
public:
    explicit PluginRCImpl(System& _system);
    explicit PluginRCImpl(std::shared_ptr<System> _system);
    ~PluginRCImpl();

    void init() override;
    void deinit() override;

    void enable() override  {};
    void disable() override  {};

    PluginRC::RcChannels rc_channels() const;

    void subscribe_rc_channels(PluginRC::RcChannelsCallback& _callback);

    PluginRCImpl(const PluginRCImpl&) = delete;
    PluginRCImpl& operator=(const PluginRCImpl&) = delete;

private:

    void set_rc_channels(mavlink_rc_channels_t _rc_channels);
    void process_rc_channels(const mavlink_message_t& _message);

    void receive_rc_channels_timeout();


    static PluginRC::Result pluginRC_result_from_command_result(MavlinkCommandSender::Result _command_result);

    static void command_result_callback(MavlinkCommandSender::Result _command_result, const PluginRC::ResultCallback& _callback);

    // Make all fields thread-safe using mutexs
    // The mutexs are mutable so that the lock can get aqcuired in
    // methods marked const.
    
    mutable std::mutex rc_channels_mutex_{};
    PluginRC::RcChannels rc_channels_{};

    std::mutex subscription_mutex_{};

    PluginRC::RcChannelsCallback rc_channels_subscription_{nullptr};

    // The velocity (former ground speed) and position are coupled to the same message, therefore,
    // we just use the faster between the two.

    void* rc_channels_timeout_cookie_{nullptr};
};
} // namespace mavsdk

#endif
