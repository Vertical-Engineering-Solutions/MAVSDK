
#include <plugins/rc_passthrough/PluginRC_impl.h>

#if not defined(AEROX_IS_WINDOWS)

#include <system.h>
#include <global_include.h>

#include <cmath>
#include <functional>
#include <string>
#include <array>
#include <cassert>

namespace mavsdk {

PluginRCImpl::PluginRCImpl(System& _system) : PluginImplBase(_system)
{
    _parent->register_plugin(this);
}

PluginRCImpl::PluginRCImpl(std::shared_ptr<System> _system) : PluginImplBase(_system)
{
    _parent->register_plugin(this);
}

PluginRCImpl::~PluginRCImpl()
{
    _parent->unregister_plugin(this);
}

void PluginRCImpl::init()
{
    using namespace std::placeholders; // for `_1`

    _parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_RC_CHANNELS, std::bind(&PluginRCImpl::process_rc_channels, this, _1), this);
}

void PluginRCImpl::deinit()
{
    _parent->unregister_all_mavlink_message_handlers(this);
}

PluginRC::Result
PluginRCImpl::pluginRC_result_from_command_result(MavlinkCommandSender::Result _command_result)
{
    switch (_command_result) {
        case MavlinkCommandSender::Result::Success:
            return PluginRC::Result::Success;
        case MavlinkCommandSender::Result::NoSystem:
            return PluginRC::Result::NoSystem;
        case MavlinkCommandSender::Result::ConnectionError:
            return PluginRC::Result::ConnectionError;
        case MavlinkCommandSender::Result::Busy:
            return PluginRC::Result::Busy;
        case MavlinkCommandSender::Result::CommandDenied:
            return PluginRC::Result::CommandDenied;
        case MavlinkCommandSender::Result::Timeout:
            return PluginRC::Result::Timeout;
        case MavlinkCommandSender::Result::Unsupported:
            return PluginRC::Result::Unsupported;
        default:
            return PluginRC::Result::Unknown;
    }
}

void PluginRCImpl::command_result_callback(
    MavlinkCommandSender::Result _command_result, const PluginRC::ResultCallback& _callback)
{
    PluginRC::Result action_result = pluginRC_result_from_command_result(_command_result);

    _callback(action_result);
}

void PluginRCImpl::process_rc_channels(const mavlink_message_t& _message)
{
    mavlink_rc_channels_t rc_channels_msg;
    mavlink_msg_rc_channels_decode(&_message, &rc_channels_msg);

    set_rc_channels(rc_channels_msg);

    std::lock_guard<std::mutex> lock(subscription_mutex_);
    if (rc_channels_subscription_) {
        auto callback = rc_channels_subscription_;
        auto arg = rc_channels();
        _parent->call_user_callback([callback, arg]() { callback(arg); });
    }

    _parent->refresh_timeout_handler(rc_channels_timeout_cookie_);
}

PluginRC::RcChannels PluginRCImpl::rc_channels() const
{
    std::lock_guard<std::mutex> lock(rc_channels_mutex_);
    return rc_channels_;
}


void PluginRCImpl::set_rc_channels(mavlink_rc_channels_t _rc_channels)
{
    const float minRcMs = 980;
    const float maxRcMs = 2020;
    const float minRcSbus = 0;
    const float maxRcSbus = 2048;


    ///                          1
    ///     mapRatio = ---------------------- * (maxRcSbus - minRcSbus) 
    ///                 (maxRcMs - minRcMs)
    float mapRatio = 1/(maxRcMs - minRcMs) * (maxRcSbus - minRcSbus);

    if(_rc_channels.chancount != 0)
    {
        rc_channels_.channels[0 ] = minRcSbus + (_rc_channels.chan1_raw - minRcMs)*mapRatio;
        rc_channels_.channels[1 ] = minRcSbus + (_rc_channels.chan2_raw - minRcMs)*mapRatio;
        rc_channels_.channels[2 ] = minRcSbus + (_rc_channels.chan3_raw - minRcMs)*mapRatio;
        rc_channels_.channels[3 ] = minRcSbus + (_rc_channels.chan4_raw - minRcMs)*mapRatio;
        rc_channels_.channels[4 ] = minRcSbus + (_rc_channels.chan5_raw - minRcMs)*mapRatio;
        rc_channels_.channels[5 ] = minRcSbus + (_rc_channels.chan6_raw - minRcMs)*mapRatio;
        rc_channels_.channels[6 ] = minRcSbus + (_rc_channels.chan7_raw - minRcMs)*mapRatio;
        rc_channels_.channels[7 ] = minRcSbus + (_rc_channels.chan8_raw - minRcMs)*mapRatio;
        rc_channels_.channels[8 ] = minRcSbus + (_rc_channels.chan9_raw - minRcMs)*mapRatio;
        rc_channels_.channels[9 ] = minRcSbus + (_rc_channels.chan10_raw - minRcMs)*mapRatio;
        rc_channels_.channels[10] = minRcSbus + (_rc_channels.chan11_raw - minRcMs)*mapRatio;
        rc_channels_.channels[11] = minRcSbus + (_rc_channels.chan12_raw - minRcMs)*mapRatio;
        rc_channels_.channels[12] = minRcSbus + (_rc_channels.chan13_raw - minRcMs)*mapRatio;
        rc_channels_.channels[13] = minRcSbus + (_rc_channels.chan14_raw - minRcMs)*mapRatio;
        rc_channels_.channels[14] = minRcSbus + (_rc_channels.chan15_raw - minRcMs)*mapRatio;
        rc_channels_.channels[15] = minRcSbus + (_rc_channels.chan16_raw - minRcMs)*mapRatio;
        rc_channels_.channels[16] = minRcSbus + (_rc_channels.chan17_raw - minRcMs)*mapRatio;
        rc_channels_.channels[17] = minRcSbus + (_rc_channels.chan18_raw - minRcMs)*mapRatio;
        rc_channels_.status = true;
    }
    else
    {
        rc_channels_.channels.clear();
        rc_channels_.status = false;
    }
}

void PluginRCImpl::subscribe_rc_channels(PluginRC::RcChannelsCallback& _callback)
{
    std::lock_guard<std::mutex> lock(subscription_mutex_);
    rc_channels_subscription_ = _callback;
}

} // namespace mavsdk

#endif
