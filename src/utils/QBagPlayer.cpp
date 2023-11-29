#include "rosbag_rviz_panel/QBagPlayer.hpp"

#include <rcutils/time.h>

#include <QDateTime>
#include <rosbag2_cpp/typesupport_helpers.hpp>

#define MAX_PLAYBACK_SPEED 10.0
#define MIN_PLAYBACK_SPEED 0.5

namespace {
rcl_publisher_options_t rosbag2_get_publisher_options(const rclcpp::QoS& qos)
{
    auto options = rcl_publisher_get_default_options();
    options.qos  = qos.get_rmw_qos_profile();
    return options;
}
} // unnamed namespace

namespace rosbag_rviz_panel {

GenericPublisher::GenericPublisher(
        rclcpp::node_interfaces::NodeBaseInterface* node_base,
        const rosidl_message_type_support_t&        type_support,
        const std::string&                          topic_name,
        const rclcpp::QoS&                          qos) :
        rclcpp::PublisherBase(node_base, topic_name, type_support, rosbag2_get_publisher_options(qos))
{}

void GenericPublisher::publish(std::shared_ptr<rmw_serialized_message_t> message)
{
    auto return_code = rcl_publish_serialized_message(this->get_publisher_handle().get(), message.get(), NULL);

    if (return_code != RCL_RET_OK) {
        rclcpp::exceptions::throw_from_rcl_error(return_code, "failed to publish serialized message");
    }
}
} // namespace rosbag_rviz_panel

namespace rosbag_rviz_panel {

QBagPlayer::QBagPlayer(QObject* parent) : QObject(parent)
{
    _nh = std::make_shared<rclcpp::Node>("QBagPlayer_node");
}

QBagPlayer::~QBagPlayer()
{
    receiveSetPause();
}

void QBagPlayer::receiveLoadBag(const QString filename)
{
    receiveSetPause();
    resetTxt();

    if (!_pubs.empty())
        _pubs.clear();

    QString loading_msg("Loading " + filename + "...");
    RCLCPP_INFO_STREAM(_nh->get_logger(), loading_msg.toStdString());
    Q_EMIT sendStatusText(loading_msg);

    try {
        rosbag2_cpp::StorageOptions storage_options;
        storage_options.uri        = filename.toStdString();
        storage_options.storage_id = "sqlite3";
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format  = "cdr";
        converter_options.output_serialization_format = "cdr";

        _reader.reset();
        _reader = std::make_shared<rosbag2_cpp::readers::SequentialReader>();
        _reader->open(storage_options, converter_options);
    } catch (const std::exception& r) {
        RCLCPP_ERROR_STREAM(_nh->get_logger(), r.what());
        Q_EMIT sendStatusText(QString::fromStdString(r.what()));
        Q_EMIT sendEnableActionButtons(false);
        return;
    }

    auto metadata   = _reader->get_metadata();
    _full_bag_start = metadata.starting_time.time_since_epoch();
    _full_bag_end   = _full_bag_start + metadata.duration;

    reset();

    /// \note: Set to zero initially:
    _last_message_time = std::chrono::nanoseconds::zero();
    _playback_speed    = 1.0;

    Q_EMIT sendBagFinished();

    /// \note: Manually set the QoS:
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    for (const auto& topic : _reader->get_all_topics_and_types()) {
        try {
            _pubs[topic.name] = createGenericPublisher(topic.name, topic.type, qos);
        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR_STREAM(_nh->get_logger(), e.what());
            Q_EMIT sendStatusText(QString::fromStdString(e.what()));
        }
    }

    Q_EMIT sendStatusText("");
    Q_EMIT sendEnableActionButtons(true);

    sizeToStr(_reader->get_metadata().bag_size);
    Q_EMIT sendStampLabel(QString::number(_full_bag_start.count() * 1e-9, 'f', 9));
    Q_EMIT sendDateLabel(
            QDateTime::fromSecsSinceEpoch(_full_bag_start.count() * 1e-9, Qt::UTC).toString("dd.MM.yyyy hh::mm::ss"));
    Q_EMIT sendPlayspeedLabel("x" + QString::number(_playback_speed));
    Q_EMIT sendSecondsLabel(
            QString::number(_last_message_time.count() * 1e-9, 'f', 2) + "/"
            + QString::number(_full_bag_end.count() * 1e-9 - _full_bag_start.count() * 1e-9, 'f', 2) + "s");
    Q_EMIT sendPlayheadProgress(
            _last_message_time.count() * 1e-9 / (_full_bag_end.count() * 1e-9 - _full_bag_start.count() * 1e-9) * 100);
}

void QBagPlayer::receiveSetStart(const double start)
{
    std::lock_guard<std::mutex> lock(_playback_mutex);

    if (_playback_speed > 0) {
        _bag_control_start = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(start));

        if (_playback_direction_changed)
            _bag_control_end = _full_bag_end;
    } else {
        _bag_control_end = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(start));

        if (_playback_direction_changed)
            _bag_control_start = _full_bag_start;
    }

    _last_message_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(start));
}

void QBagPlayer::receiveSetEnd(const double end)
{
    std::lock_guard<std::mutex> lock(_playback_mutex);

    if (_playback_speed > 0) {
        _bag_control_end = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(end));

        if (_playback_direction_changed)
            _bag_control_start = _full_bag_start;
    } else {
        _bag_control_start = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(end));

        if (_playback_direction_changed)
            _bag_control_end = _full_bag_end;
    }

    _last_message_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(end));
}

void QBagPlayer::receiveChangeSpeed(const float change)
{
    _playback_direction_changed = false;

    {
        std::lock_guard<std::mutex> lock(_playback_mutex);

        _playback_speed = _playback_speed + change;
        if (_playback_speed > MAX_PLAYBACK_SPEED)
            _playback_speed = MAX_PLAYBACK_SPEED;
        else if (_playback_speed < MIN_PLAYBACK_SPEED)
            _playback_speed = MIN_PLAYBACK_SPEED;

        if (_playback_speed == 0.0) {
            _playback_speed             = change;
            _playback_direction_changed = true;
        }
    }

    Q_EMIT sendPlayspeedLabel("x" + QString::number(_playback_speed));

    bool thread_running;
    {
        std::lock_guard<std::mutex> lock(_thread_mutex);
        thread_running = _thread_running;
    }

    if (_playback_direction_changed && !thread_running) {
        if (_last_message_time == std::chrono::nanoseconds::zero()) {
            if (change < 0.0)
                _last_message_time = _full_bag_end;
        }
        receiveSetStart(_last_message_time.count());
    }

    if (thread_running) {
        receiveSetPause();
        receiveStartPlaying();
    }
}

void QBagPlayer::receiveSetPause(void)
{
    {
        std::lock_guard<std::mutex> lock(_pause_mutex);
        _pause = true;
    }

    if (_play_thread.joinable()) {
        _play_thread.join();
    }
}

void QBagPlayer::receiveStartPlaying(void)
{
    bool thread_running;
    {
        std::lock_guard<std::mutex> lock(_thread_mutex);
        thread_running = _thread_running;
    }

    if (!thread_running) {
        _play_thread = std::thread(&QBagPlayer::run, this);
    } else
        RCLCPP_DEBUG_STREAM(_nh->get_logger(), "QBagPlayer is already running!");
}

void QBagPlayer::receiveGotoBegin(void)
{
    bool thread_running;
    {
        std::lock_guard<std::mutex> lock(_thread_mutex);
        thread_running = _thread_running;
    }

    if (thread_running) {
        receiveSetPause();
        Q_EMIT sendBagFinished();
    }

    reset();
    _last_message_time = _full_bag_start;

    Q_EMIT sendStampLabel(QString::number(_full_bag_start.count() * 1e-9, 'f', 9));
    Q_EMIT sendDateLabel(
            QDateTime::fromSecsSinceEpoch(_full_bag_start.count() * 1e-9, Qt::UTC).toString("dd.MM.yyyy hh::mm::ss"));

    const auto progress = _last_message_time.count() * 1e-9 - _full_bag_start.count() * 1e-9;
    Q_EMIT sendSecondsLabel(
            QString::number(progress, 'f', 2) + "/"
            + QString::number(_full_bag_end.count() * 1e-9 - _full_bag_start.count() * 1e-9, 'f', 2) + "s");
    Q_EMIT sendPlayheadProgress(progress / (_full_bag_end.count() * 1e-9 - _full_bag_start.count() * 1e-9) * 100);
}

void QBagPlayer::receiveGotoEnd(void)
{
    bool thread_running;
    {
        std::lock_guard<std::mutex> lock(_thread_mutex);
        thread_running = _thread_running;
    }

    if (thread_running) {
        receiveSetPause();
        Q_EMIT sendBagFinished();
    }

    reset();
    _last_message_time = _full_bag_end;

    Q_EMIT sendStampLabel(QString::number(_full_bag_start.count() * 1e-9, 'f', 9));
    Q_EMIT sendDateLabel(
            QDateTime::fromSecsSinceEpoch(_full_bag_start.count() * 1e-9, Qt::UTC).toString("dd.MM.yyyy hh::mm::ss"));

    const auto progress = _last_message_time.count() * 1e-9 - _full_bag_start.count() * 1e-9;
    Q_EMIT sendSecondsLabel(
            QString::number(progress, 'f', 2) + "/"
            + QString::number(_full_bag_end.count() * 1e-9 - _full_bag_start.count() * 1e-9, 'f', 2) + "s");
    Q_EMIT sendPlayheadProgress(progress / (_full_bag_end.count() * 1e-9 - _full_bag_start.count() * 1e-9) * 100);
}

void QBagPlayer::receiveClickedProgress(int value)
{
    bool thread_running;
    {
        std::lock_guard<std::mutex> lock(_thread_mutex);
        thread_running = _thread_running;
    }

    const auto progress_stamp = getProgressTime(value);

    if (thread_running) {
        receiveSetPause();
        receiveSetStart(progress_stamp.count());
        receiveStartPlaying();
    } else
        receiveSetStart(progress_stamp.count());
}

void QBagPlayer::run(void)
{
    {
        std::lock_guard<std::mutex> lock(_pause_mutex);
        _pause = false;
    }

    {
        std::lock_guard<std::mutex> lock(_thread_mutex);
        _thread_running = true;
    }

    if (_playback_speed > 0) {
        _play_start = std::chrono::system_clock::now();

        while (_reader->has_next()) {
            const auto& m = _reader->read_next();

            if (_pubs.find(m->topic_name) == _pubs.end())
                continue;

            if (m->time_stamp < _bag_control_start.count() || m->time_stamp > _bag_control_end.count()) {
                std::cout << "Timestamp not in range: " << m->time_stamp * 1e-9 << std::endl;
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(_pause_mutex);
                if (_pause) {
                    _last_message_time = std::chrono::nanoseconds(m->time_stamp);
                    break;
                }
            }

            std::this_thread::sleep_until(realTimeDuration(m->time_stamp));

            _last_message_time = std::chrono::nanoseconds(m->time_stamp);
            _pubs[m->topic_name]->publish(m->serialized_data);

            Q_EMIT sendStampLabel(QString::number(_last_message_time.count() * 1e-9, 'f', 9));
            Q_EMIT sendDateLabel(QDateTime::fromSecsSinceEpoch(_last_message_time.count() * 1e-9, Qt::UTC)
                                         .toString("dd.MM.yyyy hh::mm::ss"));

            const auto progress = _last_message_time.count() * 1e-9 - _full_bag_start.count() * 1e-9;
            Q_EMIT sendSecondsLabel(
                    QString::number(progress, 'f', 2) + "/"
                    + QString::number(_full_bag_end.count() * 1e-9 - _full_bag_start.count() * 1e-9, 'f', 2) + "s");
            Q_EMIT sendPlayheadProgress(
                    progress / (_full_bag_end.count() * 1e-9 - _full_bag_start.count() * 1e-9) * 100);
        }
    }

    if (!_pause) {
        reset();
        Q_EMIT sendBagFinished();
    } else {
        receiveSetStart(_last_message_time.count());
    }

    {
        std::lock_guard<std::mutex> lock(_thread_mutex);
        _thread_running = false;
    }
}

void QBagPlayer::sizeToStr(const uint64_t size)
{
    const QStringList size_name = {"B", "KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB"};
    const int         i         = static_cast<int>(std::floor(std::log(size) / std::log(1024)));
    const double      p         = std::pow(1024, i);
    const double      s         = round(size / p * 100) / 100;
    if (s > 0) {
        Q_EMIT sendBagSize(QString::number(s, 'f', 2) + " " + size_name[i]);
        return;
    }

    Q_EMIT sendBagSize(QString("0 B"));
}

void QBagPlayer::reset(void)
{
    _bag_control_start          = _full_bag_start;
    _bag_control_end            = _full_bag_end;
    _playback_direction_changed = false;
}

void QBagPlayer::resetTxt(void)
{
    Q_EMIT sendStampLabel("");
    Q_EMIT sendDateLabel("");
    Q_EMIT sendPlayspeedLabel("");
    Q_EMIT sendSecondsLabel("");
    Q_EMIT sendStatusText("");
    Q_EMIT sendPlayheadProgress(0);
}

std::chrono::_V2::system_clock::time_point QBagPlayer::realTimeDuration(const rcutils_time_point_value_t& msg_time)
{
    std::lock_guard<std::mutex> lock(_playback_mutex);

    if (_playback_speed > 0.0) {
        return _play_start
             + std::chrono::duration_cast<std::chrono::nanoseconds>(
                       (std::chrono::nanoseconds(msg_time) - _bag_control_start) * (1.0 / _playback_speed));
    } else {
        return _play_start
             + std::chrono::duration_cast<std::chrono::nanoseconds>(
                       (_bag_control_end - std::chrono::nanoseconds(msg_time)) * (1.0 / std::abs(_playback_speed)));
    }
}

std::chrono::nanoseconds QBagPlayer::getProgressTime(const int progress)
{
    auto bag_duration = _full_bag_end - _full_bag_start;
    return _full_bag_start + bag_duration * progress / 100;
}

std::shared_ptr<GenericPublisher> QBagPlayer::createGenericPublisher(
        const std::string& topic,
        const std::string& type,
        const rclcpp::QoS& qos)
{
    _library_generic_publisher = rosbag2_cpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
    auto type_support = rosbag2_cpp::get_typesupport_handle(type, "rosidl_typesupport_cpp", _library_generic_publisher);
    return std::make_shared<GenericPublisher>(_nh->get_node_base_interface().get(), *type_support, topic, qos);
}
} // namespace rosbag_rviz_panel