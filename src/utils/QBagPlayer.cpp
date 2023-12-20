#include "rosbag_rviz_panel/QBagPlayer.h"

#include <QDateTime>

#define MAX_PLAYBACK_SPEED 10.0
#define MIN_PLAYBACK_SPEED -10.0

namespace rosbag_rviz_panel {

QBagPlayer::QBagPlayer(QObject* parent) : QObject(parent), _nh("~")
{
    ros::Time::init();
}

QBagPlayer::~QBagPlayer()
{
    receiveSetPause();

    if (_bag.isOpen())
        _bag.close();
}

void QBagPlayer::receiveLoadBag(const QString filename)
{
    receiveSetPause();
    resetTxt();

    if (!_reverse_bag.empty())
        _reverse_bag.clear();

    if (!_pubs.empty())
        _pubs.clear();

    QString loading_msg("Loading " + filename + "...");
    ROS_INFO_STREAM(loading_msg.toStdString());
    Q_EMIT sendStatusText(loading_msg);

    if (_bag.isOpen())
        _bag.close();

    try {
        _bag.open(filename.toStdString(), rosbag::bagmode::Read);
    } catch (const rosbag::BagIOException& r) {
        ROS_ERROR_STREAM(r.what());
        Q_EMIT sendStatusText(QString::fromStdString(r.what()));
        Q_EMIT sendEnableActionButtons(false);
        return;
    }

    _full_view.reset();
    _full_view = std::make_unique<rosbag::View>(_bag);
    reset();

    for (rosbag::View::iterator iter = _full_view->begin(); iter != _full_view->end(); ++iter) {
        _reverse_bag.push_back(iter);
    }

    _full_bag_start = _full_view->getBeginTime();
    _full_bag_end   = _full_view->getEndTime();

    _last_message_time = ros::Time(0);
    _playback_speed    = 1.0;

    Q_EMIT sendBagFinished();

    for (const auto& info : _full_view->getConnections()) {
        try {
            ros::AdvertiseOptions opts = createAdvertiseOptions(info, 1, "");
            ros::Publisher        pub  = _nh.advertise(opts);
            _pubs[info->topic]         = _nh.advertise(opts);

        } catch (const std::runtime_error& e) {
            ROS_ERROR_STREAM(e.what());
            Q_EMIT sendStatusText(QString::fromStdString(e.what()));
        }
    }

    Q_EMIT sendStatusText("");
    Q_EMIT sendEnableActionButtons(true);

    sizeToStr(_bag.getSize());
    Q_EMIT sendStampLabel(QString::number(_full_view->getBeginTime().toSec(), 'f', 9));
    Q_EMIT sendDateLabel(QDateTime::fromSecsSinceEpoch(_full_view->getBeginTime().toSec(), Qt::UTC)
                                 .toString("dd.MM.yyyy hh::mm::ss"));
    Q_EMIT sendPlayspeedLabel("x" + QString::number(_playback_speed));
    Q_EMIT sendSecondsLabel(
            QString::number(_last_message_time.toSec(), 'f', 2) + "/"
            + QString::number(_full_bag_end.toSec() - _full_bag_start.toSec(), 'f', 2) + "s");
    Q_EMIT sendPlayheadProgress(_last_message_time.toSec() / (_full_bag_end.toSec() - _full_bag_start.toSec()) * 100);
}

void QBagPlayer::receiveSetStart(const ros::Time& start)
{
    std::lock_guard<std::mutex> lock(_playback_mutex);

    if (_playback_speed > 0) {
        _bag_control_start = start;

        if (_playback_direction_changed)
            _bag_control_end = _full_bag_end;
    } else {
        _bag_control_end = start;

        if (_playback_direction_changed)
            _bag_control_start = _full_bag_start;
    }

    _last_message_time = start;
}

void QBagPlayer::receiveSetEnd(const ros::Time& end)
{
    std::lock_guard<std::mutex> lock(_playback_mutex);

    if (_playback_speed > 0) {
        _bag_control_end = end;

        if (_playback_direction_changed)
            _bag_control_start = _full_bag_start;
    } else {
        _bag_control_start = end;

        if (_playback_direction_changed)
            _bag_control_end = _full_bag_end;
    }

    _last_message_time = end;
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
        if (_last_message_time == ros::Time(0)) {
            if (change < 0.0)
                _last_message_time = _full_bag_end;
        }
        receiveSetStart(_last_message_time);
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
        ROS_DEBUG_STREAM("QBagPlayer is already running!");
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

    Q_EMIT sendStampLabel(QString::number(_full_bag_start.toSec(), 'f', 9));
    Q_EMIT sendDateLabel(
            QDateTime::fromSecsSinceEpoch(_full_bag_start.toSec(), Qt::UTC).toString("dd.MM.yyyy hh::mm::ss"));

    const auto progress = _last_message_time.toSec() - _full_bag_start.toSec();
    Q_EMIT sendSecondsLabel(
            QString::number(progress, 'f', 2) + "/"
            + QString::number(_full_bag_end.toSec() - _full_bag_start.toSec(), 'f', 2) + "s");
    Q_EMIT sendPlayheadProgress(progress / (_full_bag_end.toSec() - _full_bag_start.toSec()) * 100);
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

    Q_EMIT sendStampLabel(QString::number(_full_bag_end.toSec(), 'f', 9));
    Q_EMIT sendDateLabel(
            QDateTime::fromSecsSinceEpoch(_full_bag_end.toSec(), Qt::UTC).toString("dd.MM.yyyy hh::mm::ss"));

    const auto progress = _last_message_time.toSec() - _full_bag_start.toSec();
    Q_EMIT sendSecondsLabel(
            QString::number(progress, 'f', 2) + "/"
            + QString::number(_full_bag_end.toSec() - _full_bag_start.toSec(), 'f', 2) + "s");
    Q_EMIT sendPlayheadProgress(progress / (_full_bag_end.toSec() - _full_bag_start.toSec()) * 100);
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
        receiveSetStart(progress_stamp);
        receiveStartPlaying();
    } else
        receiveSetStart(progress_stamp);
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

    rosbag::View view(_bag, _bag_control_start, _bag_control_end);

    if (_playback_speed > 0) {
        _play_start = ros::Time::now();

        for (rosbag::MessageInstance const& m : view) {
            if (_pubs.find(m.getTopic()) == _pubs.end())
                continue;

            {
                std::lock_guard<std::mutex> lock(_pause_mutex);
                if (_pause) {
                    _last_message_time = m.getTime();
                    break;
                }
            }

            ros::Time::sleepUntil(real_time(m.getTime()));

            _last_message_time = m.getTime();
            _pubs[m.getTopic()].publish(m);

            Q_EMIT sendStampLabel(QString::number(_last_message_time.toSec(), 'f', 9));
            Q_EMIT sendDateLabel(QDateTime::fromSecsSinceEpoch(_last_message_time.toSec(), Qt::UTC)
                                         .toString("dd.MM.yyyy hh::mm::ss"));

            const auto progress = _last_message_time.toSec() - _full_bag_start.toSec();
            Q_EMIT sendSecondsLabel(
                    QString::number(progress, 'f', 2) + "/"
                    + QString::number(_full_bag_end.toSec() - _full_bag_start.toSec(), 'f', 2) + "s");
            Q_EMIT sendPlayheadProgress(progress / (_full_bag_end.toSec() - _full_bag_start.toSec()) * 100);
        }
    } else {
        auto rfound = std::find_if(_reverse_bag.rbegin(), _reverse_bag.rend(), [this](rosbag::View::iterator& it) {
            return it->getTime() <= _bag_control_end;
        });
        if (rfound != _reverse_bag.rend()) {
            _play_start = ros::Time::now();

            for (auto r_iter = rfound; r_iter != _reverse_bag.rend(); ++r_iter) {
                const auto& m = *(*r_iter);

                if (_pubs.find(m.getTopic()) == _pubs.end())
                    continue;

                {
                    std::lock_guard<std::mutex> lock(_pause_mutex);
                    if (_pause) {
                        _last_message_time = m.getTime();
                        break;
                    }
                }

                ros::Time::sleepUntil(real_time(m.getTime()));

                _last_message_time = m.getTime();
                _pubs[m.getTopic()].publish(m);

                Q_EMIT sendStampLabel(QString::number(_last_message_time.toSec(), 'f', 9));
                Q_EMIT sendDateLabel(QDateTime::fromSecsSinceEpoch(_last_message_time.toSec(), Qt::UTC)
                                             .toString("dd.MM.yyyy hh::mm::ss"));

                const auto progress = _last_message_time.toSec() - _full_bag_start.toSec();
                Q_EMIT sendSecondsLabel(
                        QString::number(progress, 'f', 2) + "/"
                        + QString::number(_full_bag_end.toSec() - _full_bag_start.toSec(), 'f', 2) + "s");
                Q_EMIT sendPlayheadProgress(progress / (_full_bag_end.toSec() - _full_bag_start.toSec()) * 100);
            }
        } else
            ROS_WARN_STREAM("Could not find a suitable reversed time stamped message");
    }

    if (!_pause) {
        reset();
        Q_EMIT sendBagFinished();
    } else {
        receiveSetStart(_last_message_time);
    }

    {
        std::lock_guard<std::mutex> lock(_thread_mutex);
        _thread_running = false;
    }
}

ros::AdvertiseOptions QBagPlayer::createAdvertiseOptions(
        const rosbag::ConnectionInfo* c,
        uint32_t                      queue_size,
        const std::string&            prefix)
{
    ros::AdvertiseOptions opts(prefix + c->topic, queue_size, c->md5sum, c->datatype, c->msg_def);
    opts.latch = isLatching(c);
    return opts;
}

bool QBagPlayer::isLatching(const rosbag::ConnectionInfo* c)
{
    ros::M_string::const_iterator header_iter = c->header->find("latching");
    return (header_iter != c->header->end() && header_iter->second == "1");
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
    _bag_control_start          = _full_view->getBeginTime();
    _bag_control_end            = _full_view->getEndTime();
    _playback_direction_changed = false;
}

void QBagPlayer::resetTxt(void)
{
    Q_EMIT sendStampLabel("");
    Q_EMIT sendDateLabel("");
    Q_EMIT sendPlayspeedLabel("");
    Q_EMIT sendSecondsLabel("");
    Q_EMIT sendStatusText("");
    Q_EMIT sendBagSize("");
    Q_EMIT sendPlayheadProgress(0);
}

ros::Time QBagPlayer::real_time(const ros::Time& msg_time)
{
    std::lock_guard<std::mutex> lock(_playback_mutex);

    if (_playback_speed > 0.0)
        return _play_start + (msg_time - _bag_control_start) * (1 / _playback_speed);
    else
        return _play_start + (_bag_control_end - msg_time) * (1 / std::abs(_playback_speed));
}

ros::Time QBagPlayer::getProgressTime(const int progress)
{
    auto      bag_duration = _full_bag_end.toSec() - _full_bag_start.toSec();
    ros::Time new_start_stamp;
    return new_start_stamp.fromSec(_full_bag_start.toSec() + bag_duration * progress / 100);
}
} // namespace rosbag_rviz_panel