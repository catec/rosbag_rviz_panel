#pragma once

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <QObject>
#include <QString>
#include <mutex>
#include <thread>

namespace rosbag_rviz_panel {

/**
 * @brief QBagPlayer.
 *
 * This custom QOBject opens a rosbag and plays it, forward or
 * backwards, at different speed rates.
 *
 */
class ROSBAG_STORAGE_DECL QBagPlayer : public QObject
{
    Q_OBJECT

  public:
    /**
     * @brief Constructor of the QBagPlayer class.
     *
     * @param parent A parent QObject, if there is one.
     */
    explicit QBagPlayer(QObject* parent = nullptr);

    /**
     * @brief Destructor of the QBagPlayer class.
     */
    virtual ~QBagPlayer();

  private:
    /**
     * @brief Main loop to play rosbags, forward or backwards.
     */
    void run(void);

    /**
     * @brief Create a ros::AdvertiseOptions object to create
     * a publisher for the given topic.
     *
     * @param c rosbag::ConnectionInfo to check if the topic is
     *          latching its messages.
     * @param queue_size uint32_t with the size of the queue for
     *        the ros publisher.
     * @param prefix std::string with an optional prefix for the
     *        topic to publish to.
     *
     * @return ros::AdvertiseOptions with all the given info.
     */
    ros::AdvertiseOptions createAdvertiseOptions(
            const rosbag::ConnectionInfo* c,
            uint32_t                      queue_size,
            const std::string&            prefix);

    /**
     * @brief Checks if latch was true or false for a topic.
     *
     * @param c rosbag::ConnectionInfo to check if the topic is
     *          latching its messages.
     *
     * @return bool with the value of the latching option.
     */
    bool isLatching(const rosbag::ConnectionInfo* c);

    /**
     * @brief Converts the size of a rosbag into
     * {"B", "KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB"}.
     *
     * @param size uint64_t with the size of the rosbag.
     *
     */
    void sizeToStr(const uint64_t size);

    /**
     * @brief Resets some variables to restart bag replay.
     */
    void reset(void);

    /**
     * @brief Resets all UI text labels.
     */
    void resetTxt(void);

    /**
     * @brief Calculate the time stamp to send ROS to sleep
     * until that stamp has been reached.
     *
     * @param msg_time ros::Time with the last proccessed
     *        message time stamp.
     *
     * @return ros::Time with the time stam to sleep until.
     */
    ros::Time real_time(const ros::Time& msg_time);

    /**
     * @brief Calculate the time stamp to start playing from
     * the clicked progress bar value.
     *
     * @param progress Int value of the progress bar [0, 100]
     *
     * @return ros::Time with the calculated time stamp.
     */
    ros::Time getProgressTime(const int progress);

  Q_SIGNALS:
    /**
     * @brief Q_SIGNAL to notify that the bag is completed.
     */
    void sendBagFinished(void);

    /**
     * @brief Q_SIGNAL that sends the total size of the bag.
     *
     * @param size QString that contains the total bag size.
     */
    void sendBagSize(const QString size);

    /**
     * @brief Q_SIGNAL that sends the actual stamp time.
     *
     * @param stamp_label QString that contains the time stamp.
     */
    void sendStampLabel(const QString stamp_label);

    /**
     * @brief Q_SIGNAL that sends the actual stamp in a human-readable
     *        date, in a QString.
     *
     * @param human_date QString that contains the time stamp in a
     *        human-redable format.
     */
    void sendDateLabel(const QString human_date);

    /**
     * @brief Q_SIGNAL that sends the playback speed.
     *
     * @param speed QString that contains the actual playback speed.
     */
    void sendPlayspeedLabel(const QString speed);

    /**
     * @brief Q_SIGNAL that sends the number of seconds the current playhead
     *        location is from the beginning of the bag.
     *
     * @param seconds QString that contains the current playhead location
     *        in seconds.
     */
    void sendSecondsLabel(const QString seconds);

    /**
     * @brief Q_SIGNAL that sends a text to notify the user about something.
     *
     * @param status QString that contains the status message to be shown.
     */
    void sendStatusText(const QString status);

    /**
     * @brief Q_SIGNAL that enables or disables the action buttons from the
     *        parent QWidget.
     *
     * @param enable Bool to enable or disable all action buttons.
     */
    void sendEnableActionButtons(const bool enable);

    /**
     * @brief Q_SIGNAL that sends a value [0, 100] to set the progress bar.
     *
     * @param progress Int value with the calculated progress.
     */
    void sendPlayheadProgress(const int progress);

  public Q_SLOTS:
    /**
     * @brief Q_SLOT to receive the absolute file path of the selected
     *        rosbag to be loaded.
     *
     * @param filename QString that contains the absolute file path of
     *        the rosbag.
     */
    void receiveLoadBag(const QString filename);

    /**
     * @brief Q_SLOT to set the time stamp for the beginning of
     *        the bag.
     *
     * @param start ros::Time with the desired time stamp.
     */
    void receiveSetStart(const ros::Time& start);

    /**
     * @brief Q_SLOT to set the time stamp for the end of
     *        the bag.
     *
     * @param end ros::Time with the desired time stamp.
     */
    void receiveSetEnd(const ros::Time& end);

    /**
     * @brief Q_SLOT to change the playback speed.
     *
     * @param change Float with the value to increase or decrease
     *        the playback speed.
     */
    void receiveChangeSpeed(const float change);

    /**
     * @brief Q_SLOT to pause the bag reproduction, if it is being
     *        played during the signal reception.
     */
    void receiveSetPause(void);

    /**
     * @brief Q_SLOT to start playing the loaded rosbag, if it is
     *        not being already played during the signal reception.
     */
    void receiveStartPlaying(void);

    /**
     * @brief Q_SLOT to set the current playhead to the beginning
     *        of the loaded rosbag.
     */
    void receiveGotoBegin(void);

    /**
     * @brief Q_SLOT to set the current playhead to the end
     *        of the loaded rosbag.
     */
    void receiveGotoEnd(void);

    /**
     * @brief Q_SLOT to set the new start ros time samp from the
     * clicked progress bar value
     *
     * @param value Int with the value [0, 100] from the
     *        progress bar.
     */
    void receiveClickedProgress(int value);

  private:
    ros::NodeHandle               _nh;
    rosbag::Bag                   _bag;
    std::unique_ptr<rosbag::View> _full_view;

    std::map<std::string, ros::Publisher> _pubs;
    std::vector<rosbag::View::iterator>   _reverse_bag;
    std::thread                           _play_thread;

    ros::Time _bag_control_start;
    ros::Time _bag_control_end;
    ros::Time _full_bag_start, _full_bag_end;
    ros::Time _last_message_time;
    ros::Time _play_start;

    double _playback_speed{1.0};
    bool   _pause{false};
    bool   _thread_running{false};
    bool   _playback_direction_changed{false};

    std::mutex _playback_mutex;
    std::mutex _pause_mutex;
    std::mutex _thread_mutex;
};

} // namespace rosbag_rviz_panel