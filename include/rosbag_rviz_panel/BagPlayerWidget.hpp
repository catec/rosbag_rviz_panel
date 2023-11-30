#pragma once

#include <QList>
#include <QThread>
#include <QWidget>

#include "QBagPlayer.hpp"
#include "QCustomProgressBar.hpp"

namespace Ui {
class BagPlayerWidget;
}

namespace rosbag_rviz_panel {

/**
 * @brief BagPlayerWidget.
 *
 * This custom QWidget controls the Qt-based user interface,
 * connecting all signals and slots.
 *
 */
class BagPlayerWidget : public QWidget
{
    Q_OBJECT

  public:
    /**
     * @brief Constructor of the BagPlayerWidget class.
     *
     * @param parent A parent QWidget, if there is one.
     */
    explicit BagPlayerWidget(QWidget* parent = nullptr);

    /**
     * @brief Destructor of the BagPlayerWidget class.
     */
    virtual ~BagPlayerWidget();

  private:
    /**
     * @brief Function that emits the signal to start
     * playing the loaded rosbag.
     */
    void startPlaying(void);

    /**
     * @brief Function that emits the signal to stop
     * playing the loaded rosbag.
     */
    void stopPlaying(void);

    /**
     * @brief Function to connect all the necessary
     * signals and slots between this class and
     * the QBagPlayer class.
     */
    void connectSignals(void);

  Q_SIGNALS:
    /**
     * @brief Q_SIGNAL that sends the absolute file path
     * of the rosbag.
     *
     * @param file QString that contains the bag path.
     */
    void sendLoadBag(const QString file);

    /**
     * @brief Q_SIGNAL that starts the rosbag playing.
     */
    void sendStartPlaying(void);

    /**
     * @brief Q_SIGNAL that stops the rosbag playing.
     */
    void sendPausePlaying(void);

    /**
     * @brief Q_SIGNAL that sends time stamp from where
     * to start publishing messages.
     *
     * @param start int64_t with the desired time stamp in nanoseconds.
     */
    void sendSetStart(const int64_t start);

    /**
     * @brief Q_SIGNAL that sends time stamp from where
     * to stop publishing messages.
     *
     * @param end int64_t with the desired time stamp in nanoseconds.
     */
    void sendSetEnd(const int64_t end);

    /**
     * @brief Q_SIGNAL that increases the playback speed.
     *
     * @param value Float with the desired value to
     *        increase the playback speed.
     */
    void sendFaster(const float value);

    /**
     * @brief Q_SIGNAL that decreases the playback speed.
     *
     * @param value Float with the desired value to
     *        decrease the playback speed.
     */
    void sendSlower(const float value);

  private Q_SLOTS:
    /**
     * @brief Q_SLOT that handles actions for when
     * the play button has been clicked.
     *
     * @param checked Bool set to true if the button was
     *        pressed, and false if it was released.
     */
    void handlePlayClicked(const bool checked);

    /**
     * @brief Q_SIGNAL that handles actions for when
     * the faster button has been clicked.
     */
    void handleFasterClicked(void);

    /**
     * @brief Q_SIGNAL that handles actions for when
     * the slower button has been clicked.
     */
    void handleSlowerClicked(void);

    /**
     * @brief Q_SIGNAL that handles actions for when
     * the load button has been clicked.
     */
    void handleLoadClicked(void);

    /**
     * @brief Q_SLOT that gets the total size of the bag.
     *
     * @param size QString that contains the total bag size.
     */
    void receiveFileSizeLabel(const QString size);

    /**
     * @brief Q_SLOT that receives a text to notify the
     * user about something.
     *
     * @param text QString that contains the status message to be shown.
     */
    void receiveStatusText(const QString text);

    /**
     * @brief Q_SLOT that receives the actual stamp time.
     *
     * @param stamp QString that contains the time stamp.
     */
    void receiveStampLabel(const QString stamp);

    /**
     * @brief Q_SLOT that gets the actual stamp in a human-readable
     * date, in a QString.
     *
     * @param date QString that contains the time stamp in a
     *        human-redable format.
     */
    void receiveDateLabel(const QString date);

    /**
     * @brief Q_SLOT that gets the playback speed.
     *
     * @param speed QString that contains the actual playback speed.
     */
    void receivePlayspeedLabel(const QString speed);

    /**
     * @brief Q_SLOT that receives the number of seconds the current
     * playhead location is from the beginning of the bag.
     *
     * @param seconds QString that contains the current playhead location
     *        in seconds.
     */
    void receiveSecondsLabel(const QString seconds);

    /**
     * @brief Q_SLOT that enables or disables the action buttons.
     *
     * @param enable Bool to enable or disable all action buttons.
     */
    void receiveEnableActionButtons(const bool enable);

    /**
     * @brief Q_SLOT to reset the play button.
     */
    void receiveBagFinished(void);

  private:
    std::unique_ptr<Ui::BagPlayerWidget> _ui;

    std::unique_ptr<QBagPlayer>         _player;
    std::unique_ptr<QThread>            _player_thread;
    std::unique_ptr<QCustomProgressBar> _progress_bar;
};
} // namespace rosbag_rviz_panel