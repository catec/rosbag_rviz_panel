#pragma once

#include <QMouseEvent>
#include <QProgressBar>
#include <QWidget>

namespace rosbag_rviz_panel {

/**
 * @brief QCustomProgressBar.
 *
 * This custom QWidget inherits from a regular QProgressBar,
 * with additional functionalities such as changing its value
 * when the bar is clicked.
 *
 */
class QCustomProgressBar : public QProgressBar
{
    Q_OBJECT

  public:
    /**
     * @brief Constructor of the QCustomProgressBar class.
     *
     * @param parent A parent QWidget, if there is one.
     */
    explicit QCustomProgressBar(QWidget* parent = nullptr);

    /**
     * @brief Destructor of the QCustomProgressBar class.
     */
    virtual ~QCustomProgressBar();

  Q_SIGNALS:
    /**
     * @brief Q_SIGNAL to notify that the progress bar
     * has been clicked, in order to update the bag time stamp.
     */
    void sendClickedProgress(int val);

  protected:
    /**
     * @brief Overrided method from the parent class to add the
     * mouse interaction logic.
     */
    void mousePressEvent(QMouseEvent* event) override;
};
} // namespace rosbag_rviz_panel