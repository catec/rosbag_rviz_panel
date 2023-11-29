#include "rosbag_rviz_panel/QCustomProgressBar.hpp"

#include <QApplication>

namespace rosbag_rviz_panel {

QCustomProgressBar::QCustomProgressBar(QWidget* parent) : QProgressBar(parent) {}

QCustomProgressBar::~QCustomProgressBar() {}

void QCustomProgressBar::mousePressEvent(QMouseEvent* event)
{
    event->ignore();
    if ((windowType() == Qt::Popup)) {
        event->accept();
        QWidget* w;
        while ((w = QApplication::activePopupWidget()) && w != this) {
            w->close();
            if (QApplication::activePopupWidget() == w) // widget does not want to disappear
                w->hide();                              // hide at least
        }
        if (!rect().contains(event->pos())) {
            close();
        }
    }

    int newValue = minimum() + ((maximum() - minimum()) * event->x()) / width();
    setValue(newValue);

    Q_EMIT sendClickedProgress(newValue);
}
} // namespace rosbag_rviz_panel