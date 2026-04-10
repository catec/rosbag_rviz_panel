#include "rosbag_rviz_panel/BagPlayerWidget.hpp"

#include <QFileDialog>
#include <QIcon>
#include <QListWidgetItem>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>
#include <rclcpp/logger.hpp>

#include "ui_BagPlayerWidget.h"

#define INCREASE_PLAYBACK_SPEED 0.5
#define DECREASE_PLAYBACK_SPEED -0.5

namespace rosbag_rviz_panel {

BagPlayerWidget::BagPlayerWidget(QWidget* parent) : QWidget(parent), _ui(std::make_unique<Ui::BagPlayerWidget>())
{
    _ui->setupUi(this);

    setObjectName("QBagPlayer");

    _progress_bar = std::make_unique<QCustomProgressBar>(this);
    _progress_bar->setRange(0, 100);
    _progress_bar->setEnabled(false);
    _ui->horizontalLayout_2->addWidget(_progress_bar.get());

    _player        = std::make_unique<QBagPlayer>();
    _player_thread = std::make_unique<QThread>(this);
    _player->moveToThread(_player_thread.get());
    connect(_player_thread.get(), &QThread::finished, _player.get(), &QBagPlayer::deleteLater, Qt::QueuedConnection);
    _player_thread->start();

    connectSignals();

    QIcon::setThemeName("Yaru");
    _ui->play_button->setIcon(QIcon::fromTheme("media-playback-start"));
    _ui->begin_button->setIcon(QIcon::fromTheme("media-skip-backward"));
    _ui->end_button->setIcon(QIcon::fromTheme("media-skip-forward"));
    _ui->slower_button->setIcon(QIcon::fromTheme("media-seek-backward"));
    _ui->faster_button->setIcon(QIcon::fromTheme("media-seek-forward"));
    _ui->load_button->setIcon(QIcon::fromTheme("document-open"));

    connect(_ui->play_button, &QPushButton::clicked, this, &BagPlayerWidget::handlePlayClicked);
    connect(_ui->slower_button, &QPushButton::clicked, this, &BagPlayerWidget::handleSlowerClicked);
    connect(_ui->faster_button, &QPushButton::clicked, this, &BagPlayerWidget::handleFasterClicked);
    connect(_ui->load_button, &QPushButton::clicked, this, &BagPlayerWidget::handleLoadClicked);
    connect(_ui->show_topics_button, &QPushButton::clicked, this, &BagPlayerWidget::handleShowTopicsClicked);
    connect(_ui->select_all_topics_button, &QPushButton::clicked, this, &BagPlayerWidget::handleSelectAllTopicsClicked);
    connect(_ui->step_play_button, &QPushButton::clicked, this, &BagPlayerWidget::handleStepPlayClicked);

    _step_play_timer = new QTimer(this);
    _step_play_timer->setSingleShot(true);
    connect(_step_play_timer, &QTimer::timeout, this, &BagPlayerWidget::handleStepPlayTimeout);

    receiveEnableActionButtons(false);
}

BagPlayerWidget::~BagPlayerWidget()
{
    Q_EMIT sendPausePlaying();

    if (_player_thread) {
        _player_thread->quit();
        _player_thread->wait();

        _player_thread.reset();
    }

    if (_progress_bar)
        _progress_bar.reset();
}

void BagPlayerWidget::handlePlayClicked(const bool checked)
{
    if (checked) {
        _ui->play_button->setIcon(QIcon::fromTheme("media-playback-pause"));
        startPlaying();
    } else {
        _ui->play_button->setIcon(QIcon::fromTheme("media-playback-start"));
        stopPlaying();
    }
}

void BagPlayerWidget::handleFasterClicked(void)
{
    Q_EMIT sendFaster(INCREASE_PLAYBACK_SPEED);
}

void BagPlayerWidget::handleSlowerClicked(void)
{
    Q_EMIT sendSlower(DECREASE_PLAYBACK_SPEED);
}

void BagPlayerWidget::handleLoadClicked(void)
{
    const QFileInfo filename = QFileDialog::getOpenFileName(
            this,
            tr("Select the file to load"),
            QDir::homePath(),
            tr("SQLite3 file (*.db3)"),
            nullptr,
            QFileDialog::DontUseNativeDialog);

    if (filename.exists()) {
        try {
            Q_EMIT sendLoadBag(filename.absoluteFilePath());

        } catch (const std::runtime_error& e) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("global_logger"), e.what());

            receiveStatusText(QString::fromStdString(e.what()));
            receiveEnableActionButtons(false);
        }
    } else {
        std::string msg = "File: '" + filename.absoluteFilePath().toStdString() + "' does not exists!";
        RCLCPP_WARN_STREAM(rclcpp::get_logger("global_logger"), msg);
    }
}

void BagPlayerWidget::receiveFileSizeLabel(const QString size)
{
    if (!size.isEmpty()) {
        _ui->filesize_label->setText(size);
    } else
        _ui->filesize_label->clear();
}

void BagPlayerWidget::receiveStatusText(const QString text)
{
    if (!text.isEmpty()) {
        _ui->status_bar->setFormat(text);
        _ui->status_bar->setTextVisible(true);
    } else
        _ui->status_bar->setTextVisible(false);
}

void BagPlayerWidget::receiveStampLabel(const QString stamp)
{
    if (!stamp.isEmpty())
        _ui->stamp_label->setText(stamp + "s");
    else
        _ui->stamp_label->clear();
}

void BagPlayerWidget::receiveDateLabel(const QString date)
{
    if (!date.isEmpty())
        _ui->date_label->setText(date);
    else
        _ui->date_label->clear();
}

void BagPlayerWidget::receivePlayspeedLabel(const QString speed)
{
    if (!speed.isEmpty())
        _ui->playspeed_label->setText(speed);
    else
        _ui->playspeed_label->clear();
}

void BagPlayerWidget::receiveSecondsLabel(const QString seconds)
{
    if (!seconds.isEmpty())
        _ui->seconds_label->setText(seconds);
    else
        _ui->seconds_label->clear();
}

void BagPlayerWidget::receiveEnableActionButtons(const bool enable)
{
    QList<QPushButton*> actionButtons = this->findChildren<QPushButton*>();
    for (const auto& btn : actionButtons) {
        if (btn != _ui->load_button)
            btn->setEnabled(enable);
    }

    _progress_bar->setEnabled(enable);
    _ui->show_topics_button->setEnabled(enable);
    _ui->select_all_topics_button->setEnabled(enable);
    _ui->step_play_button->setEnabled(enable);
}

void BagPlayerWidget::receiveBagFinished(void)
{
    _ui->play_button->setIcon(QIcon::fromTheme("media-playback-start"));
    if (_ui->play_button->isChecked())
        _ui->play_button->click();
}

void BagPlayerWidget::receiveTopicList(const QStringList topics)
{
    _ui->topic_list_widget->clear();
    for (const auto& topic : topics) {
        QListWidgetItem* item = new QListWidgetItem(topic, _ui->topic_list_widget);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Checked);
    }
    _all_topics_selected = true;
}

void BagPlayerWidget::receiveUnsupportedTopicList(const QStringList topics)
{
    for (int i = 0; i < _ui->topic_list_widget->count(); ++i) {
        QListWidgetItem* item = _ui->topic_list_widget->item(i);
        // Extract base topic name (strip any existing suffix)
        QString base = item->data(Qt::UserRole).toString();
        if (base.isEmpty())
            base = item->text();
        if (topics.contains(base)) {
            item->setText(base + "  [unsupported]");
            item->setData(Qt::UserRole, base);  // store original name
            item->setFlags(item->flags() & ~Qt::ItemIsEnabled);  // gray out
            item->setCheckState(Qt::Unchecked);
        }
    }
    // Update selected topics after marking unsupported ones
    updateSelectedTopics();
}

void BagPlayerWidget::handleShowTopicsClicked(void)
{
    _topics_visible = !_topics_visible;
    _ui->topic_list_widget->setVisible(_topics_visible);
    _ui->show_topics_button->setText(_topics_visible ? "Hide Topics" : "Show Topics");
}

void BagPlayerWidget::handleSelectAllTopicsClicked(void)
{
    _all_topics_selected = !_all_topics_selected;
    Qt::CheckState state = _all_topics_selected ? Qt::Checked : Qt::Unchecked;
    for (int i = 0; i < _ui->topic_list_widget->count(); ++i) {
        _ui->topic_list_widget->item(i)->setCheckState(state);
    }
    _ui->select_all_topics_button->setText(_all_topics_selected ? "Deselect All" : "Select All");
    updateSelectedTopics();
}

void BagPlayerWidget::handleStepPlayClicked(void)
{
    double duration_sec = _ui->step_duration_spin->value();

    // Start playing
    if (!_ui->play_button->isChecked())
        _ui->play_button->click();

    // Set timer to pause after duration
    _step_play_timer->start(static_cast<int>(duration_sec * 1000));
}

void BagPlayerWidget::handleStepPlayTimeout(void)
{
    // Pause playback
    if (_ui->play_button->isChecked())
        _ui->play_button->click();
}

void BagPlayerWidget::updateSelectedTopics(void)
{
    QStringList selected;
    for (int i = 0; i < _ui->topic_list_widget->count(); ++i) {
        QListWidgetItem* item = _ui->topic_list_widget->item(i);
        if (item->checkState() == Qt::Checked) {
            // Use stored original name if available (unsupported topics store it in UserRole)
            QString name = item->data(Qt::UserRole).toString();
            if (name.isEmpty())
                name = item->text();
            selected.append(name);
        }
    }
    Q_EMIT sendSelectedTopics(selected);
}

void BagPlayerWidget::startPlaying(void)
{
    Q_EMIT sendStartPlaying();
}

void BagPlayerWidget::stopPlaying(void)
{
    Q_EMIT sendPausePlaying();
}

void BagPlayerWidget::connectSignals(void)
{
    connect(this, &BagPlayerWidget::sendLoadBag, _player.get(), &QBagPlayer::receiveLoadBag, Qt::QueuedConnection);
    connect(this,
            &BagPlayerWidget::sendStartPlaying,
            _player.get(),
            &QBagPlayer::receiveStartPlaying,
            Qt::QueuedConnection);
    connect(this,
            &BagPlayerWidget::sendPausePlaying,
            _player.get(),
            &QBagPlayer::receiveSetPause,
            Qt::QueuedConnection);
    connect(this, &BagPlayerWidget::sendSetStart, _player.get(), &QBagPlayer::receiveSetStart, Qt::QueuedConnection);
    connect(this, &BagPlayerWidget::sendSetEnd, _player.get(), &QBagPlayer::receiveSetEnd, Qt::QueuedConnection);
    connect(this, &BagPlayerWidget::sendFaster, _player.get(), &QBagPlayer::receiveChangeSpeed, Qt::QueuedConnection);
    connect(this, &BagPlayerWidget::sendSlower, _player.get(), &QBagPlayer::receiveChangeSpeed, Qt::QueuedConnection);

    connect(_ui->end_button, &QPushButton::clicked, _player.get(), &QBagPlayer::receiveGotoEnd, Qt::QueuedConnection);
    connect(_ui->begin_button,
            &QPushButton::clicked,
            _player.get(),
            &QBagPlayer::receiveGotoBegin,
            Qt::QueuedConnection);
    connect(_progress_bar.get(),
            &QCustomProgressBar::sendClickedProgress,
            _player.get(),
            &QBagPlayer::receiveClickedProgress);

    connect(_player.get(),
            &QBagPlayer::sendBagFinished,
            this,
            &BagPlayerWidget::receiveBagFinished,
            Qt::QueuedConnection);
    connect(_player.get(),
            &QBagPlayer::sendBagSize,
            this,
            &BagPlayerWidget::receiveFileSizeLabel,
            Qt::QueuedConnection);
    connect(_player.get(),
            &QBagPlayer::sendStampLabel,
            this,
            &BagPlayerWidget::receiveStampLabel,
            Qt::QueuedConnection);
    connect(_player.get(), &QBagPlayer::sendDateLabel, this, &BagPlayerWidget::receiveDateLabel, Qt::QueuedConnection);
    connect(_player.get(),
            &QBagPlayer::sendPlayspeedLabel,
            this,
            &BagPlayerWidget::receivePlayspeedLabel,
            Qt::QueuedConnection);
    connect(_player.get(),
            &QBagPlayer::sendSecondsLabel,
            this,
            &BagPlayerWidget::receiveSecondsLabel,
            Qt::QueuedConnection);
    connect(_player.get(),
            &QBagPlayer::sendStatusText,
            this,
            &BagPlayerWidget::receiveStatusText,
            Qt::QueuedConnection);
    connect(_player.get(),
            &QBagPlayer::sendEnableActionButtons,
            this,
            &BagPlayerWidget::receiveEnableActionButtons,
            Qt::QueuedConnection);
    connect(_player.get(),
            &QBagPlayer::sendPlayheadProgress,
            _progress_bar.get(),
            &QCustomProgressBar::setValue,
            Qt::QueuedConnection);

    connect(_player.get(),
            &QBagPlayer::sendTopicList,
            this,
            &BagPlayerWidget::receiveTopicList,
            Qt::QueuedConnection);
    connect(_player.get(),
            &QBagPlayer::sendUnsupportedTopicList,
            this,
            &BagPlayerWidget::receiveUnsupportedTopicList,
            Qt::QueuedConnection);
    connect(this,
            &BagPlayerWidget::sendSelectedTopics,
            _player.get(),
            &QBagPlayer::receiveSelectedTopics,
            Qt::QueuedConnection);

    // When topic checkboxes change, update selected topics
    connect(_ui->topic_list_widget, &QListWidget::itemChanged, this, [this](QListWidgetItem*) {
        updateSelectedTopics();
    });
}

} // namespace rosbag_rviz_panel