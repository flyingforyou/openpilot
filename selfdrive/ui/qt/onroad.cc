#include "selfdrive/ui/qt/onroad.h"

#include <cmath>

#include <QDebug>
#include <QSound>
#include <QMouseEvent>

#include "common/timing.h"
#include "selfdrive/ui/qt/util.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#endif

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(bdr_s);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  QStackedLayout *road_view_layout = new QStackedLayout;
  road_view_layout->setStackingMode(QStackedLayout::StackAll);
  nvg = new AnnotatedCameraWidget(VISION_STREAM_ROAD, this);
  road_view_layout->addWidget(nvg);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addLayout(road_view_layout);

  if (getenv("DUAL_CAMERA_VIEW")) {
    CameraWidget *arCam = new CameraWidget("camerad", VISION_STREAM_ROAD, true, this);
    split->insertWidget(0, arCam);
  }

  if (getenv("MAP_RENDER_VIEW")) {
    CameraWidget *map_render = new CameraWidget("navd", VISION_STREAM_MAP, false, this);
    split->insertWidget(0, map_render);
  }

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);

  // screen recoder - neokii

  record_timer = std::make_shared<QTimer>();
	QObject::connect(record_timer.get(), &QTimer::timeout, [=]() {
    if(recorder) {
      recorder->update_screen();
    }
  });
	record_timer->start(1000/UI_FREQ);

  QWidget* recorder_widget = new QWidget(this);
  QVBoxLayout * recorder_layout = new QVBoxLayout (recorder_widget);
  recorder_layout->setMargin(35);
  recorder = new ScreenRecoder(this);
  recorder_layout->addWidget(recorder);
  recorder_layout->setAlignment(recorder, Qt::AlignRight | Qt::AlignBottom);

  stacked_layout->addWidget(recorder_widget);
  recorder_widget->raise();
  alerts->raise();

}

void OnroadWindow::updateState(const UIState &s) {
  QColor bgColor = bg_colors[s.status];
  Alert alert = Alert::get(*(s.sm), s.scene.started_frame);
  if (s.sm->updated("controlsState") || !alert.equal({})) {
    if (alert.type == "controlsUnresponsive") {
      bgColor = bg_colors[STATUS_ALERT];
    } else if (alert.type == "controlsUnresponsivePermanent") {
      bgColor = bg_colors[STATUS_DISENGAGED];
    }
    alerts->updateAlert(alert, bgColor);
  }

  if (s.scene.map_on_left) {
    split->setDirection(QBoxLayout::LeftToRight);
  } else {
    split->setDirection(QBoxLayout::RightToLeft);
  }

  nvg->updateState(s);

  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
}

void OnroadWindow::mouseReleaseEvent(QMouseEvent* e) {

  QPoint endPos = e->pos();
  int dx = endPos.x() - startPos.x();
  int dy = endPos.y() - startPos.y();
  if(std::abs(dx) > 250 || std::abs(dy) > 200) {

    if(std::abs(dx) < std::abs(dy)) {

      if(dy < 0) { // upward
        Params().remove("CalibrationParams");
        Params().remove("LiveParameters");
        QTimer::singleShot(1500, []() {
          Params().putBool("SoftRestartTriggered", true);
        });

        QSound::play("../assets/sounds/reset_calibration.wav");
      }
      else { // downward
        QTimer::singleShot(500, []() {
          Params().putBool("SoftRestartTriggered", true);
        });
      }
    }
    else if(std::abs(dx) > std::abs(dy)) {
      if(dx < 0) { // right to left
        if(recorder)
          recorder->toggle();
      }
      else { // left to right
        if(recorder)
          recorder->toggle();
      }
    }

    return;
  }

  QRect gapRect(302-10, 773-10, 192+20, 192+20);
  QRect opRect(rect().right() - 300, 0, 300, 300);
  const SubMaster& sm = *(uiState()->sm);
  if (gapRect.contains(e->x(), e->y())) {
      const auto cs = sm["controlsState"].getControlsState();
      int myDrivingMode = cs.getMyDrivingMode();
      myDrivingMode++;
      if (myDrivingMode > 4) myDrivingMode = 1;
      QString values = QString::number(myDrivingMode);
      Params().put("MyDrivingMode", values.toStdString());
      return;
  }
  else if (opRect.contains(e->x(), e->y())) {
      const auto cs = sm["controlsState"].getControlsState();
      int longActiveUser = cs.getLongActiveUser();
      if (longActiveUser <= 0) {
          if (Params().getBool("ExperimentalMode")) Params().put("ExperimentalMode", "0");
          else Params().put("ExperimentalMode", "1");
      }
      return;
  }
  else if (map != nullptr) {
    bool sidebarVisible = geometry().x() > 0;
    map->setVisible(!sidebarVisible && !map->isVisible());
  }
  // propagation event to parent(HomeWindow)
  QWidget::mouseReleaseEvent(e);
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  startPos = e->pos();
  //QWidget::mousePressEvent(e);
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    if (map == nullptr && (uiState()->prime_type || !MAPBOX_TOKEN.isEmpty())) {
      MapWindow * m = new MapWindow(get_mapbox_settings());
      map = m;

      QObject::connect(uiState(), &UIState::offroadTransition, m, &MapWindow::offroadTransition);

      m->setFixedWidth(topWidget(this)->width() / 2);
      split->insertWidget(0, m);

      // Make map visible after adding to split
      m->offroadTransition(offroad);
    }
  }
#endif

  alerts->updateAlert({}, bg);

  if(offroad && recorder) {
    recorder->stop(false);
  }
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
}

// ***** onroad widgets *****

// OnroadAlerts
void OnroadAlerts::updateAlert(const Alert &a, const QColor &color) {
  if (!alert.equal(a) || color != bg) {
    alert = a;
    bg = color;
    update();
  }
}

void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::ControlsState::AlertSize::NONE) {
    return;
  }
  static std::map<cereal::ControlsState::AlertSize, const int> alert_sizes = {
    {cereal::ControlsState::AlertSize::SMALL, 271},
    {cereal::ControlsState::AlertSize::MID, 420},
    {cereal::ControlsState::AlertSize::FULL, height()},
  };
  int h = alert_sizes[alert.size];
  QRect r = QRect(0, height() - h, width(), h);

  QPainter p(this);

  // draw background + gradient
  p.setPen(Qt::NoPen);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  bg.setAlpha(100);
  p.setBrush(QBrush(bg));
  p.drawRect(r);

  QLinearGradient g(0, r.y(), 0, r.bottom());
  g.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.05));
  g.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0.35));

  p.setCompositionMode(QPainter::CompositionMode_DestinationOver);
  p.setBrush(QBrush(g));
  p.fillRect(r, g);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  // text
  const QPoint c = r.center();
  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setRenderHint(QPainter::TextAntialiasing);
  if (alert.size == cereal::ControlsState::AlertSize::SMALL) {
    configFont(p, "Inter", 74, "SemiBold");
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::ControlsState::AlertSize::MID) {
    configFont(p, "Inter", 88, "Bold");
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    configFont(p, "Inter", 66, "Regular");
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    configFont(p, "Inter", l ? 132 : 177, "Bold");
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    configFont(p, "Inter", 88, "Regular");
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}

ExperimentalButton::ExperimentalButton(QWidget *parent) : QPushButton(parent) {
  setVisible(false);
  setFixedSize(btn_size, btn_size);
  setCheckable(true);

  params = Params();
  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size, img_size});

  QObject::connect(this, &QPushButton::toggled, [=](bool checked) {
    params.putBool("ExperimentalMode", checked);
  });
}

void ExperimentalButton::updateState(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  // button is "visible" if engageable or enabled
  const auto cs = sm["controlsState"].getControlsState();
  setVisible(cs.getEngageable() || cs.getEnabled());

  // button is "checked" if experimental mode is enabled
  setChecked(sm["controlsState"].getControlsState().getExperimentalMode());

  // disable button when experimental mode is not available, or has not been confirmed for the first time
  const auto cp = sm["carParams"].getCarParams();
  const bool experimental_mode_available = cp.getExperimentalLongitudinalAvailable() ? params.getBool("ExperimentalLongitudinalEnabled") : cp.getOpenpilotLongitudinalControl();
  setEnabled(params.getBool("ExperimentalModeConfirmed") && experimental_mode_available);
}

void ExperimentalButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  QPoint center(btn_size / 2, btn_size / 2);
  QPixmap img = isChecked() ? experimental_img : engage_img;

  p.setOpacity(1.0);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 166));
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.setOpacity(isDown() ? 0.8 : 1.0);
  p.drawPixmap((btn_size - img_size) / 2, (btn_size - img_size) / 2, img);
}

AnnotatedCameraWidget::AnnotatedCameraWidget(VisionStreamType type, QWidget* parent) : last_update_params(0), fps_filter(UI_FREQ, 3, 1. / UI_FREQ), accel_filter(UI_FREQ, .5, 1. / UI_FREQ), CameraWidget("camerad", type, true, parent) {
  //engage_img = loadPixmap("../assets/img_chffr_wheel.png", { img_size, img_size });
  engage_img = loadPixmap("../assets/images/handle1.png", { img_size, img_size });
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size - 5, img_size - 5});

  dm_img = loadPixmap("../assets/img_driver_face.png", {img_size + 5, img_size + 5});

  // neokii
  ic_brake = QPixmap("../assets/images/img_brake_disc.png").scaled(img_size, img_size, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
  ic_autohold_warning = QPixmap("../assets/images/img_autohold_warning.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  ic_autohold_active = QPixmap("../assets/images/img_autohold_active.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  ic_nda = QPixmap("../assets/images/img_nda.png");
  ic_hda = QPixmap("../assets/images/img_hda.png");
  ic_tire_pressure = QPixmap("../assets/images/img_tire_pressure.png");
  ic_turn_signal_l = QPixmap("../assets/images/turn_signal_l.png");
  ic_turn_signal_r = QPixmap("../assets/images/turn_signal_r.png");
  ic_satellite = QPixmap("../assets/images/satellite.png");

  ic_trafficLight_green = QPixmap("../assets/images/traffic_green.png");
  ic_trafficLight_red = QPixmap("../assets/images/traffic_red.png");
  ic_trafficLight_x = QPixmap("../assets/images/traffic_x.png");
  ic_trafficLight_none = QPixmap("../assets/images/traffic_none.png");
  ic_stopman = QPixmap("../assets/images/stopman.png");
  ic_navi = QPixmap("../assets/images/img_navi.png");
  ic_scc2 = QPixmap("../assets/images/img_scc2.png");
  ic_radartracks = QPixmap("../assets/images/img_radartracks.png");
  ic_radar = QPixmap("../assets/images/radar.png");
  ic_radar_vision = QPixmap("../assets/images/radar_vision.png");
  ic_radar_no = QPixmap("../assets/images/no_radar.png");
}

void AnnotatedCameraWidget::initializeGL() {
    CameraWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);
  
}

void AnnotatedCameraWidget::updateState(const UIState &s) {
  const SubMaster &sm = *(s.sm);
  const bool cs_alive = sm.alive("controlsState");
 // TODO: Add minimum speed?
  setProperty("left_blindspot", cs_alive && sm["carState"].getCarState().getLeftBlindspot());
  setProperty("right_blindspot", cs_alive && sm["carState"].getCarState().getRightBlindspot());
  const auto cs = sm["controlsState"].getControlsState();

  // update DM icons at 2Hz
  if (sm.frame % (UI_FREQ / 2) == 0) {
    dmActive = sm["driverMonitoringState"].getDriverMonitoringState().getIsActiveMode();
  }

  hideDM = (cs.getAlertSize() != cereal::ControlsState::AlertSize::NONE);
  dm_fade_state = fmax(0.0, fmin(1.0, dm_fade_state+0.2*(0.5-(float)(dmActive))));
}

void AnnotatedCameraWidget::updateFrameMat() {
    CameraWidget::updateFrameMat();
  UIState *s = uiState();
  int w = width(), h = height();

  s->fb_w = w;
  s->fb_h = h;

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2 - x_offset, h / 2 - y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

float lerp(float a, float b, float t) {
  return a + t * (b - a);
}

float interp1d(float value, float start_min, float start_max, float end_min, float end_max) {
  value = std::max(start_min, std::min(start_max, value));
  float factor = (value - start_min) / (start_max - start_min);
  return end_min + factor * (end_max - end_min);
}

void AnnotatedCameraWidget::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();

  const UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);

  // lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    float prob = std::clamp<float>(scene.lane_line_probs[i]*2.0, 0.5, 1.0);
    painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, prob));
    //painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    painter.drawPolygon(scene.lane_line_vertices[i]);
  }
	
  // TODO: Fix empty spaces when curiving back on itself
  painter.setBrush(QColor(255, 215, 000, 150));
  if (left_blindspot) painter.drawPolygon(scene.lane_barrier_vertices[0]);
  if (right_blindspot) painter.drawPolygon(scene.lane_barrier_vertices[1]);

  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
      float prob = std::clamp<float>((2.0 - scene.road_edge_stds[i])*2.0, 0.5, 1.0);
      painter.setBrush(QColor::fromRgbF(1.0, 0, 1.0, prob));
      //painter.setBrush(QColor::fromRgbF(1.0, 0, 1.0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));
      painter.drawPolygon(scene.road_edge_vertices[i]);
  }

  //const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
  //auto xState = lp.getXState();
  //bool x = (xState == cereal::LongitudinalPlan::XState::E2E_STOP);// || (xState == cereal::LongitudinalPlan::XState::SOFT_HOLD);

  // paint path
//  QLinearGradient bg(0, height(), 0, height() / 4);
  QLinearGradient bg(0, height(), 0, 0);
  float start_hue, end_hue;
  if (sm["controlsState"].getControlsState().getExperimentalMode() || true) {

    int track_vertices_len = scene.track_vertices.length();
    assert(track_vertices_len % 2 == 0);
    QVector<QPointF> right_points = scene.track_vertices.mid(0, track_vertices_len / 2);
    qDebug() << right_points.length();
    float max_gradient_point = 1.0;
    if (right_points.length() > 0) {
//      bg.setFinalStop(right_points[right_points.length() - 1]);
      max_gradient_point = (height() - right_points[right_points.length() - 1].y()) / height();
      qDebug() << "max_gradient_point:" << max_gradient_point;
    }
//    float gradient_height = bg.finalStop().y();
    for (int i = 0; i < right_points.length(); i++) {
      const auto &acceleration = sm["uiPlan"].getUiPlan().getAccel();
      float acceleration_future = 0;
      if (i >= acceleration.size()) {
        break;
      }
      acceleration_future = acceleration[i];
      qDebug() << "Using acceleration:" << acceleration_future;

      // need to flip so 0 is bottom of frame (not really, can also flip linear gradient above)
      float lin_grad_point = (height() - right_points[i].y()) / height();
      qDebug() << right_points[i] << right_points[i].y() << lin_grad_point;
      // Some points are out of frame
      // TODO: tho maybe it makes sense to clip instead, so gradient is correct. or no clip/skip at all
      if (lin_grad_point < 0) {
        continue;
      }

      start_hue = 60;
      // speed up: 120, slow down: 0
      end_hue = fmax(fmin(start_hue + acceleration_future * 35, 148), 0);

      float saturation = std::abs(acceleration_future * 1.5);
      saturation = saturation > 1 ? 1. : saturation;
      float lightness = lerp(0.95, 0.62, saturation);
//      lightness = lerp(0.56, 0.88, lin_grad_point);
//      float alpha_lerp = (lin_grad_point - 0.5) * 2;  // ramp alpha down from 0.4 when point reached 0.5
//      float alpha = lerp(0.4, 0, alpha_lerp > 0 ? alpha_lerp : 0);
//      float alpha = interp1d(lin_grad_point, max_gradient_point / 2., max_gradient_point, 0.4, 0.0);  // looks cool, but fades off too early
      float alpha = interp1d(lin_grad_point, 0.375, 0.625, 0.4, 0.0);  // matches behavior before for alpha fade
      qDebug() << "saturation:" << saturation << "lightness:" << lightness << "alpha:" << alpha;

      // FIXME: painter.drawPolygon can be slow if hue is not rounded
      end_hue = int(end_hue * 100 + 0.5) / 100;
//      bg.setColorAt(lin_grad_point, QColor::fromHslF(end_hue / 360., 0.97, 0.56, 0.4));
      bg.setColorAt(lin_grad_point, QColor::fromHslF(end_hue / 360., saturation, lightness, alpha));

    }
//    qDebug() << right_points;

//    const auto &acceleration = sm["modelV2"].getModelV2().getAcceleration();
//    float acceleration_future = 0;
//    if (acceleration.getZ().size() > 16) {
//      acceleration_future = acceleration.getX()[16];  // 2.5 seconds
//    }
//    start_hue = 60;
//    // speed up: 120, slow down: 0
//    end_hue = fmax(fmin(start_hue + acceleration_future * 45, 148), 0);
//
//    // FIXME: painter.drawPolygon can be slow if hue is not rounded
//    end_hue = int(end_hue * 100 + 0.5) / 100;
//
//    bg.setColorAt(0.0, QColor::fromHslF(start_hue / 360., 0.97, 0.56, 0.4));
//    bg.setColorAt(0.5, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.35));
//    bg.setColorAt(1.0, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.0));
  } else {
    bg.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 0.4));
    bg.setColorAt(0.5, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.0));
  }

  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices);

  painter.restore();
}

void AnnotatedCameraWidget::drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd, bool is_radar, bool no_radar/*=false*/) {

    UIState* s = uiState();
    SubMaster& sm = *(s->sm);

    if (!sm.updated("controlsState") || !sm.updated("carControl") || !sm.updated("carState")) return;

    auto lead_radar = sm["radarState"].getRadarState().getLeadOne();
    auto lead_one = sm["modelV2"].getModelV2().getLeadsV3()[0];
    const auto controls_state = sm["controlsState"].getControlsState();
    auto car_control = sm["carControl"].getCarControl();
    auto car_state = sm["carState"].getCarState();
    int longActiveUser = controls_state.getLongActiveUser();


    painter.save();
    const UIScene& scene = s->scene;
    int track_vertices_len = scene.track_vertices.length();
    float path_x = width() / 2;
    float path_y = height() - 200;
    float path_width = 160;
    if (track_vertices_len >= 10) {
        //float right_y = scene.track_vertices[track_vertices_len / 2 - 1].y();
        //float right_x = scene.track_vertices[track_vertices_len / 2 - 1].x();
        //float left_y = scene.track_vertices[track_vertices_len / 2].y();
        //float left_x = scene.track_vertices[track_vertices_len / 2].x();

        path_width = scene.track_vertices[track_vertices_len / 2].x() - scene.track_vertices[track_vertices_len / 2 - 1].x();
        path_x = (scene.track_vertices[track_vertices_len / 2].x() + scene.track_vertices[track_vertices_len / 2 - 1].x()) / 2.;
        path_y = scene.track_vertices[track_vertices_len / 2].y();

        QRect rectPath(path_x - path_width / 2., path_y - 5, path_width, 5);
        QRect rectPathL(path_x - path_width / 2., path_y - 5, 10, 10);
        QRect rectPathR(path_x + path_width / 2. - 5, path_y - 5, 10, 10);
        painter.setPen(Qt::NoPen);
        painter.setBrush(redColor(160));
        painter.drawRect(rectPath);
        painter.drawRect(rectPathL);
        painter.drawRect(rectPathR);


        //if (path_y < height() - 200) path_y = height() - 200;
    }


    //const float speedBuff = 10.;
  //const float leadBuff = 40.;
  const float d_rel = lead_data.getX()[0];
  //const float v_rel = lead_data.getV()[0];

  /*
  float fillAlpha = 0;
  if (d_rel < leadBuff) {
    fillAlpha = 255 * (1.0 - (d_rel / leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255 * (-1 * (v_rel / speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }
  */

  //float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  //float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  float x = std::clamp((float)vd.x(), 220.f, width() - 220.f);
  //float y = std::fmin(height() - sz * .6, (float)vd.y());
  float y = std::clamp((float)vd.y(), 300.f, height() - 180.f);

  y -= ((256/2)-d_rel);  // 과녁 위로~

  if (no_radar) {
      x = path_x;
      y = path_y; // height() - 250;
  }
  if (y > height() - 400) y = height() - 400;

  //float g_xo = sz / 5;
  //float g_yo = sz / 10;

  //QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};
  //painter.setBrush(is_radar ? QColor(86, 121, 216, 255) : QColor(218, 202, 37, 255));
  //painter.drawPolygon(glow, std::size(glow));

  // chevron
  //QPointF chevron[] = {{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}};
  //painter.setBrush(redColor(fillAlpha));
  //painter.drawPolygon(chevron, std::size(chevron));

  auto hud_control = car_control.getHudControl();
  bool radar_detected = lead_radar.getStatus() && lead_radar.getRadar();
  float radar_dist = radar_detected ? lead_radar.getDRel() : 0;
  float vision_dist = lead_one.getProb() > .5 ? (lead_one.getX()[0] - 0) : 0;
  float disp_dist = (radar_detected) ? radar_dist : vision_dist;
  int brake_hold = car_state.getBrakeHoldActive();
  int soft_hold = (hud_control.getSoftHold()) ? 1 : 0;
  bool brake_valid = car_state.getBrakeLights();


  int circle_size = 160;
  painter.setOpacity(1.0);
  painter.setPen(Qt::NoPen);
  QColor bgColor = QColor(0, 0, 0, 166);
  const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
  float stop_dist = 0;
  bool stopping = false;
  if (lp.getTrafficState() >= 100) bgColor = yellowColor(120);
  else {
      switch (lp.getTrafficState() % 100) {
      case 0: bgColor = blackColor(20); break;
      case 1: bgColor = redColor(160);
          stop_dist = lp.getXStop();
          stopping = true;
          //painter.drawPixmap(400, 400, 350, 350, ic_stopman);
          break;
      case 2: bgColor = greenColor(160); break;
      case 3: bgColor = yellowColor(160); break;
      }
  }
  painter.setBrush(bgColor);

  painter.drawEllipse(x - circle_size / 2, y - circle_size / 2, circle_size, circle_size);

  //radar_detected = true;

  QString str;
  //str.sprintf("%.1fm", radar_detected ? radar_dist : vision_dist);
  QColor textColor = QColor(255, 255, 255, 255);
  //configFont(painter, "Inter", 75, "Bold");
  //drawTextWithColor(painter, x, y + sz / 1.5f + 80.0, str, textColor);
  if (radar_detected) {
      float radar_rel_speed = lead_radar.getVRel();
      str.sprintf("%.0fkm/h", m_cur_speed + radar_rel_speed * 3.6);
      if (radar_rel_speed < -0.1) textColor = QColor(255, 0, 0, 255);
      else if (radar_rel_speed > 0.1) textColor = QColor(0, 255, 0, 255);
      else textColor = QColor(255, 255, 255, 255);
      configFont(painter, "Inter", 40, "Bold");
      drawTextWithColor(painter, x, y-130, str, textColor);
  }
  int size = 256;
  painter.setOpacity(0.7);
  painter.drawPixmap(x - size / 2, y - size / 2, size, size, (no_radar)?ic_radar_no: (radar_detected)? ic_radar : ic_radar_vision);
  if (no_radar) {
      if (stop_dist > 0.5) {
          textColor = QColor(255, 255, 255, 255);
          configFont(painter, "Inter", 70, "Bold");
          if (stop_dist < 10.0) str.sprintf("%.1f", stop_dist);
          else str.sprintf("%.0f", stop_dist);
          drawTextWithColor(painter, x, y + 22.0, str, textColor);
      }      
      else if (longActiveUser > 0 && stopping) {
          textColor = QColor(255, 255, 255, 255);
          configFont(painter, "Inter", 40, "Bold");
          if (brake_hold || soft_hold) {
              drawTextWithColor(painter, x, y - 10, (brake_hold)?"AUTO":"SOFT", textColor);
              drawTextWithColor(painter, x, y + 30, "HOLD", textColor);
          }
          else {
              drawTextWithColor(painter, x, y - 10, "신호", textColor);
              drawTextWithColor(painter, x, y + 30, "대기", textColor);
          }
      }
  }
  else {
      textColor = QColor(255, 255, 255, 255);
      configFont(painter, "Inter", 70, "Bold");
      if (disp_dist < 10.0) str.sprintf("%.1f", disp_dist);
      else str.sprintf("%.0f", disp_dist);
      drawTextWithColor(painter, x, y + 22.0, str, textColor);
  }

  int myDrivingMode = controls_state.getMyDrivingMode();
  //const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
  int gap = lp.getCruiseGap();
  //float tFollow = lp.getTFollow();
  int gap1 = controls_state.getLongCruiseGap(); // car_state.getCruiseGap();
  QString strDrivingMode;
  switch (myDrivingMode)
  {
  case 0: strDrivingMode = "GAP"; break;
  case 1: strDrivingMode = "연비"; break;
  case 2: strDrivingMode = "안전"; break;
  case 3: strDrivingMode = "일반"; break;
  case 4: strDrivingMode = "고속"; break;
  }

  int x1 = x - size / 2 - 80;
  int y1 = y - size / 2 + 80;
  configFont(painter, "Inter", 40, "Bold");
  textColor = whiteColor(255);
  drawTextWithColor(painter, x1 + 30, y1 - 25, strDrivingMode, textColor);
  QRect rectGap1(x1, y1, 60, 20);
  QRect rectGap2(x1, y1+35, 60, 20);
  QRect rectGap3(x1, y1+70, 60, 20);
  painter.setBrush(whiteColor(255));
  painter.drawRect(rectGap1);
  if(gap>=2) painter.drawRect(rectGap2);
  if(gap>=3) painter.drawRect(rectGap3);
  configFont(painter, "Inter", 60, "Bold");
  textColor = whiteColor(255);
  //str.sprintf("%.1f", tFollow);
  str.sprintf("%d", gap1);
  drawTextWithColor(painter, x1+30, y + 100, str, textColor);

  //int brake_hold = car_state.getBrakeHoldActive();
  //int soft_hold = (hud_control.getSoftHold()) ? 1 : 0;
  //bool brake_valid = car_state.getBrakeLights();
  if (brake_hold || soft_hold || brake_valid ) {
      if (brake_hold) str.sprintf("AUTOHOLD");
      else if (soft_hold && longActiveUser > 0) str.sprintf("SOFTHOLD");
      else str.sprintf("BRAKE");

      int len = 30 * str.length();
      QRect rectBrake(x-len/2, y + 125, len+4, 45);
      painter.setPen(Qt::NoPen);
      painter.setBrush(redColor(200));
      painter.drawRect(rectBrake);

      configFont(painter, "Inter", 40, "Bold");
      textColor = whiteColor(200);
      drawTextWithColor(painter, x - 0, y + 160, str, textColor);
  }
  else if(longActiveUser > 0) {
      //const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
      const auto lpSource = lp.getLongitudinalPlanSource();
      if (lpSource == cereal::LongitudinalPlan::LongitudinalPlanSource::CRUISE) str.sprintf("CRUISE");
      else if (lpSource == cereal::LongitudinalPlan::LongitudinalPlanSource::LEAD0) str.sprintf("LEAD");
      else if (lpSource == cereal::LongitudinalPlan::LongitudinalPlanSource::LEAD1) str.sprintf("LEAD1");
      else if (lpSource == cereal::LongitudinalPlan::LongitudinalPlanSource::LEAD2) str.sprintf("LEAD2");
      else if (lpSource == cereal::LongitudinalPlan::LongitudinalPlanSource::TURN) str.sprintf("TURN");
      else if (lpSource == cereal::LongitudinalPlan::LongitudinalPlanSource::E2E) str.sprintf("E2E");
      else str.sprintf("LS: UNKNOWN");

      int len = 30 * str.length();
      QRect rectBrake(x - len / 2, y + 125, len + 4, 45);
      painter.setPen(Qt::NoPen);
      painter.setBrush(greenColor(200));
      painter.drawRect(rectBrake);

      configFont(painter, "Inter", 40, "Bold");
      textColor = whiteColor(200);
      drawTextWithColor(painter, x - 0, y + 160, str, textColor);
  }
  float accel = car_state.getAEgo();
  QRect rectAccel(x + 128 + 5, y - 128, 40, 256);
  painter.setPen(Qt::NoPen);
  painter.setBrush(blackColor(20));
  painter.drawRect(rectAccel);
  QRect rectAccelPos(x + 128 + 5, y, 40, -accel / 4. * 128);
  painter.setBrush((accel>=0.0)?yellowColor(200):redColor(200));
  painter.drawRect(rectAccelPos);

  if(0) {
      float v_ego;
      if (sm["carState"].getCarState().getVEgoCluster() == 0.0 && !v_ego_cluster_seen) {
          v_ego = sm["carState"].getCarState().getVEgo();
      }
      else {
          v_ego = sm["carState"].getCarState().getVEgoCluster();
          v_ego_cluster_seen = true;
      }

      const bool cs_alive = sm.alive("controlsState");
      float cur_speed = cs_alive ? std::max<float>(0.0, v_ego) : 0.0;
      cur_speed *= s->scene.is_metric ? MS_TO_KPH : MS_TO_MPH;
      m_cur_speed = cur_speed;
      //auto car_state = sm["carState"].getCarState();
      //float accel = car_state.getAEgo();

      QColor color = QColor(255, 255, 255, 230);

      if (accel > 0) {
          int a = (int)(255.f - (180.f * (accel / 2.f)));
          a = std::min(a, 255);
          a = std::max(a, 80);
          color = QColor(a, a, 255, 230);
      }
      else {
          int a = (int)(255.f - (255.f * (-accel / 3.f)));
          a = std::min(a, 255);
          a = std::max(a, 60);
          color = QColor(255, a, a, 230);
      }

      QString speed;
      speed.sprintf("%.0f", cur_speed);
      configFont(painter, "Inter", 130, "Bold");
      drawTextWithColor(painter, x + 260, y - 50, speed, color);
  }


  painter.restore();
}

void AnnotatedCameraWidget::paintGL() {
}

void AnnotatedCameraWidget::paintEvent(QPaintEvent *event) {

  UIState *s = uiState();
  const cereal::ModelDataV2::Reader &model = (*s->sm)["modelV2"].getModelV2();

  QPainter p(this);

  p.beginNativePainting();
  SubMaster &sm = *(s->sm);
  //const double start_draw_t = millis_since_boot();

  // draw camera frame
  {
    std::lock_guard lk(frame_lock);

    if (frames.empty()) {
      if (skip_frame_count > 0) {
        skip_frame_count--;
        qDebug() << "skipping frame, not ready";
        return;
      }
    } else {
      // skip drawing up to this many frames if we're
      // missing camera frames. this smooths out the
      // transitions from the narrow and wide cameras
      skip_frame_count = 5;
    }

    // Wide or narrow cam dependent on speed
    float v_ego = sm["carState"].getCarState().getVEgo();
    if ((v_ego < 10) || s->wide_cam_only) {
      wide_cam_requested = true;
    } else if (v_ego > 15) {
      wide_cam_requested = false;
    }
    const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    auto xState = lp.getXState();
    bool x = (xState == cereal::LongitudinalPlan::XState::E2E_STOP) || (xState==cereal::LongitudinalPlan::XState::SOFT_HOLD);

    wide_cam_requested = wide_cam_requested && (sm["controlsState"].getControlsState().getExperimentalMode() || x);
    // TODO: also detect when ecam vision stream isn't available
    // for replay of old routes, never go to widecam
    wide_cam_requested = wide_cam_requested && s->scene.calibration_wide_valid;
    CameraWidget::setStreamType(wide_cam_requested ? VISION_STREAM_WIDE_ROAD : VISION_STREAM_ROAD);

    s->scene.wide_cam = CameraWidget::getStreamType() == VISION_STREAM_WIDE_ROAD;
    if (s->scene.calibration_valid) {
      auto calib = s->scene.wide_cam ? s->scene.view_from_wide_calib : s->scene.view_from_calib;
      CameraWidget::updateCalibration(calib);
    } else {
      CameraWidget::updateCalibration(DEFAULT_CALIBRATION);
    }
    CameraWidget::setFrameId(model.getFrameId());
    CameraWidget::paintGL();
  }

  if (s->worldObjectsVisible()) {
    if (sm.updated("uiPlan") && sm.rcv_frame("modelV2") > s->scene.started_frame) {
      update_model(s, model, sm["uiPlan"].getUiPlan());
      if (sm.rcv_frame("radarState") > s->scene.started_frame) {
        update_leads(s, sm["radarState"].getRadarState(), model.getPosition());
      }
    }
    drawHud(p, model);

    // DMoji
    if (!hideDM && (sm.rcv_frame("driverStateV2") > s->scene.started_frame)) {
      update_dmonitoring(s, sm["driverStateV2"].getDriverStateV2(), dm_fade_state, false);
      drawDriverState(p, s);
    }
  }
  p.endNativePainting();

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  double fps = fps_filter.update(1. / dt * 1000);
  m_fps = fps;
  if (fps < 15) {
    //LOGW("slow frame rate: %.2f fps", fps);
  }
  prev_draw_t = cur_draw_t;
}

void AnnotatedCameraWidget::showEvent(QShowEvent *event) {
    CameraWidget::showEvent(event);

  auto now = millis_since_boot();
  if(now - last_update_params > 1000) {
    last_update_params = now;
    ui_update_params(uiState());
  }

  prev_draw_t = millis_since_boot();
}

void AnnotatedCameraWidget::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void AnnotatedCameraWidget::drawTextWithColor(QPainter &p, int x, int y, const QString &text, QColor& color) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(color);
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void AnnotatedCameraWidget::drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity, float rotate/*=0.0*/) {
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);
  p.setOpacity(opacity);
  //p.drawPixmap(x - img_size / 2, y - img_size / 2, img_size, img_size, img);
  if (rotate < 0.0 || rotate >0.0) {
      QMatrix rm;
      rm.rotate(rotate);
      QPixmap img2 = img;
      img2 = img2.transformed(rm);
      p.drawPixmap(x - img2.size().width() / 2, y - img2.size().height() / 2, img2);
  }
  else p.drawPixmap(x - img.size().width() / 2, y - img.size().height() / 2, img);

}

void AnnotatedCameraWidget::drawText2(QPainter &p, int x, int y, int flags, const QString &text, const QColor& color) {
  QFontMetrics fm(p.font());
  QRect rect = fm.boundingRect(text);
  rect.adjust(-1, -1, 1, 1);
  p.setPen(color);
  p.drawText(QRect(x, y, rect.width()+1, rect.height()), flags, text);
}

void AnnotatedCameraWidget::drawHud(QPainter &p, const cereal::ModelDataV2::Reader &model) {

  p.setRenderHint(QPainter::Antialiasing);
  p.setPen(Qt::NoPen);
  p.setOpacity(1.);

  // Header gradient
  QLinearGradient bg(0, header_h - (header_h / 2.5), 0, header_h);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), header_h, bg);

  UIState *s = uiState();

  const SubMaster &sm = *(s->sm);

  drawLaneLines(p, s);

  auto leads = model.getLeadsV3();
  drawLead(p, leads[0], s->scene.lead_vertices[0], s->scene.lead_radar[0], leads[0].getProb() < .5);
  /*
  if (leads[0].getProb() > .5) {
    drawLead(p, leads[0], s->scene.lead_vertices[0], s->scene.lead_radar[0]);
  }
  if (leads[1].getProb() > .5 && (std::abs(leads[1].getX()[0] - leads[0].getX()[0]) > 3.0)) {
    drawLead(p, leads[1], s->scene.lead_vertices[1], s->scene.lead_radar[1]);
  }
  */
  
  //drawMaxSpeed(p);
  drawSpeed(p);
  //drawSteer(p);
  drawDeviceState(p);
  drawTurnSignals(p);
  //if(width() > 1200) drawGpsStatus(p);

  if(s->show_debug && width() > 1200)
    drawDebugText(p);

  //const auto controls_state = sm["controlsState"].getControlsState();
  //const auto car_params = sm["carParams"].getCarParams();
  //const auto live_params = sm["liveParameters"].getLiveParameters();
  const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();

  /*
  int TRsign_w = 140;
  int TRsign_h = 250;
  int TRsign_x = 70;
  int TRsign_y = 545;

  p.setOpacity(0.8);
  if (lp.getTrafficState() >= 100) {
      p.drawPixmap(TRsign_x, TRsign_y, TRsign_w, TRsign_h, ic_trafficLight_x);
  }
  else {
      switch (lp.getTrafficState() % 100) {
      case 0: p.drawPixmap(TRsign_x, TRsign_y, TRsign_w, TRsign_h, ic_trafficLight_none); break;
      case 1: p.drawPixmap(TRsign_x, TRsign_y, TRsign_w, TRsign_h, ic_trafficLight_red); 
      	      p.drawPixmap(400, 400, 350, 350, ic_stopman);
      		  break;
      case 2: p.drawPixmap(TRsign_x, TRsign_y, TRsign_w, TRsign_h, ic_trafficLight_green); break;
      case 3: p.drawPixmap(TRsign_x, TRsign_y, TRsign_w, TRsign_h, ic_trafficLight_x); break;
      }
  }
  */

  QString infoText1, infoText2;
  infoText1 = lp.getDebugLongText1().cStr();
  infoText2 = lp.getDebugLongText2().cStr();

  p.save();
  if (s->show_debug) {
      configFont(p, "Inter", 34, "Regular");
      p.setPen(QColor(0xff, 0xff, 0xff, 200));
      p.drawText(rect().left() + 20, rect().height() - 15, infoText1);
      p.drawText(rect().left() + 20, rect().height() - 45, infoText2);
  }

  auto car_state = sm["carState"].getCarState();
  float steer_angle =  car_state.getSteeringAngleDeg();
  auto gps = sm["gpsLocationExternal"].getGpsLocationExternal();
  bool gpsOK = sm["liveLocationKalman"].getLiveLocationKalman().getGpsOK();
  float accuracy = gps.getAccuracy();
  bool gpsOn = false;
  QString str;
  if (accuracy < 0.01f || accuracy > 20.f || gpsOK==false);
  else {
      gpsOn = true;
      str.sprintf("GPS: %4.1fm", accuracy);
      p.drawText(rect().right() - 240, 360, str);
  }
  str.sprintf("STR: %4.1f°", steer_angle);
  p.drawText(rect().right() - 240, 310, str);
  p.restore();

  drawBottomIcons(p);


  //const auto cs = sm["controlsState"].getControlsState();
  //bool engageable = cs.getEngageable() || cs.getEnabled();
  // engage-ability icon
  //if (1 || engageable) {
  if(gpsOn) {
    //SubMaster &sm = *(uiState()->sm);
      bool experimentalMode = Params().getBool("ExperimentalMode");
    drawIcon(p, rect().right() - radius / 2 - bdr_s * 2, radius / 2 + 80,
        experimentalMode ? experimental_img : gpsOn? ic_satellite :engage_img, QColor(0,0,0,0.)/*blackColor(166)*/, 1.0, -steer_angle);
  }
  
}

static const QColor get_tpms_color(float tpms) {
    if(tpms < 5 || tpms > 60) // N/A
        return QColor(255, 255, 255, 220);
    if(tpms < 31)
        return QColor(255, 90, 90, 220);
    return QColor(255, 255, 255, 220);
}

static const QString get_tpms_text(float tpms) {
    if(tpms < 5 || tpms > 60)
        return "  -";

    char str[32];
    snprintf(str, sizeof(str), "%.0f", round(tpms));
    return QString(str);
}

void AnnotatedCameraWidget::drawBottomIcons(QPainter &p) {
  p.save();
  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();
  //const auto controls_state = sm["controlsState"].getControlsState();
  //auto scc_smoother = sm["carControl"].getCarControl().getSccSmoother();
  //UIState* s = uiState();

  {
      const int x = (btn_size - 24) / 2 + (bdr_s * 2);
      const int y = rect().bottom() - footer_h / 2;
      auto tpms = car_state.getTpms();
      const float fl = tpms.getFl();
      const float fr = tpms.getFr();
      const float rl = tpms.getRl();
      const float rr = tpms.getRr();
      configFont(p, "Inter", 38, "Bold");

      QFontMetrics fm(p.font());
      QRect rcFont = fm.boundingRect("9");

      int center_x = x - 30;
      int center_y = y - 0;
      const int marginX = (int)(rcFont.width() * 3.2f);
      const int marginY = (int)((footer_h / 2 - rcFont.height()) * 0.6f);

      drawText2(p, center_x - marginX, center_y - marginY - rcFont.height(), Qt::AlignRight, get_tpms_text(fl), get_tpms_color(fl));
      drawText2(p, center_x + marginX, center_y - marginY - rcFont.height(), Qt::AlignLeft, get_tpms_text(fr), get_tpms_color(fr));
      drawText2(p, center_x - marginX, center_y + marginY, Qt::AlignRight, get_tpms_text(rl), get_tpms_color(rl));
      drawText2(p, center_x + marginX, center_y + marginY, Qt::AlignLeft, get_tpms_text(rr), get_tpms_color(rr));
  }

#if 0
  // tire pressure
  {
    const int w = 58;
    const int h = 126;
    const int x = radius / 2 + (bdr_s * 2) + (radius + 50);
    //const int x = 110;
    const int y = height() - h - 85 + 15;

    auto tpms = car_state.getTpms();
    const float fl = tpms.getFl();
    const float fr = tpms.getFr();
    const float rl = tpms.getRl();
    const float rr = tpms.getRr();

    p.setOpacity(0.8);
    p.drawPixmap(x, y, w, h, ic_tire_pressure);

    configFont(p, "Inter", 38, "Bold");

    QFontMetrics fm(p.font());
    QRect rcFont = fm.boundingRect("9");

    int center_x = x + 3;
    int center_y = y + h/2;
    const int marginX = (int)(rcFont.width() * 2.7f);
    const int marginY = (int)((h/2 - rcFont.height()) * 0.7f);

    drawText2(p, center_x-marginX, center_y-marginY-rcFont.height(), Qt::AlignRight, get_tpms_text(fl), get_tpms_color(fl));
    drawText2(p, center_x+marginX, center_y-marginY-rcFont.height(), Qt::AlignLeft, get_tpms_text(fr), get_tpms_color(fr));
    drawText2(p, center_x-marginX, center_y+marginY, Qt::AlignRight, get_tpms_text(rl), get_tpms_color(rl));
    drawText2(p, center_x+marginX, center_y+marginY, Qt::AlignLeft, get_tpms_text(rr), get_tpms_color(rr));
  }
#endif
#if 0
  int x = radius / 2 + (bdr_s * 2) + (radius + 50);
  const int y = rect().bottom() - footer_h / 2 - 10 + 15;

  // cruise gap
  int gap = controls_state.getLongCruiseGap(); // car_state.getCruiseGap();
  int myDrivingMode = controls_state.getMyDrivingMode();
  //bool longControl = 0;// scc_smoother.getLongControl();
  //int autoTrGap = 0;// scc_smoother.getAutoTrGap();

  p.setPen(Qt::NoPen);
  p.setBrush(QBrush(QColor(0, 0, 0, 255 * .1f)));
  p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);
  //int x1 = x - radius / 2;
  //int y1 = y - radius / 2;
  //QRect rectGap(x1, y1, radius, radius);
  //printf("%d %d %d\n", x1, y1, radius); // 302, 773, 192
  //p.drawRect(rectGap);
  //QRect opRect(rect().right() - radius, 0, radius, radius);
  //p.drawRect(opRect);


  QString str, strDrivingMode;
  float textSize = 50.f;
  QColor textColor = QColor(255, 255, 255, 200);

  if(gap <= 0) {
    str = "N/A";
  }
  else if (s->scene.longitudinal_control) {
    str.sprintf("%d", (int)gap);
    //str = "AUTO";
    textColor = QColor(120, 255, 120, 200);
  }
  else {
    str.sprintf("%d", (int)gap);
    textColor = QColor(120, 255, 120, 200);
    textSize = 70.f;
  }

  switch (myDrivingMode)
  {
  case 0: strDrivingMode = "GAP"; break;
  case 1: strDrivingMode = "연비"; break;
  case 2: strDrivingMode = "안전"; break;
  case 3: strDrivingMode = "일반"; break;
  case 4: strDrivingMode = "고속"; break;
  }
  configFont(p, "Inter", 35, "Bold");
  drawText(p, x, y-20, strDrivingMode, 200);

  configFont(p, "Inter", textSize, "Bold");
  drawTextWithColor(p, x, y+50, str, textColor);
#endif

#if 0
  // brake
  x = radius / 2 + (bdr_s * 2) + (radius + 50) * 2;
  bool brake_valid = car_state.getBrakeLights();
  float img_alpha = brake_valid ? 1.0f : 0.15f;
  float bg_alpha = brake_valid ? 0.3f : 0.1f;
  drawIcon(p, x, y, ic_brake, QColor(0, 0, 0, (255 * bg_alpha)), img_alpha);

  // auto hold
  //const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
  const auto cs = sm["controlsState"].getControlsState();
  auto car_control = sm["carControl"].getCarControl();
  auto hud_control = car_control.getHudControl();

  //int xState = lp.getXState();
  int enabled = cs.getEnabled();
  int brake_hold = car_state.getBrakeHoldActive();
  int autohold = (hud_control.getSoftHold()) ? 1 : 0;
  if(s->scene.longitudinal_control) autohold = (enabled && hud_control.getSoftHold()) ? 1 : 0;
  else autohold = (brake_hold > 0) ? 1 : 0;
  if(true) {

    x = radius / 2 + (bdr_s * 2) + (radius + 50) * 3;
    img_alpha = autohold > 0 ? 1.0f : 0.15f;
    bg_alpha = autohold > 0 ? 0.3f : 0.1f;
    drawIcon(p, x, y, autohold ? ic_autohold_warning : ic_autohold_active,
            QColor(0, 0, 0, (255 * bg_alpha)), img_alpha);
  }
#endif
  p.restore();
}

void AnnotatedCameraWidget::drawApilot(QPainter& p) {

    UIState* s = uiState();
    const SubMaster& sm = *(s->sm);
    const auto cs = sm["controlsState"].getControlsState();
    const auto car_state = sm["carState"].getCarState();
    //const auto scc_smoother = sm["carControl"].getCarControl().getSccSmoother();
    const auto road_limit_speed = sm["roadLimitSpeed"].getRoadLimitSpeed();
    const auto navi_info = car_state.getNaviSafetyInfo();
    const auto car_params = sm["carParams"].getCarParams();

    //bool is_metric = s->scene.is_metric;
    bool long_control = 1;// scc_smoother.getLongControl();

    // kph
    float applyMaxSpeed = cs.getVCruiseOut();// scc_smoother.getApplyMaxSpeed();
    float cruiseMaxSpeed = cs.getVCruiseCluster();// scc_smoother.getCruiseMaxSpeed();

    //bool is_cruise_set = (cruiseMaxSpeed > 0 && cruiseMaxSpeed < 255);
    //bool is_cruise_set = (applyMaxSpeed > 0 && applyMaxSpeed < 255);
    int longActiveUser = cs.getLongActiveUser();

    int sccBus = (int)car_params.getSccBus();
    int navCluster = (int)car_params.getNaviCluster();

    int enabled = cs.getEnabled();

    int activeNDA = road_limit_speed.getActive();
    int roadLimitSpeed = road_limit_speed.getRoadLimitSpeed();
    int camLimitSpeed = road_limit_speed.getCamLimitSpeed();
    int camLimitSpeedLeftDist = road_limit_speed.getCamLimitSpeedLeftDist();
    int sectionLimitSpeed = road_limit_speed.getSectionLimitSpeed();
    int sectionLeftDist = road_limit_speed.getSectionLeftDist();

    int limit_speed = 0;
    int left_dist = 0;

    if (camLimitSpeed > 0 && camLimitSpeedLeftDist > 0) {
        limit_speed = camLimitSpeed;
        left_dist = camLimitSpeedLeftDist;
    }
    else if (sectionLimitSpeed > 0 && sectionLeftDist > 0) {
        limit_speed = sectionLimitSpeed;
        left_dist = sectionLeftDist;
    }

    int radar_tracks = Params().getBool("EnableRadarTracks");
    QString nda_mode_str = QString::fromStdString(Params().get("AutoNaviSpeedCtrl"));
    int nda_mode = nda_mode_str.toInt();
    // debug Code
    int w = 120;
    int dx = w + 15;
    int h = 54;
    int x = (width() + (bdr_s * 2)) / 2 - w / 2 - bdr_s - dx;
    int y = 40 - bdr_s;

    if (sccBus == 2) {
        p.drawPixmap(x, y, w, h, ic_scc2);
        x += dx;
    }
    if (navCluster == 1 && nda_mode == 2) {
        p.drawPixmap(x, y, w, h, ic_navi);
        x += dx;
    }

    if (activeNDA > 0 && nda_mode > 0) {
        p.setOpacity(1.f);
        p.drawPixmap(x, y, w, h, activeNDA == 1 ? ic_nda : ic_hda);
        x += dx;
    }
    else {
        limit_speed = navi_info.getSpeedLimit();
        left_dist = navi_info.getDist();
    }
    if (radar_tracks) {
        p.drawPixmap(x, y, w * 2, h, ic_radartracks);
        x += (w + dx);
    }

    float v_ego;
    if (sm["carState"].getCarState().getVEgoCluster() == 0.0 && !v_ego_cluster_seen) {
        v_ego = sm["carState"].getCarState().getVEgo();
    }
    else {
        v_ego = sm["carState"].getCarState().getVEgoCluster();
        v_ego_cluster_seen = true;
    }

    const bool cs_alive = sm.alive("controlsState");
    float cur_speed = cs_alive ? std::max<float>(0.0, v_ego) : 0.0;
    cur_speed *= s->scene.is_metric ? MS_TO_KPH : MS_TO_MPH;
    m_cur_speed = cur_speed;
    float accel = car_state.getAEgo();

    QColor color = QColor(255, 255, 255, 230);

    if (accel > 0) {
        int a = (int)(255.f - (180.f * (accel / 2.f)));
        a = std::min(a, 255);
        a = std::max(a, 80);
        color = QColor(a, a, 255, 230);
    }
    else {
        int a = (int)(255.f - (255.f * (-accel / 3.f)));
        a = std::min(a, 255);
        a = std::max(a, 60);
        color = QColor(255, a, a, 230);
    }

    x = width() / 2;
    y = height() - 230;
    if (width() < 1200) x += 300;
    QRect rectSpeed(x - 500, y + 215, 1000, 5);
    //QRect rect1(x - 500, y+90, 10, 100);
    //QRect rect2(x + 500 - 10, y+90, 10, 100);
    p.setPen(Qt::NoPen);
    p.setBrush(whiteColor(255));
    //p.drawRect(rectSpeed);
    //p.drawRect(rect1);
    //p.drawRect(rect2);

    QString speed, str;
    speed.sprintf("%.0f", cur_speed);
    configFont(p, "Inter", 170, "Bold");
    drawTextWithColor(p, x, y + 170, speed, color);

    color = whiteColor(255);

    configFont(p, "Inter", 40, "Bold");
    drawTextWithColor(p, x - 250, y + 35, "CRUISE", color);
    configFont(p, "Inter", 70, "Bold");
    if (enabled && longActiveUser > 0) str.sprintf("%d", (int)(cruiseMaxSpeed + 0.5));
    else str = "N/A";
    drawTextWithColor(p, x - 250, y + 100, str, color);
    QRect rectBar(x - 250 - 50, y + 100 + 15, 100, 5);
    p.drawRect(rectBar);
    configFont(p, "Inter", 60, "Bold");
    if (enabled && longActiveUser > 0) str.sprintf("%d", (int)(applyMaxSpeed + 0.5));
    else str = long_control ? "OP" : "MAX";
    drawTextWithColor(p, x - 250, y + 180, str, color);

    QColor blackColor = QColor(0, 0, 0, 230);
    if (limit_speed > 0) {
        QRect rectLimit(x - 500, y, 140, 140);
        p.setBrush(QBrush(Qt::white));
        p.drawEllipse(rectLimit);
        int padding = 10;
        rectLimit.adjust(padding, padding, -padding, -padding);
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(Qt::red, 12));
        p.drawEllipse(rectLimit);
        configFont(p, "Inter", 60, "Bold");        
        str.sprintf("%d", limit_speed);
        drawTextWithColor(p, x - 500 + 75, y + 90, str, blackColor);
        if (left_dist > 0) {
            configFont(p, "Inter", 40, "Bold");
            if (left_dist < 1000) str.sprintf("%dm", left_dist);
            else  str.sprintf("%.1fkm", left_dist / 1000.f);
            drawTextWithColor(p, x - 500 + 75, y + 180, str, color);
        }
    }
    else if (roadLimitSpeed > 0 && roadLimitSpeed < 200) {
        QRect rect(x - 500, y, 160, 190);
        p.setBrush(QBrush(Qt::white));
        p.drawRoundedRect(rect, 16, 16);
        int padding = 10;
        rect.adjust(padding, padding, -padding, -padding);
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(Qt::black, padding));
        p.drawRoundedRect(rect, 8, 8);
        
        str = "SPEED";
        configFont(p, "Inter", 35, "Bold");
        drawTextWithColor(p, x - 500 + 75, y + 50, "SPEED", blackColor);
        drawTextWithColor(p, x - 500 + 75, y + 85, "LIMIT", blackColor);
        //roadLimitSpeed = 100;
        str.sprintf("%d", roadLimitSpeed);
        configFont(p, "Inter", 50, "Bold");
        drawTextWithColor(p, x - 500 + 75, y + 150, str, blackColor);
    }
}
void AnnotatedCameraWidget::drawSpeed(QPainter &p) {
  p.save();
  drawApilot(p);
  UIState* s = uiState();
#if 0
  const SubMaster &sm = *(s->sm);
  float v_ego;
  if (sm["carState"].getCarState().getVEgoCluster() == 0.0 && !v_ego_cluster_seen) {
    v_ego = sm["carState"].getCarState().getVEgo();
  } else {
    v_ego = sm["carState"].getCarState().getVEgoCluster();
    v_ego_cluster_seen = true;
  }

  const bool cs_alive = sm.alive("controlsState");
  float cur_speed = cs_alive ? std::max<float>(0.0, v_ego) : 0.0;
  cur_speed  *= s->scene.is_metric ? MS_TO_KPH : MS_TO_MPH;
  m_cur_speed = cur_speed;
  auto car_state = sm["carState"].getCarState();
  float accel = car_state.getAEgo();

  QColor color = QColor(255, 255, 255, 230);

  if(accel > 0) {
    int a = (int)(255.f - (180.f * (accel/2.f)));
    a = std::min(a, 255);
    a = std::max(a, 80);
    color = QColor(a, a, 255, 230);
  }
  else {
    int a = (int)(255.f - (255.f * (-accel/3.f)));
    a = std::min(a, 255);
    a = std::max(a, 60);
    color = QColor(255, a, a, 230);
  }
  QString speed;
  speed.sprintf("%.0f", cur_speed);
  configFont(p, "Inter", 176, "Bold");
  drawTextWithColor(p, rect().center().x(), 230, speed, color);

  configFont(p, "Inter", 66, "Regular");
  //drawText(p, rect().center().x(), 310, s->scene.is_metric ? "km/h" : "mph", 200);
#endif
#if 0
  const auto lmd = sm["liveMapData"].getLiveMapData();
  const uint64_t lmd_fix_time = lmd.getLastGpsTimestamp();
  const uint64_t current_ts = std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::system_clock::now().time_since_epoch()).count();
  const bool show_road_name = current_ts - lmd_fix_time < 10000; // hide if fix older than 10s
  QString str1;
  str1 = show_road_name ? QString::fromStdString(lmd.getCurrentRoadName()) : "";
  drawText(p, rect().center().x(), 350, str1, 200);
#endif
  
  if (s->show_datetime) { // && width() > 1200) {
      // ajouatom: 현재시간표시
#if 1
      QColor color = QColor(255, 255, 255, 230);
      configFont(p, "Open Sans", 80, "Bold");
      drawTextWithColor(p, 150, height() - 400, QDateTime::currentDateTime().toString("hh:mm"), color);
      configFont(p, "Open Sans", 45, "Bold");
      drawTextWithColor(p, 150, height() - 400 + 80, QDateTime::currentDateTime().toString("MM-dd-ddd"), color);
#else
      QTextOption  textOpt = QTextOption(Qt::AlignLeft);
      configFont(p, "Open Sans", 110, "Bold");
      //p.drawText(QRect(270, 50, width(), 500), QDateTime::currentDateTime().toString("hh:mmap"), textOpt);
      p.drawText(QRect(280, 35, width(), 500), QDateTime::currentDateTime().toString("hh:mm"), textOpt);
      configFont(p, "Open Sans", 50, "Bold");
      p.drawText(QRect(280, 35 + 150, width(), 500), QDateTime::currentDateTime().toString("MM월 dd일 (ddd)"), textOpt);
#endif
  }

  p.restore();
}

QRect getRect(QPainter &p, int flags, QString text) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  return fm.boundingRect(init_rect, flags, text);
}

void AnnotatedCameraWidget::drawMaxSpeed(QPainter &p) {
  p.save();

  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  const auto cs = sm["controlsState"].getControlsState();
  const auto car_state = sm["carState"].getCarState();
  //const auto scc_smoother = sm["carControl"].getCarControl().getSccSmoother();
  const auto road_limit_speed = sm["roadLimitSpeed"].getRoadLimitSpeed();
  const auto navi_info = car_state.getNaviSafetyInfo();
  const auto car_params = sm["carParams"].getCarParams();

  bool is_metric = s->scene.is_metric;
  bool long_control = 1;// scc_smoother.getLongControl();

  // kph
  float applyMaxSpeed = cs.getVCruiseOut();// scc_smoother.getApplyMaxSpeed();
  float cruiseMaxSpeed = cs.getVCruiseCluster();// scc_smoother.getCruiseMaxSpeed();

  //bool is_cruise_set = (cruiseMaxSpeed > 0 && cruiseMaxSpeed < 255);
  //bool is_cruise_set = (applyMaxSpeed > 0 && applyMaxSpeed < 255);
  int longActiveUser = cs.getLongActiveUser();

  int sccBus = (int)car_params.getSccBus();
  int navCluster = (int)car_params.getNaviCluster();

  int enabled = cs.getEnabled();

  int activeNDA = road_limit_speed.getActive();
  int roadLimitSpeed = road_limit_speed.getRoadLimitSpeed();
  int camLimitSpeed = road_limit_speed.getCamLimitSpeed();
  int camLimitSpeedLeftDist = road_limit_speed.getCamLimitSpeedLeftDist();
  int sectionLimitSpeed = road_limit_speed.getSectionLimitSpeed();
  int sectionLeftDist = road_limit_speed.getSectionLeftDist();

  int limit_speed = 0;
  int left_dist = 0;

  if(camLimitSpeed > 0 && camLimitSpeedLeftDist > 0) {
    limit_speed = camLimitSpeed;
    left_dist = camLimitSpeedLeftDist;
  }
  else if(sectionLimitSpeed > 0 && sectionLeftDist > 0) {
    limit_speed = sectionLimitSpeed;
    left_dist = sectionLeftDist;
  }

  int radar_tracks = Params().getBool("EnableRadarTracks");
  QString nda_mode_str = QString::fromStdString(Params().get("AutoNaviSpeedCtrl"));
  int nda_mode = nda_mode_str.toInt();
  // debug Code
  int w = 120;
  int dx = w + 15;
  int h = 54;
  int x = (width() + (bdr_s * 2)) / 2 - w / 2 - bdr_s - dx;
  int y = 40 - bdr_s;

  if (sccBus == 2) {
      p.drawPixmap(x, y, w, h, ic_scc2); 
      x += dx;
  }
  if (navCluster == 1 && nda_mode==2) {
      p.drawPixmap(x, y, w, h, ic_navi); 
      x += dx;
  }

  if (activeNDA > 0 && nda_mode>0) {
      p.setOpacity(1.f);
      p.drawPixmap(x, y, w, h, activeNDA == 1 ? ic_nda : ic_hda);
      x += dx;
  }
  else {
      limit_speed = navi_info.getSpeedLimit();
      left_dist = navi_info.getDist();
  }
  if (radar_tracks) {
      p.drawPixmap(x, y, w * 2, h, ic_radartracks);
      x += (w + dx);
  }


  const int x_start = 30;
  const int y_start = 30;

  int board_width = 210;
  int board_height = 384;

  const int corner_radius = 32;
  int max_speed_height = 210;

  QColor bgColor = QColor(0, 0, 0, 166);
  /*
  const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
  if (lp.getTrafficState() >= 100) bgColor = blackColor(166);
  else {
      switch (lp.getTrafficState() % 100) {
      case 0: bgColor = blackColor(166); break;
      case 1: bgColor = redColor(166); 
          p.drawPixmap(400, 400, 350, 350, ic_stopman);
          break;
      case 2: bgColor = greenColor(166); break;
      case 3: bgColor = yellowColor(166); break;
      }
  }
  */

  {
    // draw board
    QPainterPath path;
    path.setFillRule(Qt::WindingFill);

    if(limit_speed > 0) {
      board_width = limit_speed < 100 ? 210 : 230;
      board_height = max_speed_height + board_width;

      path.addRoundedRect(QRectF(x_start, y_start, board_width, board_height-board_width/2), corner_radius, corner_radius);
      path.addRoundedRect(QRectF(x_start, y_start+corner_radius, board_width, board_height-corner_radius), board_width/2, board_width/2);
    }
    else if(roadLimitSpeed > 0 && roadLimitSpeed < 200) {
      board_height = 485;
      path.addRoundedRect(QRectF(x_start, y_start, board_width, board_height), corner_radius, corner_radius);
    }
    else {
      max_speed_height = 235;
      board_height = max_speed_height;
      path.addRoundedRect(QRectF(x_start, y_start, board_width, board_height), corner_radius, corner_radius);
    }

    p.setPen(Qt::NoPen);
    p.fillPath(path.simplified(), bgColor);
  }

  QString str;

  // Max Speed
  {
    p.setPen(QColor(255, 255, 255, 230));

    //if(is_cruise_set) {
    if(enabled && longActiveUser>0) {
      configFont(p, "Inter", 80, "Bold");

      if(is_metric)
        str.sprintf( "%d", (int)(cruiseMaxSpeed + 0.5));
      else
        str.sprintf( "%d", (int)(cruiseMaxSpeed*KM_TO_MILE + 0.5));
    }
    else {
      configFont(p, "Inter", 60, "Bold");
      str = "N/A";
    }

    QRect speed_rect = getRect(p, Qt::AlignCenter, str);
    QRect max_speed_rect(x_start, y_start, board_width, max_speed_height/2);
    speed_rect.moveCenter({max_speed_rect.center().x(), 0});
    speed_rect.moveTop(max_speed_rect.top() + 35);
    p.drawText(speed_rect, Qt::AlignCenter | Qt::AlignVCenter, str);
  }


  // applyMaxSpeed
  {
    p.setPen(QColor(255, 255, 255, 180));

    configFont(p, "Inter", 50, "Bold");
    if (enabled && longActiveUser > 0) {
    //if(is_cruise_set && applyMaxSpeed > 0) {
      if(is_metric)
        str.sprintf( "%d", (int)(applyMaxSpeed + 0.5));
      else
        str.sprintf( "%d", (int)(applyMaxSpeed*KM_TO_MILE + 0.5));
    }
    else {
      str = long_control ? "OP" : "MAX";
    }

    QRect speed_rect = getRect(p, Qt::AlignCenter, str);
    QRect max_speed_rect(x_start, y_start + max_speed_height/2, board_width, max_speed_height/2);
    speed_rect.moveCenter({max_speed_rect.center().x(), 0});
    speed_rect.moveTop(max_speed_rect.top() + 24);
    p.drawText(speed_rect, Qt::AlignCenter | Qt::AlignVCenter, str);
  }

  //
  if(limit_speed > 0) {
    QRect board_rect = QRect(x_start, y_start+board_height-board_width, board_width, board_width);
    int padding = 14;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(QBrush(Qt::white));
    p.drawEllipse(board_rect);

    padding = 18;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(Qt::red, 25));
    p.drawEllipse(board_rect);

    p.setPen(QPen(Qt::black, padding));

    str.sprintf("%d", limit_speed);
    configFont(p, "Inter", 70, "Bold");

    QRect text_rect = getRect(p, Qt::AlignCenter, str);
    QRect b_rect = board_rect;
    text_rect.moveCenter({b_rect.center().x(), 0});
    text_rect.moveTop(b_rect.top() + (b_rect.height() - text_rect.height()) / 2);
    p.drawText(text_rect, Qt::AlignCenter, str);

    // left dist
    if (left_dist > 0) {
        QRect rcLeftDist;
        QString strLeftDist;

        if (left_dist < 1000)
            strLeftDist.sprintf("%dm", left_dist);
        else
            strLeftDist.sprintf("%.1fkm", left_dist / 1000.f);

        QFont font("Inter");
        font.setPixelSize(55);
        font.setStyleName("Bold");

        QFontMetrics fm(font);
        int width = fm.width(strLeftDist);

        padding = 10;

        int center_x = x_start + board_width / 2;
        rcLeftDist.setRect(center_x - width / 2, y_start + board_height + 15, width, font.pixelSize() + 10);
        rcLeftDist.adjust(-padding * 2, -padding, padding * 2, padding);

        p.setPen(Qt::NoPen);
        p.setBrush(bgColor);
        p.drawRoundedRect(rcLeftDist, 20, 20);

        configFont(p, "Inter", 55, "Bold");
        p.setBrush(Qt::NoBrush);
        p.setPen(QColor(255, 255, 255, 230));
        p.drawText(rcLeftDist, Qt::AlignCenter | Qt::AlignVCenter, strLeftDist);
    }
  }
  else if(roadLimitSpeed > 0 && roadLimitSpeed < 200) {
    QRectF board_rect = QRectF(x_start, y_start+max_speed_height, board_width, board_height-max_speed_height);
    int padding = 14;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(QBrush(Qt::white));
    p.drawRoundedRect(board_rect, corner_radius-padding/2, corner_radius-padding/2);

    padding = 10;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(Qt::black, padding));
    p.drawRoundedRect(board_rect, corner_radius-12, corner_radius-12);

    {
      str = "SPEED\nLIMIT";
      configFont(p, "Inter", 35, "Bold");

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y(), board_rect.width(), board_rect.height()/2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + 20);
      p.drawText(text_rect, Qt::AlignCenter, str);
    }

    {
      str.sprintf("%d", roadLimitSpeed);
      configFont(p, "Inter", 75, "Bold");

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y()+board_rect.height()/2, board_rect.width(), board_rect.height()/2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + 3);
      p.drawText(text_rect, Qt::AlignCenter, str);
    }
  }

  p.restore();
}

void AnnotatedCameraWidget::drawSteer(QPainter &p) {
  p.save();

  int x = 30;
  int y = 500;// 540;

  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();
  auto car_control = sm["carControl"].getCarControl();

  float steer_angle = car_state.getSteeringAngleDeg();
  float desire_angle = car_control.getActuators().getSteeringAngleDeg();

  configFont(p, "Inter", 50, "Bold");

  QString str;
  int width = 192;

  str.sprintf("%.1f°", steer_angle);
  QRect rect = QRect(x, y, width, width);

  p.setPen(QColor(255, 255, 255, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  str.sprintf("%.1f°", desire_angle);
  rect.setRect(x, y + 60, width, width);

  p.setPen(QColor(155, 255, 155, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  auto lead_radar = sm["radarState"].getRadarState().getLeadOne();
  auto lead_one = sm["modelV2"].getModelV2().getLeadsV3()[0];

  float radar_dist = lead_radar.getStatus() && lead_radar.getRadar() ? lead_radar.getDRel() : 0;
  float radar_rel_speed = lead_radar.getStatus() && lead_radar.getRadar() ? lead_radar.getVRel() : 0;
  float vision_dist = lead_one.getProb() > .5 ? (lead_one.getX()[0] - 0) : 0;

  rect.setRect(x, y + 240, 600, width);
  str.sprintf("L:%.1f,V%.1f(%.1f)\n", radar_dist, vision_dist, radar_rel_speed * 3.6);
  p.drawText(rect, Qt::AlignLeft, str);


  p.restore();
}

template <class T>
float interp(float x, std::initializer_list<T> x_list, std::initializer_list<T> y_list, bool extrapolate)
{
  std::vector<T> xData(x_list);
  std::vector<T> yData(y_list);
  int size = xData.size();

  int i = 0;
  if(x >= xData[size - 2]) {
    i = size - 2;
  }
  else {
    while ( x > xData[i+1] ) i++;
  }
  T xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];
  if (!extrapolate) {
    if ( x < xL ) yR = yL;
    if ( x > xR ) yL = yR;
  }

  T dydx = ( yR - yL ) / ( xR - xL );
  return yL + dydx * ( x - xL );
}

void AnnotatedCameraWidget::drawDeviceState(QPainter &p) {
  p.save();

  const SubMaster &sm = *(uiState()->sm);
  auto deviceState = sm["deviceState"].getDeviceState();
  auto car_state = sm["carState"].getCarState();

  const auto freeSpacePercent = deviceState.getFreeSpacePercent();

  const auto cpuTempC = deviceState.getCpuTempC();
  //const auto gpuTempC = deviceState.getGpuTempC();
  float ambientTemp = deviceState.getAmbientTempC();

  float cpuTemp = 0.f;
  //float gpuTemp = 0.f;

  if(std::size(cpuTempC) > 0) {
    for(int i = 0; i < std::size(cpuTempC); i++) {
      cpuTemp += cpuTempC[i];
    }
    cpuTemp = cpuTemp / (float)std::size(cpuTempC);
  }

  /*if(std::size(gpuTempC) > 0) {
    for(int i = 0; i < std::size(gpuTempC); i++) {
      gpuTemp += gpuTempC[i];
    }
    gpuTemp = gpuTemp / (float)std::size(gpuTempC);
    cpuTemp = (cpuTemp + gpuTemp) / 2.f;
  }*/
#if 1
  QString str;
  str.sprintf("STORAGE: %.0f%%   CPU: %.0f°C    AMBIENT: %.0f°C", freeSpacePercent, cpuTemp, ambientTemp);
  int r = interp<float>(cpuTemp, { 50.f, 90.f }, { 200.f, 255.f }, false);
  int g = interp<float>(cpuTemp, { 50.f, 90.f }, { 255.f, 200.f }, false);
  QColor textColor = QColor(r, g, 200, 200);
  configFont(p, "Inter", 30, "Bold");
  if (width() > 1200) {
      drawTextWithColor(p, width() - 350, 35, str, textColor);
      str.sprintf("RPM: %.0f CHARGE: %.0f%%", car_state.getEngineRpm(), car_state.getChargeMeter());
      drawTextWithColor(p, width() - 350, 80, str, textColor);
  }
#else
  int w = 192;
  int x = width() - (30 + w);
  int y = 340;

  QString str;
  QRect rect;

  configFont(p, "Inter", 50, "Bold");
  str.sprintf("%.0f%%", freeSpacePercent);
  rect = QRect(x, y, w, w);

  int r = interp<float>(freeSpacePercent, {10.f, 90.f}, {255.f, 200.f}, false);
  int g = interp<float>(freeSpacePercent, {10.f, 90.f}, {200.f, 255.f}, false);
  p.setPen(QColor(r, g, 200, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  y += 55;
  configFont(p, "Inter", 25, "Bold");
  rect = QRect(x, y, w, w);
  p.setPen(QColor(255, 255, 255, 200));
  p.drawText(rect, Qt::AlignCenter, "STORAGE");

  y += 80;
  configFont(p, "Inter", 50, "Bold");
  str.sprintf("%.0f°C", cpuTemp);
  rect = QRect(x, y, w, w);
  r = interp<float>(cpuTemp, {50.f, 90.f}, {200.f, 255.f}, false);
  g = interp<float>(cpuTemp, {50.f, 90.f}, {255.f, 200.f}, false);
  p.setPen(QColor(r, g, 200, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  y += 55;
  configFont(p, "Inter", 25, "Bold");
  rect = QRect(x, y, w, w);
  p.setPen(QColor(255, 255, 255, 200));
  p.drawText(rect, Qt::AlignCenter, "CPU");

  y += 80;
  configFont(p, "Inter", 50, "Bold");
  str.sprintf("%.0f°C", ambientTemp);
  rect = QRect(x, y, w, w);
  r = interp<float>(ambientTemp, {35.f, 60.f}, {200.f, 255.f}, false);
  g = interp<float>(ambientTemp, {35.f, 60.f}, {255.f, 200.f}, false);
  p.setPen(QColor(r, g, 200, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  y += 55;
  configFont(p, "Inter", 25, "Bold");
  rect = QRect(x, y, w, w);
  p.setPen(QColor(255, 255, 255, 200));
  p.drawText(rect, Qt::AlignCenter, "AMBIENT");
#endif
  p.restore();
}

void AnnotatedCameraWidget::drawTurnSignals(QPainter &p) {
  p.save();

  static int blink_index = 0;
  static int blink_wait = 0;
  static double prev_ts = 0.0;

  if(blink_wait > 0) {
    blink_wait--;
    blink_index = 0;
  }
  else {
    const SubMaster &sm = *(uiState()->sm);
    auto car_state = sm["carState"].getCarState();
    bool left_on = car_state.getLeftBlinker();
    bool right_on = car_state.getRightBlinker();

    const float img_alpha = 0.8f;
    const int fb_w = width() / 2 - 200;
    const int center_x = width() / 2;
    const int w = fb_w / 25;
    const int h = 160;
    const int gap = fb_w / 25;
    const int margin = (int)(fb_w / 3.8f);
    const int base_y = (height() - h) / 2;
    const int draw_count = 8;

    int x = center_x;
    int y = base_y;

    if(left_on) {
      for(int i = 0; i < draw_count; i++) {
        float alpha = img_alpha;
        int d = std::abs(blink_index - i);
        if(d > 0)
          alpha /= d*2;

        p.setOpacity(alpha);
        float factor = (float)draw_count / (i + draw_count);
        p.drawPixmap(x - w - margin, y + (h-h*factor)/2, w*factor, h*factor, ic_turn_signal_l);
        x -= gap + w;
      }
    }

    x = center_x;
    if(right_on) {
      for(int i = 0; i < draw_count; i++) {
        float alpha = img_alpha;
        int d = std::abs(blink_index - i);
        if(d > 0)
          alpha /= d*2;

        float factor = (float)draw_count / (i + draw_count);
        p.setOpacity(alpha);
        p.drawPixmap(x + margin, y + (h-h*factor)/2, w*factor, h*factor, ic_turn_signal_r);
        x += gap + w;
      }
    }

    if(left_on || right_on) {

      double now = millis_since_boot();
      if(now - prev_ts > 900/UI_FREQ) {
        prev_ts = now;
        blink_index++;
      }

      if(blink_index >= draw_count) {
        blink_index = draw_count - 1;
        blink_wait = UI_FREQ/4;
      }
    }
    else {
      blink_index = 0;
    }
  }

  p.restore();
}

void AnnotatedCameraWidget::drawGpsStatus(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto gps = sm["gpsLocationExternal"].getGpsLocationExternal();
  float accuracy = gps.getAccuracy();
  if (accuracy < 0.01f || accuracy > 20.f)
      return;

  int w = 120;
  int h = 100;
  int x = width() - w - 30 - 250;
  int y = 30+50;

  p.save();

  p.setOpacity(0.8);
  p.drawPixmap(x, y, w, h, ic_satellite);

  configFont(p, "Inter", 40, "Bold");
  p.setPen(QColor(255, 255, 255, 200));
  p.setRenderHint(QPainter::TextAntialiasing);

  QRect rect = QRect(x, y + h + 10, w, 40);
  rect.adjust(-30, 0, 30, 0);

  QString str;
  str.sprintf("%.1fm", accuracy);
  p.drawText(rect, Qt::AlignHCenter, str);

  p.restore();
}

void AnnotatedCameraWidget::drawDebugText(QPainter &p) {
  p.save();
  const SubMaster &sm = *(uiState()->sm);
  QString str, temp;

  int y = 80 + 180;
  const int height = 60;

  const int text_x = width()/2 + 250;

  auto controls_state = sm["controlsState"].getControlsState();
  auto car_control = sm["carControl"].getCarControl();
  auto car_state = sm["carState"].getCarState();

  float gas = car_state.getGas();
  //float brake = car_state.getBrake();
  //float applyAccel = 0.;//controls_state.getApplyAccel();

  //float aReqValue = 0.;//controls_state.getAReqValue();
  //float aReqValueMin = 0.;//controls_state.getAReqValueMin();
  //float aReqValueMax = 0.;//controls_state.getAReqValueMax();

  //int sccStockCamAct = (int)controls_state.getSccStockCamAct();
  //int sccStockCamStatus = (int)controls_state.getSccStockCamStatus();
  QString debugText1 = controls_state.getDebugText1().cStr();
  QString debugText2 = controls_state.getDebugText2().cStr();
  QString debugTextCC = car_control.getDebugTextCC().cStr();

  //float vEgo = car_state.getVEgo();
  //float vEgoRaw = car_state.getVEgoRaw();
  int longControlState = (int)controls_state.getLongControlState();
  //float vPid = controls_state.getVPid();
  //float upAccelCmd = controls_state.getUpAccelCmd();
  //float uiAccelCmd = controls_state.getUiAccelCmd();
  //float ufAccelCmd = controls_state.getUfAccelCmd();
  float accel = car_control.getActuators().getAccel();

  const char* long_state[] = {"off", "pid", "stopping", "starting"};

  configFont(p, "Inter", 35, "Regular");
  p.setPen(QColor(255, 255, 255, 200));
  p.setRenderHint(QPainter::TextAntialiasing);

  str.sprintf("State: %s\n", long_state[longControlState]);
  p.drawText(text_x, y, str);

  y += height;
  p.drawText(text_x, y, debugText1);
  y += height;
  p.drawText(text_x, y, debugText2);
  y += height;
  p.drawText(text_x, y, debugTextCC);
  //printf("debugTextCC=%s\n", debugTextCC.toStdString().c_str());

  y += height;
  str.sprintf("FPS: %d\n", m_fps);
  p.drawText(text_x, y, str);

#if 0
  y += height;
  str.sprintf("vEgo: %.2f/%.2f\n", vEgo*3.6f, vEgoRaw*3.6f);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("vPid: %.2f/%.2f\n", vPid, vPid*3.6f);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("P: %.3f\n", upAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("I: %.3f\n", uiAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("F: %.3f\n", ufAccelCmd);
  p.drawText(text_x, y, str);
#endif

  y += height;
  str.sprintf("Accel: %.3f\nGAS: %.1f%%\n", accel, gas);
  p.drawText(text_x, y, str);

  //y += height;
  //str.sprintf("Apply: %.3f, Stock: %.3f\n", applyAccel, aReqValue);
  //p.drawText(text_x, y, str);

  //y += height;
  //str.sprintf("%.3f (%.3f/%.3f)\n", aReqValue, aReqValueMin, aReqValueMax);
  //p.drawText(text_x, y, str);

  //y += height;
  //str.sprintf("aEgo: %.3f\n", car_state.getAEgo());
  //p.drawText(text_x, y, str);

#if 0
  auto lead_radar = sm["radarState"].getRadarState().getLeadOne();
  auto lead_one = sm["modelV2"].getModelV2().getLeadsV3()[0];

  float radar_dist = lead_radar.getStatus() && lead_radar.getRadar() ? lead_radar.getDRel() : 0;
  float radar_rel_speed = lead_radar.getStatus() && lead_radar.getRadar() ? lead_radar.getVRel() : 0;
  float vision_dist = lead_one.getProb() > .5 ? (lead_one.getX()[0] - 1.5) : 0;

  y += height;
  str.sprintf("Lead: %.1f(%.1f)/%.1f/%.1f\n", radar_dist, radar_rel_speed*3.6, vision_dist, (radar_dist - vision_dist));
  p.drawText(text_x, y, str);
#endif

  const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
  const auto lpSource = lp.getLongitudinalPlanSource();

  if (lpSource == cereal::LongitudinalPlan::LongitudinalPlanSource::CRUISE) str.sprintf("LS: CRUISE");
  else if (lpSource == cereal::LongitudinalPlan::LongitudinalPlanSource::LEAD0) str.sprintf("LS: LEAD0");
  else if (lpSource == cereal::LongitudinalPlan::LongitudinalPlanSource::LEAD1) str.sprintf("LS: LEAD1");
  else if (lpSource == cereal::LongitudinalPlan::LongitudinalPlanSource::LEAD2) str.sprintf("LS: LEAD2");
  else if (lpSource == cereal::LongitudinalPlan::LongitudinalPlanSource::E2E) str.sprintf("LS: E2E");
  else str.sprintf("LS: UNKNOWN");

  y += height;
  p.drawText(text_x, y, str);

#if 0
  const auto lmd = sm["liveMapData"].getLiveMapData();
  const uint64_t lmd_fix_time = lmd.getLastGpsTimestamp();
  const uint64_t current_ts = std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::system_clock::now().time_since_epoch()).count();
  const bool show_road_name = current_ts - lmd_fix_time < 10000; // hide if fix older than 10s
  //str.sprintf("roadName: %s\n", show_road_name ? QString::fromStdString(lmd.getCurrentRoadName()) : "");
  QString str1 = "roadName:";
  str1 += show_road_name ? QString::fromStdString(lmd.getCurrentRoadName()) : "";
  y += height;
  p.drawText(text_x, y, str1);

  static float speedLimit1 = 0.0;
  bool speedLimitValid = lmd.getSpeedLimitValid();
  float speedLimit = lmd.getSpeedLimit();
  float speedLimitAhead = lmd.getSpeedLimitAhead();
  if (speedLimitValid) speedLimit1 = speedLimit;
  bool turnSpeedValid = lmd.getTurnSpeedLimitValid();
  float turnSpeedLimit = lmd.getTurnSpeedLimit();
  str.sprintf("SpeedLimit(%d): %.1f, %.1f, A:%.1f", speedLimitValid, speedLimit, speedLimit1, speedLimitAhead);
  y += height;
  p.drawText(text_x, y, str);
  str.sprintf("TurnSpeed(%d): %.1f", turnSpeedValid, turnSpeedLimit);
  y += height;
  p.drawText(text_x, y, str);

#endif

  p.restore();
}

void AnnotatedCameraWidget::drawDriverState(QPainter &painter, const UIState *s) {
  const UIScene &scene = s->scene;

  painter.save();

  // base icon
  //int x = radius / 2 + (bdr_s * 2) + radius - 20;// (radius + 50) + (radius + 50) * 3;
  //int x = (radius + 50) + (radius + 50) * 3;
  //int x = rightHandDM ? rect().right() - (btn_size - 24) / 2 - (bdr_s * 2) : (btn_size - 24) / 2 + (bdr_s * 2);
  //int y = rect().bottom() - footer_h / 2 - 10 - radius;
  int x = (btn_size - 24) / 2 + (bdr_s * 2);
  int y = rect().bottom() - footer_h / 2;


  float opacity = dmActive ? 0.65f : 0.15f;
  drawIcon(painter, x, y, dm_img, blackColor(0), opacity);

  // circle background
  painter.setOpacity(1.0);
  painter.setPen(Qt::NoPen);
  painter.setBrush(blackColor(70));
  painter.drawEllipse(x - btn_size / 2, y - btn_size / 2, btn_size, btn_size);

  // face
  QPointF face_kpts_draw[std::size(default_face_kpts_3d)];
  float kp;
  for (int i = 0; i < std::size(default_face_kpts_3d); ++i) {
    kp = (scene.face_kpts_draw[i].v[2] - 8) / 120 + 1.0;
    face_kpts_draw[i] = QPointF(scene.face_kpts_draw[i].v[0] * kp + x, scene.face_kpts_draw[i].v[1] * kp + y);
  }

  painter.setPen(QPen(QColor::fromRgbF(1.0, 1.0, 1.0, opacity), 5.2, Qt::SolidLine, Qt::RoundCap));
  painter.drawPolyline(face_kpts_draw, std::size(default_face_kpts_3d));

  // tracking arcs
  const int arc_l = 133;
  const float arc_t_default = 6.7;
  const float arc_t_extend = 12.0;
  //QColor arc_color = QColor::fromRgbF(0.09, 0.945, 0.26, 0.4*(1.0-dm_fade_state)*(s->engaged()));
  QColor arc_color = QColor::fromRgbF(0.545 - 0.445 * s->engaged(),
                                    0.545 + 0.4 * s->engaged(),
                                    0.545 - 0.285 * s->engaged(),
                                    0.4 * (1.0 - dm_fade_state));
 
  float delta_x = -scene.driver_pose_sins[1] * arc_l / 2;
  float delta_y = -scene.driver_pose_sins[0] * arc_l / 2;
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[1] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(std::fmin(x + delta_x, x), y - arc_l / 2, fabs(delta_x), arc_l), (scene.driver_pose_sins[1]>0 ? 90 : -90) * 16, 180 * 16);
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[0] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(x - arc_l / 2, std::fmin(y + delta_y, y), arc_l, fabs(delta_y)), (scene.driver_pose_sins[0]>0 ? 0 : 180) * 16, 180 * 16);

  painter.restore();
}