#include "preprocess.h"

#include <cmath>
#include <cstring>

#define RETURN0 0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
    : lidar_type(AVIA),
      point_filter_num(1),
      blind(0.01),
      blind_for_human_pilots(1.5),
      feature_enabled(0) {
  inf_bound         = 10;
  N_SCANS           = 6;
  SCAN_RATE         = 10;
  group_size        = 8;
  disA              = 0.01;
  disA              = 0.1;  // B?
  p2l_ratio         = 225;
  limit_maxmid      = 6.25;
  limit_midmin      = 6.25;
  limit_maxmin      = 3.24;
  jump_up_limit     = 170.0;
  jump_down_limit   = 8.0;
  cos160            = 160.0;
  edgea             = 2;
  edgeb             = 0.1;
  smallp_intersect  = 172.5;
  smallp_ratio      = 1.2;
  given_offset_time = false;

  jump_up_limit    = std::cos(jump_up_limit / 180 * M_PI);
  jump_down_limit  = std::cos(jump_down_limit / 180 * M_PI);
  cos160           = std::cos(cos160 / 180 * M_PI);
  smallp_intersect = std::cos(smallp_intersect / 180 * M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num) {
  feature_enabled  = feat_en;
  lidar_type       = lid_type;
  blind            = bld;
  point_filter_num = pfilt_num;
}

// ---------------------------------------------------------------------------
// Manual PointCloud2 field parsing helpers
// ---------------------------------------------------------------------------

const sensor_msgs::msg::PointField* Preprocess::findField(
    const sensor_msgs::msg::PointCloud2 &msg, const std::string &name) {
  for (const auto &f : msg.fields) {
    if (f.name == name) return &f;
  }
  return nullptr;
}

double Preprocess::readFieldValue(const uint8_t *point_data,
                                  const sensor_msgs::msg::PointField &field) {
  const uint8_t *ptr = point_data + field.offset;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::UINT8:
      return static_cast<double>(*ptr);
    case sensor_msgs::msg::PointField::UINT16: {
      uint16_t v;
      std::memcpy(&v, ptr, sizeof(v));
      return static_cast<double>(v);
    }
    case sensor_msgs::msg::PointField::UINT32: {
      uint32_t v;
      std::memcpy(&v, ptr, sizeof(v));
      return static_cast<double>(v);
    }
    case sensor_msgs::msg::PointField::INT32: {
      int32_t v;
      std::memcpy(&v, ptr, sizeof(v));
      return static_cast<double>(v);
    }
    case sensor_msgs::msg::PointField::FLOAT32: {
      float v;
      std::memcpy(&v, ptr, sizeof(v));
      return static_cast<double>(v);
    }
    case sensor_msgs::msg::PointField::FLOAT64: {
      double v;
      std::memcpy(&v, ptr, sizeof(v));
      return v;
    }
    default:
      return 0.0;
  }
}

void Preprocess::parseOusterCloud(const sensor_msgs::msg::PointCloud2 &msg,
                                  std::vector<OusterPoint> &out_points) {
  // Look up the mandatory xyz fields
  const auto *fx = findField(msg, "x");
  const auto *fy = findField(msg, "y");
  const auto *fz = findField(msg, "z");
  if (!fx || !fy || !fz) {
    RCLCPP_ERROR(rclcpp::get_logger("Preprocess"),
                 "PointCloud2 missing x/y/z fields!");
    return;
  }

  // Look up configurable fields (may be nullptr if not present)
  const auto *f_time      = findField(msg, ouster_fields.time_field);
  const auto *f_ring      = findField(msg, ouster_fields.ring_field);
  const auto *f_intensity = findField(msg, ouster_fields.intensity_field);

  // Log once which fields were found/not found
  static bool logged_fields = false;
  if (!logged_fields) {
    auto logger = rclcpp::get_logger("Preprocess");
    RCLCPP_INFO(logger, "Ouster field mapping: time='%s'%s, ring='%s'%s, intensity='%s'%s",
                ouster_fields.time_field.c_str(),   f_time      ? " [OK]" : " [NOT FOUND]",
                ouster_fields.ring_field.c_str(),    f_ring      ? " [OK]" : " [NOT FOUND]",
                ouster_fields.intensity_field.c_str(), f_intensity ? " [OK]" : " [NOT FOUND]");
    logged_fields = true;
  }

  const size_t n_points = msg.width * msg.height;
  out_points.resize(n_points);

  for (size_t i = 0; i < n_points; ++i) {
    const uint8_t *ptr = msg.data.data() + i * msg.point_step;

    auto &op = out_points[i];
    // xyz are always float32
    std::memcpy(&op.x, ptr + fx->offset, sizeof(float));
    std::memcpy(&op.y, ptr + fy->offset, sizeof(float));
    std::memcpy(&op.z, ptr + fz->offset, sizeof(float));

    op.intensity = f_intensity ? static_cast<float>(readFieldValue(ptr, *f_intensity)) : 0.0f;
    op.t         = f_time      ? static_cast<uint32_t>(readFieldValue(ptr, *f_time))   : 0u;
    op.ring      = f_ring      ? static_cast<uint16_t>(readFieldValue(ptr, *f_ring))   : 0u;
  }
}

// ---------------------------------------------------------------------------

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
void Preprocess::process(const livox_ros_driver::CustomMsg &msg, PointCloudXYZI::Ptr &pcl_out) {
  avia_handler(msg);
  *pcl_out = pl_surf;
}
#endif

void Preprocess::process(const sensor_msgs::msg::PointCloud2 &msg, PointCloudXYZI::Ptr &pcl_out) {
  switch (time_unit) {
    case SEC:
      time_unit_scale = 1.e3f;
      break;
    case MS:
      time_unit_scale = 1.f;
      break;
    case US:
      time_unit_scale = 1.e-3f;
      break;
    case NS:
      time_unit_scale = 1.e-6f;
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  switch (lidar_type) {
    case OUST64:
      oust64_handler(msg);
      break;
    case KMOUST64:
      kmoust64_handler(msg);
      break;

    case VELO16:
      velodyne_handler(msg);
      break;

    default:
      RCLCPP_FATAL(rclcpp::get_logger("Preprocess"), "Error LiDAR Type");
      break;
  }
  *pcl_out = pl_surf;
}

bool Preprocess::is_from_pilot_zone(const float &pt_x,
                                    const float &pt_y,
                                    const float &pt_z,
                                    const std::string mode) {
  if (mode == "ouster") {
    if (pt_y < 0.6 && pt_y > -0.6 && pt_x < blind_for_human_pilots && pt_x > 0) {
      return true;
    } else {
      return false;
    }
  } else if (mode == "velodyne") {
    if (pt_y < 0.6 && pt_y > -0.6 && pt_x < 0 && pt_x > -blind_for_human_pilots) {
      return true;
    } else {
      return false;
    }
  } else {
    throw std::invalid_argument("Invalid mode is given");
  }
}

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
void Preprocess::avia_handler(const livox_ros_driver::CustomMsg &msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pl_from_pilots.clear();

  double t1  = omp_get_wtime();
  int plsize = msg.point_num;

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for (int i = 0; i < N_SCANS; i++) {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;

  if (feature_enabled) {
    for (uint i = 1; i < plsize; i++) {
      if ((msg.points[i].line < N_SCANS) &&
          ((msg.points[i].tag & 0x30) == 0x10 || (msg.points[i].tag & 0x30) == 0x00)) {
        pl_full[i].x         = msg.points[i].x;
        pl_full[i].y         = msg.points[i].y;
        pl_full[i].z         = msg.points[i].z;
        pl_full[i].intensity = msg.points[i].reflectivity;
        pl_full[i].curvature =
            msg.points[i].offset_time /
            static_cast<float>(1000000);

        bool is_new = false;
        if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) ||
            (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
            (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7)) {
          pl_buff[msg.points[i].line].push_back(pl_full[i]);
        }
      }
    }
    static int count   = 0;
    static double time = 0.0;
    count++;
    double t0 = omp_get_wtime();
    for (int j = 0; j < N_SCANS; j++) {
      if (pl_buff[j].size() <= 5) continue;
      pcl::PointCloud<PointType> &pl = pl_buff[j];
      plsize                         = pl.size();
      std::vector<orgtype> &types    = typess[j];
      types.clear();
      types.resize(plsize);
      plsize--;
      for (uint i = 0; i < plsize; i++) {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx             = pl[i].x - pl[i + 1].x;
        vy             = pl[i].y - pl[i + 1].y;
        vz             = pl[i].z - pl[i + 1].z;
        types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);
      }
      types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
      give_feature(pl, types);
    }
    time += omp_get_wtime() - t0;
    ROS_DEBUG("Feature extraction time: %lf \n", time / count);
  } else {
    for (uint i = 1; i < plsize; i++) {
      if ((msg.points[i].line < N_SCANS) &&
          ((msg.points[i].tag & 0x30) == 0x10 || (msg.points[i].tag & 0x30) == 0x00)) {
        valid_num++;
        if (valid_num % point_filter_num == 0) {
          pl_full[i].x         = msg.points[i].x;
          pl_full[i].y         = msg.points[i].y;
          pl_full[i].z         = msg.points[i].z;
          pl_full[i].intensity = msg.points[i].reflectivity;
          pl_full[i].curvature =
              msg.points[i].offset_time /
              static_cast<float>(1000000);

          if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) ||
              (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
              (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7) &&
                  (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y +
                       pl_full[i].z * pl_full[i].z >
                   (blind * blind))) {
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }
}
#endif

// ---------------------------------------------------------------------------
// Ouster handler — now uses manual field parsing with configurable names
// ---------------------------------------------------------------------------
void Preprocess::oust64_handler(const sensor_msgs::msg::PointCloud2 &msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pl_from_pilots.clear();

  // Parse using configurable field names instead of pcl::fromROSMsg
  std::vector<OusterPoint> pl_orig;
  parseOusterCloud(msg, pl_orig);

  const auto plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  if (feature_enabled) {
    for (int i = 0; i < N_SCANS; i++) {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (size_t i = 0; i < plsize; i++) {
      double range = pl_orig[i].x * pl_orig[i].x +
                     pl_orig[i].y * pl_orig[i].y +
                     pl_orig[i].z * pl_orig[i].z;
      if (range < (blind * blind)) continue;

      PointType added_pt;
      added_pt.x         = pl_orig[i].x;
      added_pt.y         = pl_orig[i].y;
      added_pt.z         = pl_orig[i].z;
      added_pt.intensity = pl_orig[i].intensity;
      added_pt.normal_x  = 0;
      added_pt.normal_y  = 0;
      added_pt.normal_z  = 0;

      added_pt.curvature = pl_orig[i].t * time_unit_scale;
      if (pl_orig[i].ring < N_SCANS) {
        pl_buff[pl_orig[i].ring].push_back(added_pt);
      }
    }

    for (int j = 0; j < N_SCANS; j++) {
      PointCloudXYZI &pl          = pl_buff[j];
      auto linesize               = pl.size();
      std::vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++) {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx             = pl[i].x - pl[i + 1].x;
        vy             = pl[i].y - pl[i + 1].y;
        vz             = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range =
          sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  } else {
    for (size_t i = 0; i < pl_orig.size(); i++) {
      if (i % point_filter_num != 0) continue;

      double range = pl_orig[i].x * pl_orig[i].x +
                     pl_orig[i].y * pl_orig[i].y +
                     pl_orig[i].z * pl_orig[i].z;

      if (range < (blind * blind)) continue;

      PointType added_pt;
      added_pt.x         = pl_orig[i].x;
      added_pt.y         = pl_orig[i].y;
      added_pt.z         = pl_orig[i].z;
      added_pt.intensity = pl_orig[i].intensity;
      added_pt.normal_x  = 0;
      added_pt.normal_y  = 0;
      added_pt.normal_z  = 0;
      added_pt.curvature = pl_orig[i].t * time_unit_scale;  // curvature unit: ms

      pl_surf.points.push_back(added_pt);
    }
  }
}

// ---------------------------------------------------------------------------
// Kimera-Multi Ouster handler — also updated for manual parsing
// ---------------------------------------------------------------------------
void Preprocess::kmoust64_handler(const sensor_msgs::msg::PointCloud2 &msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pl_from_pilots.clear();

  std::vector<OusterPoint> pl_orig;
  parseOusterCloud(msg, pl_orig);

  // Subtract the first point's timestamp offset
  if (!pl_orig.empty()) {
    const uint32_t time_offset = pl_orig[0].t;
    for (size_t i = 0; i < pl_orig.size(); i++) {
      pl_orig[i].t -= time_offset;
    }
  }

  size_t plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  if (feature_enabled) {
    for (int i = 0; i < N_SCANS; i++) {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (size_t i = 0; i < plsize; i++) {
      double range = pl_orig[i].x * pl_orig[i].x +
                     pl_orig[i].y * pl_orig[i].y +
                     pl_orig[i].z * pl_orig[i].z;
      if (range < (blind * blind) ||
          is_from_pilot_zone(pl_orig[i].x, pl_orig[i].y, pl_orig[i].z, "ouster"))
        continue;

      PointType added_pt;
      added_pt.x         = pl_orig[i].x;
      added_pt.y         = pl_orig[i].y;
      added_pt.z         = pl_orig[i].z;
      added_pt.intensity = pl_orig[i].intensity;
      added_pt.normal_x  = 0;
      added_pt.normal_y  = 0;
      added_pt.normal_z  = 0;

      added_pt.curvature = pl_orig[i].t * time_unit_scale;
      if (pl_orig[i].ring < N_SCANS) {
        pl_buff[pl_orig[i].ring].push_back(added_pt);
      }
    }

    for (int j = 0; j < N_SCANS; j++) {
      PointCloudXYZI &pl          = pl_buff[j];
      auto linesize               = pl.size();
      std::vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++) {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx             = pl[i].x - pl[i + 1].x;
        vy             = pl[i].y - pl[i + 1].y;
        vz             = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range =
          sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  } else {
    for (size_t i = 0; i < pl_orig.size(); i++) {
      if (i % point_filter_num != 0) continue;

      double range = pl_orig[i].x * pl_orig[i].x +
                     pl_orig[i].y * pl_orig[i].y +
                     pl_orig[i].z * pl_orig[i].z;

      if (range < (blind * blind)) continue;

      PointType added_pt;
      added_pt.x         = pl_orig[i].x;
      added_pt.y         = pl_orig[i].y;
      added_pt.z         = pl_orig[i].z;
      added_pt.intensity = pl_orig[i].intensity;
      added_pt.normal_x  = 0;
      added_pt.normal_y  = 0;
      added_pt.normal_z  = 0;
      added_pt.curvature = pl_orig[i].t * time_unit_scale;

      pl_surf.points.push_back(added_pt);
    }
  }
}

void Preprocess::velodyne_handler(const sensor_msgs::msg::PointCloud2 &msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pl_from_pilots.clear();

  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0) return;
  pl_surf.reserve(plsize);

  /*** These variables only works when no point timestamps given ***/
  double omega_l = 0.361 * SCAN_RATE;  // scan angular velocity
  std::vector<bool> is_first(N_SCANS, true);
  std::vector<double> yaw_fp(N_SCANS, 0.0);
  std::vector<float> yaw_last(N_SCANS, 0.0);
  std::vector<float> time_last(N_SCANS, 0.0);
  /*****************************************************************/
  if (pl_orig.points[plsize - 1].time > 0) {
    given_offset_time = true;
  } else {
    given_offset_time = false;
  }

  if (feature_enabled) {
    for (int i = 0; i < N_SCANS; i++) {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (int i = 0; i < plsize; i++) {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      int layer         = pl_orig.points[i].ring;
      if (layer >= N_SCANS) continue;
      added_pt.x         = pl_orig.points[i].x;
      added_pt.y         = pl_orig.points[i].y;
      added_pt.z         = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time * time_unit_scale;

      if (!given_offset_time) {
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
        if (is_first[layer]) {
          yaw_fp[layer]      = yaw_angle;
          is_first[layer]    = false;
          added_pt.curvature = 0.0;
          yaw_last[layer]    = yaw_angle;
          time_last[layer]   = added_pt.curvature;
          continue;
        }

        if (yaw_angle <= yaw_fp[layer]) {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        } else {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;
        yaw_last[layer]  = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      pl_buff[layer].points.push_back(added_pt);
    }

    for (int j = 0; j < N_SCANS; j++) {
      PointCloudXYZI &pl = pl_buff[j];
      auto linesize      = pl.size();
      if (linesize < 2) continue;
      std::vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++) {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx             = pl[i].x - pl[i + 1].x;
        vy             = pl[i].y - pl[i + 1].y;
        vz             = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range =
          sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  } else {
    for (int i = 0; i < plsize; i++) {
      PointType added_pt;

      added_pt.normal_x  = 0;
      added_pt.normal_y  = 0;
      added_pt.normal_z  = 0;
      added_pt.x         = pl_orig.points[i].x;
      added_pt.y         = pl_orig.points[i].y;
      added_pt.z         = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time * time_unit_scale;

      if (!given_offset_time) {
        int layer        = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer]) {
          yaw_fp[layer]      = yaw_angle;
          is_first[layer]    = false;
          added_pt.curvature = 0.0;
          yaw_last[layer]    = yaw_angle;
          time_last[layer]   = added_pt.curvature;
          continue;
        }

        if (yaw_angle <= yaw_fp[layer]) {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        } else {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer]  = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      if (i % point_filter_num == 0) {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z >
            (blind * blind)) {
          if (is_from_pilot_zone(added_pt.x, added_pt.y, added_pt.z)) {
            pl_from_pilots.push_back(added_pt);
            continue;
          }
          pl_surf.points.push_back(added_pt);
        }
      }
    }
  }
}

void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, std::vector<orgtype> &types) {
  auto plsize = pl.size();
  size_t plsize2;
  if (plsize == 0) {
    return;
  }
  uint head = 0;

  while (types[head].range < blind) {
    head++;
  }

  plsize2 =
      (plsize > static_cast<size_t>(group_size)) ? (plsize - static_cast<size_t>(group_size)) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex     = 0;
  int last_state = 0;
  int plane_type;

  for (uint i = head; i < plsize2; i++) {
    if (types[i].range < blind) {
      continue;
    }

    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    if (plane_type == 1) {
      for (uint j = i; j <= i_nex; j++) {
        if (j != i && j != i_nex) {
          types[j].ftype = Real_Plane;
        } else {
          types[j].ftype = Poss_Plane;
        }
      }

      if (last_state == 1 && last_direct.norm() > 0.1) {
        double mod = last_direct.transpose() * curr_direct;
        if (mod > -0.707 && mod < 0.707) {
          types[i].ftype = Edge_Plane;
        } else {
          types[i].ftype = Real_Plane;
        }
      }

      i          = i_nex - 1;
      last_state = 1;
    } else {
      i          = i_nex;
      last_state = 0;
    }

    last_direct = curr_direct;
  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for (uint i = head + 3; i < plsize2; i++) {
    if (types[i].range < blind || types[i].ftype >= Real_Plane) {
      continue;
    }

    if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16) {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];

    for (int j = 0; j < 2; j++) {
      int m = -1;
      if (j == 1) {
        m = 1;
      }

      if (types[i + m].range < blind) {
        if (types[i].range > inf_bound) {
          types[i].edj[j] = Nr_inf;
        } else {
          types[i].edj[j] = Nr_blind;
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
      vecs[j] = vecs[j] - vec_a;

      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if (types[i].angle[j] < jump_up_limit) {
        types[i].edj[j] = Nr_180;
      } else if (types[i].angle[j] > jump_down_limit) {
        types[i].edj[j] = Nr_zero;
      }
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 &&
        types[i].dista > 4 * types[i - 1].dista) {
      if (types[i].intersect > cos160) {
        if (edge_jump_judge(pl, types, i, Prev)) {
          types[i].ftype = Edge_Jump;
        }
      }
    } else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor &&
               types[i - 1].dista > 0.0225 && types[i - 1].dista > 4 * types[i].dista) {
      if (types[i].intersect > cos160) {
        if (edge_jump_judge(pl, types, i, Next)) {
          types[i].ftype = Edge_Jump;
        }
      }
    } else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf) {
      if (edge_jump_judge(pl, types, i, Prev)) {
        types[i].ftype = Edge_Jump;
      }
    } else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor) {
      if (edge_jump_judge(pl, types, i, Next)) {
        types[i].ftype = Edge_Jump;
      }

    } else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor) {
      if (types[i].ftype == Nor) {
        types[i].ftype = Wire;
      }
    }
  }

  plsize2 = plsize - 1;
  double ratio;
  for (uint i = head + 1; i < plsize2; i++) {
    if (types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind) {
      continue;
    }

    if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8) {
      continue;
    }

    if (types[i].ftype == Nor) {
      if (types[i - 1].dista > types[i].dista) {
        ratio = types[i - 1].dista / types[i].dista;
      } else {
        ratio = types[i].dista / types[i - 1].dista;
      }

      if (types[i].intersect < smallp_intersect && ratio < smallp_ratio) {
        if (types[i - 1].ftype == Nor) {
          types[i - 1].ftype = Real_Plane;
        }
        if (types[i + 1].ftype == Nor) {
          types[i + 1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;
  for (uint j = head; j < plsize; j++) {
    if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane) {
      if (last_surface == -1) {
        last_surface = j;
      }

      if (j == uint(last_surface + point_filter_num - 1)) {
        PointType ap;
        ap.x         = pl[j].x;
        ap.y         = pl[j].y;
        ap.z         = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    } else {
      if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane) {
        pl_corn.push_back(pl[j]);
      }
      if (last_surface != -1) {
        PointType ap;
        for (uint k = last_surface; k < j; k++) {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j - last_surface);
        ap.y /= (j - last_surface);
        ap.z /= (j - last_surface);
        ap.intensity /= (j - last_surface);
        ap.curvature /= (j - last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct) {
  pl.height = 1;
  pl.width  = pl.size();
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp    = ct;
}

int Preprocess::plane_judge(const PointCloudXYZI &pl,
                            std::vector<orgtype> &types,
                            uint i_cur,
                            uint &i_nex,
                            Eigen::Vector3d &curr_direct) {
  double group_dis = disA * types[i_cur].range + disB;
  group_dis        = group_dis * group_dis;

  double two_dis;
  std::vector<double> disarr;
  disarr.reserve(20);

  for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++) {
    if (types[i_nex].range < blind) {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }

  for (;;) {
    if ((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if (types[i_nex].range < blind) {
      curr_direct.setZero();
      return 2;
    }
    vx      = pl[i_nex].x - pl[i_cur].x;
    vy      = pl[i_nex].y - pl[i_cur].y;
    vz      = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx * vx + vy * vy + vz * vz;
    if (two_dis >= group_dis) {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0;
  double v1[3], v2[3];
  for (uint j = i_cur + 1; j < i_nex; j++) {
    if ((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1] * vz - vy * v1[2];
    v2[1] = v1[2] * vx - v1[0] * vz;
    v2[2] = v1[0] * vy - vx * v1[1];

    double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
    if (lw > leng_wid) {
      leng_wid = lw;
    }
  }

  if ((two_dis * two_dis / leng_wid) < p2l_ratio) {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  for (uint j = 0; j < disarrsize - 1; j++) {
    for (uint k = j + 1; k < disarrsize; k++) {
      if (disarr[j] < disarr[k]) {
        leng_wid  = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if (disarr[disarr.size() - 2] < 1e-16) {
    curr_direct.setZero();
    return 0;
  }

  if (lidar_type == AVIA) {
    double dismax_mid = disarr[0] / disarr[disarrsize / 2];
    double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

    if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin) {
      curr_direct.setZero();
      return 0;
    }
  } else {
    double dismax_min = disarr[0] / disarr[disarrsize - 2];
    if (dismax_min >= limit_maxmin) {
      curr_direct.setZero();
      return 0;
    }
  }

  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl,
                                 std::vector<orgtype> &types,
                                 uint i,
                                 Surround nor_dir) {
  if (nor_dir == 0) {
    if (types[i - 1].range < blind || types[i - 2].range < blind) {
      return false;
    }
  } else if (nor_dir == 1) {
    if (types[i + 1].range < blind || types[i + 2].range < blind) {
      return false;
    }
  }
  double d1 = types[i + nor_dir - 1].dista;
  double d2 = types[i + 3 * nor_dir - 2].dista;
  double d;

  if (d1 < d2) {
    d  = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  if (d1 > edgea * d2 || (d1 - d2) > edgeb) {
    return false;
  }

  return true;
}