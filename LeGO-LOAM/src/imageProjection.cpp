// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// 订阅点云数据及回调函数
// 获取参数服务器定义参数
// 


#include <boost/circular_buffer.hpp>
#include "imageProjection.h"

ImageProjection::ImageProjection(ros::NodeHandle& nh,
                                 Channel<ProjectionOut>& output_channel)
    : _nh(nh),
      _output_channel(output_channel)
{
  // 订阅原始点云，注册cloudHandler回调函数
  _sub_laser_cloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/lidar_points", 1, &ImageProjection::cloudHandler, this);

  // 发布经过投影之后的深度图点云
  _pub_full_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_projected", 1);

  // 发布带距离值的深度图点云，数据与深度图数据一致，但其intensity属性存储距离值
  _pub_full_info_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_info", 1);

  // 发布地面点点云
  _pub_ground_cloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);

  // 发布分割点和经过降采样的地面点
  _pub_segmented_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);

  // 发布分割点点云
  _pub_segmented_cloud_pure =
      nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud_pure", 1);

  // 发布自定义分割点云的信息
  _pub_segmented_cloud_info =
      nh.advertise<cloud_msgs::cloud_info>("/segmented_cloud_info", 1);

  // 发布经过降采样的未分割点
  _pub_outlier_cloud = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud", 1);

  // 获取参数服务器自定义参数
  nh.getParam("/lego_loam/laser/num_vertical_scans", _vertical_scans);  // 垂直线束数
  nh.getParam("/lego_loam/laser/num_horizontal_scans", _horizontal_scans);  // 水平线束数，根据分辨率为0.2计算
  nh.getParam("/lego_loam/laser/vertical_angle_bottom", _ang_bottom);  // 下视角
  float vertical_angle_top;
  nh.getParam("/lego_loam/laser/vertical_angle_top", vertical_angle_top);  // 上视角

  // 计算水平分辨率，弧度表示
  _ang_resolution_X = (M_PI*2) / (_horizontal_scans);

  // 计算竖直分辨率，弧度表示
  _ang_resolution_Y = DEG_TO_RAD*(vertical_angle_top - _ang_bottom) / float(_vertical_scans-1);

  // 下视角，弧度表示，进行了取反操作，得到的角度为正
  _ang_bottom = -( _ang_bottom - 0.1) * DEG_TO_RAD;

  // 参数赋值
  _segment_alpha_X = _ang_resolution_X;
  _segment_alpha_Y = _ang_resolution_Y;

  nh.getParam("/lego_loam/imageProjection/segment_theta", _segment_theta);
  _segment_theta *= DEG_TO_RAD;

  nh.getParam("/lego_loam/imageProjection/segment_valid_point_num",
              _segment_valid_point_num);
  nh.getParam("/lego_loam/imageProjection/segment_valid_line_num",
              _segment_valid_line_num);

  // 水平激光线束ring
  nh.getParam("/lego_loam/laser/ground_scan_index",
              _ground_scan_index);

  // lidar安装角度
  nh.getParam("/lego_loam/laser/sensor_mount_angle",
              _sensor_mount_angle);
  _sensor_mount_angle *= DEG_TO_RAD;

  // 计算一帧点云大小
  const size_t cloud_size = _vertical_scans * _horizontal_scans;

  // 智能指针的reset方法
  // 为PointType类型，PointType = PointXYZI 
  _laser_cloud_in.reset(new pcl::PointCloud<PointType>());
  _full_cloud.reset(new pcl::PointCloud<PointType>());
  _full_info_cloud.reset(new pcl::PointCloud<PointType>());

  _ground_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud_pure.reset(new pcl::PointCloud<PointType>());
  _outlier_cloud.reset(new pcl::PointCloud<PointType>());

  _full_cloud->points.resize(cloud_size);
  _full_info_cloud->points.resize(cloud_size);

}

// 参数、数据重置
void ImageProjection::resetParameters() {
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  PointType nanPoint;
  nanPoint.x = std::numeric_limits<float>::quiet_NaN();  // NaN非数，表示未定义或不可表示的值
  nanPoint.y = std::numeric_limits<float>::quiet_NaN();
  nanPoint.z = std::numeric_limits<float>::quiet_NaN();

  // clear方法，清空点云数据
  _laser_cloud_in->clear();
  _ground_cloud->clear();
  _segmented_cloud->clear();
  _segmented_cloud_pure->clear();
  _outlier_cloud->clear();

  // 重置eigen矩阵的大小
  _range_mat.resize(_vertical_scans, _horizontal_scans);
  _ground_mat.resize(_vertical_scans, _horizontal_scans);
  _label_mat.resize(_vertical_scans, _horizontal_scans);

  // FLT_MAX表示最大的正浮点数
  _range_mat.fill(FLT_MAX);
  _ground_mat.setZero();
  _label_mat.setZero();

  _label_count = 1;

  // 进行无效数据的填充
  std::fill(_full_cloud->points.begin(), _full_cloud->points.end(), nanPoint);
  std::fill(_full_info_cloud->points.begin(), _full_info_cloud->points.end(),
            nanPoint);

  // assign()实现不同数据结构之间的赋值操作
  _seg_msg.startRingIndex.assign(_vertical_scans, 0);
  _seg_msg.endRingIndex.assign(_vertical_scans, 0);

  _seg_msg.segmentedCloudGroundFlag.assign(cloud_size, false);
  _seg_msg.segmentedCloudColInd.assign(cloud_size, 0);
  _seg_msg.segmentedCloudRange.assign(cloud_size, 0);
}

// 订阅原始数据的回调函数
void ImageProjection::cloudHandler(
    const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
  // Reset parameters
  resetParameters();

  // Copy and remove NAN points
  pcl::fromROSMsg(*laserCloudMsg, *_laser_cloud_in);  // 从ros点云格式转为pcl数据格式
  std::vector<int> indices;

  // indices实现索引的映射操作，cloud_out[i] = cloud_in[index[i]]
  pcl::removeNaNFromPointCloud(*_laser_cloud_in, *_laser_cloud_in, indices);
  _seg_msg.header = laserCloudMsg->header;  // copy原始点云的header信息

  // 计算一帧数据的起始角度
  findStartEndAngle();
  // Range image projection
  projectPointCloud();
  // Mark ground points
  groundRemoval();
  // Point cloud segmentation
  cloudSegmentation();
  //publish (optionally)
  publishClouds();
}

// 将原始点云进行range image投影
void ImageProjection::projectPointCloud() {
  // range image projection
  const size_t cloudSize = _laser_cloud_in->points.size();

  // 遍历所有点云
  for (size_t i = 0; i < cloudSize; ++i) {
    PointType thisPoint = _laser_cloud_in->points[i];

    // range计算
    float range = sqrt(thisPoint.x * thisPoint.x +
                       thisPoint.y * thisPoint.y +
                       thisPoint.z * thisPoint.z);

    // find the row and column index in the image for this point
    float verticalAngle = std::asin(thisPoint.z / range);
        //std::atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y));
    // 行index计算，（垂直角度+取正后的下视角）/垂直角分辨率
    int rowIdn = (verticalAngle + _ang_bottom) / _ang_resolution_Y;
    if (rowIdn < 0 || rowIdn >= _vertical_scans) {
      continue;
    }

    // 计算水平角度，后续计算减去π/2，与起始点的处理结果保持一致
    float horizonAngle = std::atan2(thisPoint.x, thisPoint.y);

    // 列index计算
    // 水平角度-π/2得到与起始点计算一致的坐标系，再进行一次取反
    // _ang_resolution_X为水平分辨率，暂定为0.2
    // _horizontal_scans * 0.5，相当于坐标系加上π
    // round():四舍五入取正
    // 整个实现了[-π，π]->[0,1800]的映射，得到列index
    int columnIdn = -round((horizonAngle - M_PI_2) / _ang_resolution_X) + _horizontal_scans * 0.5;

    // 列index修正
    if (columnIdn >= _horizontal_scans){
      columnIdn -= _horizontal_scans;
    }

    if (columnIdn < 0 || columnIdn >= _horizontal_scans){
      continue;
    }

    if (range < 0.1){
      continue;
    }

    // 填充_range_mat
    _range_mat(rowIdn, columnIdn) = range;

    // 修改intensity字段保存index信息
    thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

    // 计算range image的一维index，并存储到容器中
    size_t index = columnIdn + rowIdn * _horizontal_scans;
    _full_cloud->points[index] = thisPoint;
    // the corresponding range of a point is saved as "intensity"
    _full_info_cloud->points[index] = thisPoint;
    _full_info_cloud->points[index].intensity = range;
  }
}

void ImageProjection::findStartEndAngle() {
  // start and end orientation of this cloud
  auto point = _laser_cloud_in->points.front();  // 获取一帧数据头部point
  // 计算角度，取负号是因为lidar扫描旋转的方向和坐标系定义方向不一致
  _seg_msg.startOrientation = -std::atan2(point.y, point.x);

  point = _laser_cloud_in->points.back();  // 获取一帧数据的尾部point
  // 计算角度，对atan2计算的角度加上2π，修正计算结果
  _seg_msg.endOrientation = -std::atan2(point.y, point.x) + 2 * M_PI;

  // end point角度范围调整
  if (_seg_msg.endOrientation - _seg_msg.startOrientation > 3 * M_PI) {
    _seg_msg.endOrientation -= 2 * M_PI;
  } else if (_seg_msg.endOrientation - _seg_msg.startOrientation < M_PI) {
    _seg_msg.endOrientation += 2 * M_PI;
  }

  // 计算角度差
  _seg_msg.orientationDiff =
      _seg_msg.endOrientation - _seg_msg.startOrientation;
}

void ImageProjection::groundRemoval() {
  // _ground_mat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  // 遍历范围：给定ground_scan以下的所有水平point
  for (size_t j = 0; j < _horizontal_scans; ++j) {
    for (size_t i = 0; i < _ground_scan_index; ++i) {
      size_t lowerInd = j + (i)*_horizontal_scans;
      size_t upperInd = j + (i + 1) * _horizontal_scans;

      // intensity字段已被赋值为index信息
      // intensity字段为-1表示该点为无效点，继续下次循环
      if (_full_cloud->points[lowerInd].intensity == -1 ||
          _full_cloud->points[upperInd].intensity == -1) {
        // no info to check, invalid points
        _ground_mat(i, j) = -1;
        continue;
      }

      // intensity字段有效的处理
      // 计算同一列两点的角度
      float dX =
          _full_cloud->points[upperInd].x - _full_cloud->points[lowerInd].x;
      float dY =
          _full_cloud->points[upperInd].y - _full_cloud->points[lowerInd].y;
      float dZ =
          _full_cloud->points[upperInd].z - _full_cloud->points[lowerInd].z;

      float vertical_angle = std::atan2(dZ , sqrt(dX * dX + dY * dY + dZ * dZ));

      // TODO: review this change
      // 若角度小于阈值，则对应两点为地面点
      if ( (vertical_angle - _sensor_mount_angle) <= 10 * DEG_TO_RAD) {
        _ground_mat(i, j) = 1;
        _ground_mat(i + 1, j) = 1;
      }
    }
  }
  // extract ground cloud (_ground_mat == 1)
  // mark entry that doesn't need to label (ground and invalid point) for
  // segmentation note that ground remove is from 0~_N_scan-1, need _range_mat
  // for mark label matrix for the 16th scan

  // label mat初始值为0
  // label_mat中标记地面点和无效点（rang没有进行赋值的点）为-1
  // 标记点不需进行后续的分割
  // range_mat没有进行正确赋值：lidar并不能保证完全产生 H x V个数据
  for (size_t i = 0; i < _vertical_scans; ++i) {
    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_ground_mat(i, j) == 1 ||
          _range_mat(i, j) == FLT_MAX) {
        _label_mat(i, j) = -1;
      }
    }
  }

  // 输出地面点
  for (size_t i = 0; i <= _ground_scan_index; ++i) {
    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_ground_mat(i, j) == 1)
        _ground_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
    }
  }
}

// 点云分割
void ImageProjection::cloudSegmentation() {
  // segmentation process
  // _label_mat为0表示该点非地面点和无效点
  // labelComponents对该点进行BFS聚类处理
  for (size_t i = 0; i < _vertical_scans; ++i)
    for (size_t j = 0; j < _horizontal_scans; ++j)
      if (_label_mat(i, j) == 0) labelComponents(i, j);

  int sizeOfSegCloud = 0;
  // extract segmented cloud for lidar odometry

  for (size_t i = 0; i < _vertical_scans; ++i) {
    _seg_msg.startRingIndex[i] = sizeOfSegCloud - 1 + 5;

    for (size_t j = 0; j < _horizontal_scans; ++j) {

      // 分割点 + 地面点
      if (_label_mat(i, j) > 0 || _ground_mat(i, j) == 1) {
        // outliers that will not be used for optimization (always continue)
        // 保留部分外点（非分割点、非地面点、在ground_scan以上）
        if (_label_mat(i, j) == 999999) {
          if (i > _ground_scan_index && j % 5 == 0) {
            _outlier_cloud->push_back(
                _full_cloud->points[j + i * _horizontal_scans]);
            continue;
          } else {
            continue;
          }
        }
        // majority of ground points are skipped
        // 保留部分地面点
        if (_ground_mat(i, j) == 1) {
          if (j % 5 != 0 && j > 5 && j < _horizontal_scans - 5) continue;
        }
        // mark ground points so they will not be considered as edge features
        // later
        _seg_msg.segmentedCloudGroundFlag[sizeOfSegCloud] =
            (_ground_mat(i, j) == 1);
        // mark the points' column index for marking occlusion later
        _seg_msg.segmentedCloudColInd[sizeOfSegCloud] = j;
        // save range info
        _seg_msg.segmentedCloudRange[sizeOfSegCloud] =
            _range_mat(i, j);
        // save seg cloud
        _segmented_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
        // size of seg cloud
        ++sizeOfSegCloud;
      }
    }

    _seg_msg.endRingIndex[i] = sizeOfSegCloud - 1 - 5;
  }

  // extract segmented cloud for visualization
  // _segmented_cloud_pure只有分割点
  for (size_t i = 0; i < _vertical_scans; ++i) {
    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_label_mat(i, j) > 0 && _label_mat(i, j) != 999999) {
        _segmented_cloud_pure->push_back(
            _full_cloud->points[j + i * _horizontal_scans]);
        _segmented_cloud_pure->points.back().intensity =
            _label_mat(i, j);
      }
    }
  }
}

// edge与surf的判断
// 通过标准的BFS对point进行标记：以(row,col)为中心向外扩散，判断其是否是平面点
void ImageProjection::labelComponents(int row, int col) {

  // 给定角度阈值，大于则为surf
  const float segmentThetaThreshold = tan(_segment_theta);

  // 垂直point计算标志，辅助判断条件
  std::vector<bool> lineCountFlag(_vertical_scans, false);
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  using Coord2D = Eigen::Vector2i;

  // circular_buffer循环缓冲区提供固定大小的buffer，继续存入数据会被覆盖
  boost::circular_buffer<Coord2D> queue(cloud_size);
  boost::circular_buffer<Coord2D> all_pushed(cloud_size);

  queue.push_back({ row,col } );
  all_pushed.push_back({ row,col } );

  // 该点的4个临近点
  const Coord2D neighborIterator[4] = {
      {0, -1}, {-1, 0}, {1, 0}, {0, 1}};

  while (queue.size() > 0) {
    // Pop point
    Coord2D fromInd = queue.front();
    queue.pop_front();

    // Mark popped point
    // _label_count = 1
    _label_mat(fromInd.x(), fromInd.y()) = _label_count;
    // Loop through all the neighboring grids of popped grid

    for (const auto& iter : neighborIterator) {
      // new index
      int thisIndX = fromInd.x() + iter.x();
      int thisIndY = fromInd.y() + iter.y();
      // index should be within the boundary
      if (thisIndX < 0 || thisIndX >= _vertical_scans){
        continue;
      }
      // at range image margin (left or right side)
      if (thisIndY < 0){
        thisIndY = _horizontal_scans - 1;
      }
      if (thisIndY >= _horizontal_scans){
        thisIndY = 0;
      }
      // prevent infinite loop (caused by put already examined point back)
      // 标记判断，防止重复分割
      if (_label_mat(thisIndX, thisIndY) != 0){
        continue;
      }

      // 计算到原点距离，取最大和最小
      float d1 = std::max(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));
      float d2 = std::min(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));

      // 根据点的位置，选择水平/竖直分辨率
      float alpha = (iter.x() == 0) ? _ang_resolution_X : _ang_resolution_Y;
      
      // 计算最短边和最长边之间的角度
      float tang = (d2 * sin(alpha) / (d1 - d2 * cos(alpha)));

      // surf点判断，判断的是周围点
      if (tang > segmentThetaThreshold) {
        queue.push_back( {thisIndX, thisIndY } );

        // _label_mat标记为1
        _label_mat(thisIndX, thisIndY) = _label_count;
        lineCountFlag[thisIndX] = true;

        all_pushed.push_back(  {thisIndX, thisIndY } );
      }
    }
  }

  // check if this segment is valid
  bool feasibleSegment = false;

  // all_pushed加入了大于30个的surf点，分割有效
  if (all_pushed.size() >= 30){
    feasibleSegment = true;
  }

  // all_pushed加入大于给定的5个surf点，同时lineCount大于给定的3个点，分割有效
  else if (all_pushed.size() >= _segment_valid_point_num) {
    int lineCount = 0;
    for (size_t i = 0; i < _vertical_scans; ++i) {
      if (lineCountFlag[i] == true) ++lineCount;
    }
    if (lineCount >= _segment_valid_line_num) feasibleSegment = true;
  }
  // segment is valid, mark these points
  // 每一次有效的分割对应一个label_count，赋值到_label_mat中
  if (feasibleSegment == true) {
    ++_label_count;
  } 

  // 分割无效，所有点标记为999999
  else {  // segment is invalid, mark these points
    for (size_t i = 0; i < all_pushed.size(); ++i) {
      _label_mat(all_pushed[i].x(), all_pushed[i].y()) = 999999;
    }
  }
}

void ImageProjection::publishClouds() {

  sensor_msgs::PointCloud2 temp;
  temp.header.stamp = _seg_msg.header.stamp;
  temp.header.frame_id = "base_link";

  auto PublishCloud = [](ros::Publisher& pub, sensor_msgs::PointCloud2& temp,
                          const pcl::PointCloud<PointType>::Ptr& cloud) {
    if (pub.getNumSubscribers() != 0) {
      pcl::toROSMsg(*cloud, temp);
      pub.publish(temp);
    }
  };

  PublishCloud(_pub_outlier_cloud, temp, _outlier_cloud);
  PublishCloud(_pub_segmented_cloud, temp, _segmented_cloud);
  PublishCloud(_pub_full_cloud, temp, _full_cloud);
  PublishCloud(_pub_ground_cloud, temp, _ground_cloud);
  PublishCloud(_pub_segmented_cloud_pure, temp, _segmented_cloud_pure);
  PublishCloud(_pub_full_info_cloud, temp, _full_info_cloud);

  if (_pub_segmented_cloud_info.getNumSubscribers() != 0) {
    _pub_segmented_cloud_info.publish(_seg_msg);
  }

  //--------------------
  ProjectionOut out;
  out.outlier_cloud.reset(new pcl::PointCloud<PointType>());
  out.segmented_cloud.reset(new pcl::PointCloud<PointType>());

  std::swap( out.seg_msg, _seg_msg);
  std::swap(out.outlier_cloud, _outlier_cloud);
  std::swap(out.segmented_cloud, _segmented_cloud);

  _output_channel.send( std::move(out) );

}


