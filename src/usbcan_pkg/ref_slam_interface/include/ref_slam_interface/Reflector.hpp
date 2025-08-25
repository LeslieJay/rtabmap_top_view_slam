/*
 * @Author: LiFang6606397
 * @Date: 2024-01-08 14:55:10
 * @LastEditors: LiFang6606397
 * @LastEditTime: 2024-07-04 10:15:29
 * @FilePath: /dev_ws/src/ref_slam_interface/include/ref_slam_interface/Reflector.hpp
 * @Description: 
 * 
 * Copyright (c) 2024 by LiFang6606397, All Rights Reserved. 
 */
#ifndef REF_SLAM_INTERFACE__REFLECTOR_HPP_
#define REF_SLAM_INTERFACE__REFLECTOR_HPP_

#include <string>
#include <memory>
#include <vector>
#include <Eigen/Core>

namespace ref_slam_interface
{

class Reflector
{
public:
  Reflector();
  ~Reflector();

  Reflector(int id, float x, float y);

  void SetPoints(const std::vector<Eigen::Vector3f>& pcl);

  /** Interfaces */
  const std::string& GetName() const;
  void SetName(const std::string& name);

  int GetId() const;
  void SetId(int id);

  float GetX() const;
  void SetX(float x);

  float GetY() const;
  void SetY(float y);

  float GetRadius() const;
  void SetRadius(float radius);

  int GetNumBeDetected() const;
  void SetNumBeDetected(int num_be_detected);

  const std::vector<Eigen::Vector3f>& GetPoints() const;
  void SetPoints(std::vector<Eigen::Vector3f> points);

  Eigen::Vector2d GetCor() const;


private:
    std::string name_;
    int id_;
    float x_;
    float y_;
    float radius_;
    int num_be_detected_; 
    std::vector<Eigen::Vector3f> points_;
};


}//namespace ref_slam_interface

#endif // REF_SLAM_INTERFACE__REFLECTOR_HPP_