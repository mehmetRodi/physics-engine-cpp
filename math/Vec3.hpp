#pragma once

struct Vec3 {
  float x, y, z;

  // Constructors
  Vec3();
  Vec3(float x, float y, float z);

  // Basic operations
  Vec3 operator*(float scalar) const;
  Vec3 operator+(const Vec3& other) const;
  Vec3 operator-(const Vec3& other) const;

  // Physics essentials
  float dot(const Vec3& other) const;
  Vec3 cross(const Vec3& other) const;
  float lengthSq() const; // faster (no square root!)
  float length() const;
  
};
