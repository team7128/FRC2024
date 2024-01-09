#pragma once

#include <units/angle.h>
#include <units/length.h>

/**
 * Struct to store apriltag position on the field
*/
struct ApriltagInfo
{
	/**
	 * Apriltag ID number
	 * For 2024 Crescendo, IDs range from 1 to 16
	*/
	int id;

	/**
	 * Field position of apriltag stored as 3D coordinate in meters
	 * Field origin is in the bottom left (red alliance source, at the projected intersection of the two field walls)
	 * X is towards red alliance driver stations
	 * Y is towards blue alliance amp
	 * Z is vertical distance 
	*/
	struct {
		units::meter_t x, y, z;
	};

	/**
	 * Apriltag angle in degrees
	 * 0 degrees is facing right, from blue alliance driver stations to red alliance
	*/
	units::degree_t angle;
};

/**
 * Array to store the position of each apriltag on the field
 * Defined in ApriltagInfo.cpp
*/
extern ApriltagInfo apriltagPositions[];