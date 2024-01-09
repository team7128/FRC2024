#include "ApriltagInfo.h"

ApriltagInfo apriltagPositions[] = {
	{  1, 593.68_in,   9.68_in, 53.38_in, 120_deg }, // Blue source (closer to centerline)
	{  2, 637.21_in,  34.79_in, 53.38_in, 120_deg }, // Blue source (closer to red driver stations)
	{  3, 652.73_in, 196.17_in, 57.13_in, 180_deg }, // Red speaker (off center)
	{  4, 652.73_in, 218.42_in, 57.13_in, 180_deg }, // Red speaker (centered)
	{  5, 578.77_in, 323.00_in, 53.38_in, 270_deg }, // Red amp
	{  6,  72.50_in, 323.00_in, 53.38_in, 270_deg }, // Blue amp
	{  7,  -1.50_in, 218.42_in, 57.13_in,   0_deg }, // Blue speaker (centered)
	{  8,  -1.50_in, 196.17_in, 57.13_in,   0_deg }, // Blue speaker (off center)
	{  9,  14.02_in,  34.79_in, 53.38_in,  60_deg }, // Red source (closer to blue driver station)
	{ 10,  57.54_in,   9.68_in, 53.38_in,  60_deg }, // Red source (closer to centerline)
	{ 11, 468.69_in, 146.19_in, 52.00_in, 300_deg }, // Red left stage
	{ 12, 468.69_in, 177.10_in, 52.00_in,  60_deg }, // Red right stage
	{ 13, 441.74_in, 161.62_in, 52.00_in, 180_deg }, // Red center stage
	{ 14, 209.48_in, 161.62_in, 52.00_in,   0_deg }, // Blue center stage
	{ 15, 182.73_in, 177.10_in, 52.00_in, 120_deg }, // Blue left stage
	{ 16, 182.73_in, 146.19_in, 52.00_in, 240_deg }  // Blue right stage
};