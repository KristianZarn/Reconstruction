#ifndef REALTIME_RECONSTRUCTION_UTILITY_H
#define REALTIME_RECONSTRUCTION_UTILITY_H

#include <string>

#include <OpenMVS/MVS.h>
#include <theia/sfm/camera_intrinsics_prior.h>
#include <theia/sfm/reconstruction_builder.h>
#include <theia/sfm/reconstruction_estimator.h>

#include "reconstruction/RealtimeReconstructionBuilder.h"

theia::RealtimeReconstructionBuilder::Options SetRealtimeReconstructionBuilderOptions();

theia::CameraIntrinsicsPrior ReadCalibration(const std::string &filename);

bool TheiaToMVS(const theia::Reconstruction &reconstruction,
                const std::string &images_path,
                MVS::Scene &mvs_scene);

#endif //REALTIME_RECONSTRUCTION_UTILITY_H
