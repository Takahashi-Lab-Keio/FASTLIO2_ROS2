#include "icp_localizer.h"

#include <algorithm>
#include <cmath>
#include <pcl/common/point_tests.h>

ICPLocalizer::ICPLocalizer(const ICPConfig &config) : m_config(config)
{
    m_refine_inp.reset(new CloudType);
    m_refine_tgt.reset(new CloudType);
    m_rough_inp.reset(new CloudType);
    m_rough_tgt.reset(new CloudType);
    m_rough_icp.setMaxCorrespondenceDistance(
        m_config.rough_max_correspondence_distance);
    m_refine_icp.setMaxCorrespondenceDistance(
        m_config.refine_max_correspondence_distance);
}
bool ICPLocalizer::loadMap(const std::string &path)
{
    if (!std::filesystem::exists(path))
    {
        std::cerr << "Map file not found: " << path << std::endl;
        return false;
    }
    pcl::PCDReader reader;
    CloudType::Ptr cloud(new CloudType);
    if (reader.read(path, *cloud) < 0 || cloud->empty() ||
        !std::all_of(cloud->begin(), cloud->end(), [](const PointType &point) {
            return pcl::isFinite(point) && std::isfinite(point.intensity);
        }))
    {
        std::cerr << "Map file is unreadable or empty: " << path << std::endl;
        return false;
    }
    CloudType::Ptr refine_target(new CloudType);
    CloudType::Ptr rough_target(new CloudType);
    if (m_config.refine_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_map_resolution, m_config.refine_map_resolution, m_config.refine_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*refine_target);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *refine_target);
    }

    if (m_config.rough_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_map_resolution, m_config.rough_map_resolution, m_config.rough_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*rough_target);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *rough_target);
    }
    if (refine_target->empty() || rough_target->empty())
    {
        std::cerr << "Map is empty after voxel filtering: " << path << std::endl;
        return false;
    }
    m_refine_tgt = refine_target;
    m_rough_tgt = rough_target;
    m_pcd_path = path;
    return true;
}
void ICPLocalizer::setInput(const CloudType::Ptr &cloud)
{
    if (!cloud || cloud->empty() ||
        !std::all_of(cloud->begin(), cloud->end(), [](const PointType &point) {
            return pcl::isFinite(point) && std::isfinite(point.intensity);
        }))
    {
        m_refine_inp->clear();
        m_rough_inp->clear();
        return;
    }
    if (m_config.refine_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_scan_resolution, m_config.refine_scan_resolution, m_config.refine_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_inp);
    }

    if (m_config.rough_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_scan_resolution, m_config.rough_scan_resolution, m_config.rough_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_inp);
    }
}

bool ICPLocalizer::align(M4F &guess)
{
    CloudType::Ptr aligned_cloud(new CloudType);
    if (m_refine_tgt->empty() || m_rough_tgt->empty() ||
        m_refine_inp->empty() || m_rough_inp->empty() || !guess.allFinite())
        return false;
    m_rough_icp.setMaximumIterations(m_config.rough_max_iteration);
    m_rough_icp.setInputSource(m_rough_inp);
    m_rough_icp.setInputTarget(m_rough_tgt);
    m_rough_icp.align(*aligned_cloud, guess);
    const double rough_score = m_rough_icp.getFitnessScore();
    const M4F rough_transform = m_rough_icp.getFinalTransformation();
    if (!m_rough_icp.hasConverged() || !std::isfinite(rough_score) ||
        !rough_transform.allFinite() || rough_score > m_config.rough_score_thresh)
        return false;
    m_refine_icp.setMaximumIterations(m_config.refine_max_iteration);
    m_refine_icp.setInputSource(m_refine_inp);
    m_refine_icp.setInputTarget(m_refine_tgt);
    m_refine_icp.align(*aligned_cloud, rough_transform);
    const double refine_score = m_refine_icp.getFitnessScore();
    const M4F refine_transform = m_refine_icp.getFinalTransformation();
    if (!m_refine_icp.hasConverged() || !std::isfinite(refine_score) ||
        !refine_transform.allFinite() || refine_score > m_config.refine_score_thresh)
        return false;

    const M3F raw_rotation = refine_transform.block<3, 3>(0, 0);
    const float determinant = raw_rotation.determinant();
    const M3F orthogonality_error =
        raw_rotation.transpose() * raw_rotation - M3F::Identity();
    if (!std::isfinite(determinant) || determinant <= 0.0F ||
        std::abs(determinant - 1.0F) > 0.05F ||
        orthogonality_error.norm() > 0.05F)
        return false;

    Eigen::Quaternionf rotation(raw_rotation);
    if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-6F)
        return false;
    rotation.normalize();
    guess = M4F::Identity();
    guess.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    guess.block<3, 1>(0, 3) = refine_transform.block<3, 1>(0, 3);
    return true;
}
