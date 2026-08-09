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
    if (m_config.refine_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_map_resolution, m_config.refine_map_resolution, m_config.refine_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_tgt);
    }

    if (m_config.rough_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_map_resolution, m_config.rough_map_resolution, m_config.rough_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_tgt);
    }
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
    guess = refine_transform;
    return true;
}
