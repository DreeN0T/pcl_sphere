#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/core.hpp>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cout << "Usage: " << argv[0] << " input.pcd num_clusters\n";
        return 1;
    }

    //load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(argv[1], *cloud) < 0) {
        std::cerr << "Failed to load " << argv[1] << "\n";
        return 1;
    }

    //using RANSAC Sphere detection
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    //debatable but usually good for removing artifacts
    seg.setRadiusLimits(0.01, 1.0);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    std::vector<cv::Vec4f> spheres;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remain = cloud;

    //detect spheres one by one
    while (true) {
        //storing model and inliers
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        //run segmentation
        seg.setInputCloud(cloud_remain);
        seg.segment(*inliers, *coeff);

        //stop when too few points support a sphere
        if (inliers->indices.size() < 50) break;

        //store detected sphere (x,y,z,r)
        spheres.emplace_back(
            coeff->values[0], coeff->values[1],
            coeff->values[2], coeff->values[3]
        );

        //remove sphere points and continue
        extract.setInputCloud(cloud_remain);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>());
        extract.filter(*temp);
        cloud_remain.swap(temp);
    }

    if (spheres.empty()) {
        std::cout << "No spheres found.\n";
        return 0;
    }

    //prepare data for k-means clustering
    int N = std::stoi(argv[2]);
    cv::Mat data((int)spheres.size(), 3, CV_32F);
    for (int i = 0; i < spheres.size(); ++i) {
        data.at<float>(i,0) = spheres[i][0];
        data.at<float>(i,1) = spheres[i][1];
        data.at<float>(i,2) = spheres[i][2];
    }

    cv::Mat labels, centers;
    cv::kmeans(
        data, N, labels,
        cv::TermCriteria(
            cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
            100, 0.01
        ),
        3, cv::KMEANS_PP_CENTERS,
        centers
    );

    // Print clusters and spheres
    for (int c = 0; c < N; ++c) {
        std::cout << "Cluster " << c+1 << ":\n";
        for (int i = 0; i < labels.rows; ++i) {
            if (labels.at<int>(i) == c) {
                auto &s = spheres[i];
                std::cout << "  Sphere " << i << " - center=("
                          << s[0] << ", " << s[1] << ", " << s[2]
                          << ") r=" << s[3] << "\n";
            }
        }
    }
    return 0;
}
