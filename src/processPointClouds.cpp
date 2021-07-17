// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    std::cout<<"Begining filtering...";
    // Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT> ());
  
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*cloudFiltered);
    
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT> ());
  
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);
  
    std::vector<int> indices;
  
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f ( -1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f ( 2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);
  
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int point : indices)
      inliers->indices.push_back(point);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloudRegion);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    // Time clound separation process
    auto startTime = std::chrono::steady_clock::now();
    std::cout<<"Begining cloud seperation...";
	typename pcl::PointCloud<PointT>::Ptr ground_plane (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT> ());
    for (int index : inliers->indices){ ground_plane->points.push_back(cloud->points[index]); }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacles);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, ground_plane);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "separation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    std::cout<<"Begining segmentation...";
	pcl::PointIndices::Ptr inliers;
    // Fill in this function to find inliers for the cloud.
	/* // PCL Method
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0){
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    */
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = Ransac(cloud, maxIterations, distanceThreshold);
  	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{   
    auto startTime = std::chrono::steady_clock::now();
    std::cout<<"\nBegining RANSAC...";
	
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
	
	// Fill in this function
	while(maxIterations--){
		//select 3 points randomly
        std::unordered_set<int> inliers;
        while (inliers.size() < 3){ inliers.insert(rand()%cloud->points.size()); }
        // Get the points that define the two vectors and their coordinate values
      	auto itr = inliers.begin();
      	float x1, x2, x3, y1, y2, y3, z1, z2, z3, a, b, c, d;
		x1=cloud->points[*itr].x;
		y1=cloud->points[*itr].y;
		z1=cloud->points[*itr].z;
		itr++;
		x2=cloud->points[*itr].x;
		y2=cloud->points[*itr].y;
		z2=cloud->points[*itr].z;
		itr++;
		x3=cloud->points[*itr].x;
		y3=cloud->points[*itr].y;
		z3=cloud->points[*itr].z;
      	// Define the plane coefficients
		a = ((y2-y1)*(z3-z1)-(z2-z1)*(y3-y1));
		b = ((z2-z1)*(x3-x1)-(x2-x1)*(z3-z1));
		c = ((x2-x1)*(y3-y1)-(y2-y1)*(x3-x1));
		d = (-(a*x1+b*y1+c*z1));
        //number of points within distanceTol to line
		for(int id=0; id < cloud->points.size();id++)
        {
            if (inliers.count(id) > 0)
                continue;
			const PointT pt = cloud->points[id];
			float x4 = pt.x;
			float y4 = pt.y;
			float z4 = pt.z;
			float dist = fabs(a*x4 + b*y4 + c*z4 + d)/sqrt(a*a + b*b + c*c);
           	if (dist <= distanceTol){ inliers.insert(id); }
	   }
       if (inliers.size() > inliersResult.size()) { inliersResult = inliers; }
	}
    typename pcl::PointCloud<PointT>::Ptr inliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr outliers(new pcl::PointCloud<PointT>());
    if(!inliersResult.empty())
    {
        for(int i=0; i<cloud->points.size(); i++)
        {
            const PointT point_(cloud->points.at(i));
            if(inliersResult.count(i))
            {
                inliers->points.push_back(point_);
            }
            else
            {
                outliers->points.push_back(point_);
            }
        }
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(outliers, inliers);
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::cout<<"Begining clustering...";
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    // Fill in the function to perform euclidean clustering to group detected obstacles
	std::vector<std::vector<float>> points;
    for(PointT &p : cloud->points){ points.push_back({p.x, p.y, p.z}); }
    
    KdTree* tree = new KdTree;
  	tree->dimension = 3;
    for(int i=0; i<points.size(); i++)
    { 
        tree->insert(points[i], i); 
    }

    
    const std::vector<std::vector<int>> cluster_indices = EuclideanCluster(points, tree, clusterTolerance, minSize, maxSize);
    
  	for(std::vector<int> cluster : cluster_indices)
    {
      if(cluster.size() < minSize || cluster.size() > maxSize)
      {
          continue;
      }
      typename pcl::PointCloud<PointT>::Ptr cluster_coud(new typename pcl::PointCloud<PointT>);
      for (std::vector<int>::const_iterator itr = cluster.begin (); itr != cluster.end (); itr++)
      {
        cluster_coud->points.push_back (cloud->points[*itr]);
      }
      cluster_coud->width = cluster_coud->points.size();
      cluster_coud->height = 1;
      cluster_coud->is_dense = true;
      clusters.push_back(cluster_coud);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::EuclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
  // Euclidean clustering algorithm
  	std::vector<std::vector<int>> cluster_indices; // list of list of indices for a given cluster
  	std::vector<bool> is_processed(points.size(), false);
    auto startTime = std::chrono::steady_clock::now();
    std::cout<<"\tStarting clustering loop...\n";
    /*
  	for(int i=0; i<points.size(); i++)
    {
      if(~is_processed[i]){ 
      std::vector<int> cluster;
      clusterer(i, points, cluster, is_processed, tree, clusterTolerance);
      int size = cluster.size();
      if( size >= minSize && size <= maxSize) { cluster_indices.push_back(cluster); }
      }
    }
    */
    
    int i = 0;
    while(i<points.size())
    {
        if(is_processed[i])
        {
            i++;
            continue;
        }
        std::vector<int> cluster;
        clusterer(i, points, cluster, is_processed, tree, distanceTol);
        int size = cluster.size();
        if( size >= minSize && size <= maxSize) { cluster_indices.push_back(cluster); }
        i++;
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout<<"clustering loop took "<< elapsedTime.count() << " milliseconds\n";
  return cluster_indices;
}
template<typename PointT>
void ProcessPointClouds<PointT>::clusterer(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	if(processed[indice]){ return; }
    processed[indice]=true;
	cluster.push_back(indice);
  	auto start_time = std::chrono::steady_clock::now();
 	//std::cout<<"Searching for neighbors...";
	std::vector<int> nearby= tree->search(points[indice],distanceTol);
  	//auto end_time = std::chrono::steady_clock::now();
  	//auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    //std::cout<<" took "<< elapsedTime.count() << " milliseconds\n";
    //auto start_time = std::chrono::steady_clock::now();
    //std::cout<<"\tGoing through nearby neighbors...";
	for(int id : nearby) { clusterer(id, points, cluster, processed, tree, distanceTol); }
    //auto endTime = std::chrono::steady_clock::now();
    //auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - start_time);
    //std::cout << "neighbors loop took " << elapsedTime.count() << " milliseconds and found "<< std::endl;

}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}