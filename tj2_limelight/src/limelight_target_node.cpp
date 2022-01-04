#include "limelight_target_node.h"

LimelightTargetNode::LimelightTargetNode(ros::NodeHandle* nodehandle) :
    nh(*nodehandle),
    _image_transport(nh),
    _tf_listener(_tf_buffer)
{
    ros::param::param<double>("~text_marker_size", _text_marker_size, 0.25);
    ros::param::param<string>("~target_frame", _target_frame, "base_link");
    ros::param::param<double>("~marker_persistance_s", _marker_persistance_s, 0.1);
    ros::param::param<double>("~min_contour_area", _min_contour_area, 0.0);
    ros::param::param<double>("~max_contour_area", _max_contour_area, 1000.0);
    ros::param::param<int>("~approx_sync_queue_size", _approx_sync_queue_size, 10);
    _marker_persistance = ros::Duration(_marker_persistance_s);

    ros::param::param<string>("~model_name", _model_name, "googlenet");
    ros::param::param<string>("~model_path", _model_path, "");
    ros::param::param<string>("~prototxt_path", _prototxt_path, "");
    ros::param::param<string>("~class_labels_path", _class_labels_path, "");

    ros::param::param<string>("~input_blob", _input_blob, IMAGENET_DEFAULT_INPUT);
    ros::param::param<string>("~output_blob", _output_blob, IMAGENET_DEFAULT_OUTPUT);

    ros::param::param<double>("~threshold", _threshold, 0.5);

    string key;
    if (!ros::param::search("hsv_lower_bound", key)) {
        THROW_EXCEPTION("Failed to find hsv_lower_bound parameter");
    }
    ROS_DEBUG("Found hsv_lower_bound: %s", key.c_str());
    nh.getParam(key, _hsv_lower_bound_param);

    if (_hsv_lower_bound_param.size() != 3) {
        ROS_ERROR("hsv_upper_bound is size %lu", _hsv_lower_bound_param.size());
        THROW_EXCEPTION("hsv_lower_bound parameter is the incorrect size");
    }

    if (!ros::param::search("hsv_upper_bound", key)) {
        THROW_EXCEPTION("Failed to find hsv_upper_bound parameter");
    }
    ROS_DEBUG("Found hsv_upper_bound: %s", key.c_str());
    nh.getParam(key, _hsv_upper_bound_param);

    if (_hsv_upper_bound_param.size() != 3) {
        ROS_ERROR("hsv_upper_bound is size %lu", _hsv_upper_bound_param.size());
        THROW_EXCEPTION("hsv_upper_bound parameter is the incorrect size");
    }

    hsv_lower_bound = cv::Scalar(
        _hsv_lower_bound_param.at(0),
        _hsv_lower_bound_param.at(1),
        _hsv_lower_bound_param.at(2)
    );
    hsv_upper_bound = cv::Scalar(
        _hsv_upper_bound_param.at(0),
        _hsv_upper_bound_param.at(1),
        _hsv_upper_bound_param.at(2)
    );

    if (!ros::param::search("marker_colors", key)) {
        THROW_EXCEPTION("Failed to find marker_colors parameter");
    }
    ROS_DEBUG("Found marker_colors: %s", key.c_str());
    nh.getParam(key, _marker_colors_param);

    // _marker_colors_param is a map
    if (_marker_colors_param.getType() != XmlRpc::XmlRpcValue::Type::TypeStruct ||
        _marker_colors_param.size() == 0) {
        THROW_EXCEPTION("marker_colors wrong type or size");
    }
    ROS_DEBUG("marker_colors is the correct type");

    for (XmlRpc::XmlRpcValue::iterator it = _marker_colors_param.begin(); it != _marker_colors_param.end(); ++it)
    {
        if (it->second.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            THROW_EXCEPTION("marker_colors element is not a list");
        }
        ROS_DEBUG("\tFound marker_colors label");
        string label = it->first;

        std_msgs::ColorRGBA color;
        color.r = (double)(it->second[0]);
        color.g = (double)(it->second[1]);
        color.b = (double)(it->second[2]);
        color.a = (double)(it->second[3]);
        _marker_colors[label] = color;
        ROS_DEBUG("\t%s: R=%0.2f, G=%0.2f, B=%0.2f, A=%0.2f", label.c_str(), color.r, color.g, color.b, color.a);
    }

    if (_marker_colors.size() == 0) {
        THROW_EXCEPTION("marker_colors has zero length!");
    }

    if (!ros::param::search("z_depth_estimations", key)) {
        THROW_EXCEPTION("Failed to find z_depth_estimations parameter");
    }
    ROS_DEBUG("Found z_depth_estimations: %s", key.c_str());
    nh.getParam(key, _z_depth_estimations);
    if (_z_depth_estimations.size() == 0) {
        THROW_EXCEPTION("z_depth_estimations has zero length!");
    }
    ROS_DEBUG("z_depth_estimations is the correct size");

    _classify_resize = cv::Size(300, 300);

    _input_size = 0;
    _output_size = 0;
    _image_width = 0;
    _image_height = 0;

    _image_input_cpu = NULL;
    _image_input_gpu = NULL;
    _image_output_cpu = NULL;
    _image_output_gpu = NULL;

    _current_targets = new vector<cv::Rect>();

    load_imagenet_model();
    load_labels();

    Log::SetLevel(Log::LevelFromStr("info"));  // set jetson-inference log level

    _pipeline_pub = _image_transport.advertise("pipeline/image_raw", 2);

    _marker_target_pub = nh.advertise<visualization_msgs::MarkerArray>("markers/targets", 10);
    _marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers/pipeline", 10);
    _target_detection_pub = nh.advertise<vision_msgs::Detection2DArray>("detections/targets", 10);
    _detection_array_pub = nh.advertise<vision_msgs::Detection2DArray>("detections/pipeline", 10);

    _color_sub.subscribe(nh, "color/image_raw", 10);
    _depth_sub.subscribe(nh, "depth/image_raw", 10);
    _depth_info_sub.subscribe(nh, "depth/camera_info", 10);
    
    _target_sub = nh.subscribe("targets", 1, &LimelightTargetNode::target_callback, this);

    camera_sync.reset(new CameraSync(CameraApproxSyncPolicy(_approx_sync_queue_size), _color_sub, _depth_sub, _depth_info_sub));
    camera_sync->registerCallback(boost::bind(&LimelightTargetNode::camera_callback, this, _1, _2, _3));
}

void LimelightTargetNode::camera_callback(const ImageConstPtr& color_image, const ImageConstPtr& depth_image, const CameraInfoConstPtr& depth_info)
{
    _camera_model.fromCameraInfo(depth_info);
    
    cv::Mat color_cv_image;
    if (!msg_to_frame(color_image, color_cv_image)) {
        return;
    }
    cv::Mat depth_cv_image;
    if (!msg_to_frame(depth_image, depth_cv_image)) {
        return;
    }
    vector<cv::Rect>* detection_boxes = new vector<cv::Rect>();
    contour_pipeline(color_cv_image, detection_boxes);

    publish_markers_and_detections(&_detection_array_pub, &_marker_pub, detection_boxes, color_image->header, color_cv_image, depth_cv_image);
    publish_markers_and_detections(&_target_detection_pub, &_marker_target_pub, _current_targets, color_image->header, color_cv_image, depth_cv_image);
}

void LimelightTargetNode::publish_markers_and_detections(ros::Publisher* detection_array_pub, ros::Publisher* marker_pub, vector<cv::Rect>* detection_boxes, std_msgs::Header header, cv::Mat color_cv_image, cv::Mat depth_cv_image)
{
    vector<int>* class_indices = new vector<int>();
    detection_pipeline(color_cv_image, detection_boxes, class_indices);
    vision_msgs::Detection2DArray det_array_msg;
    det_array_msg.header = header;
    det_array_msg.header.frame_id = _camera_model.tfFrame();
    for (size_t index = 0; index < detection_boxes->size(); index++) {
        cv::Rect bndbox = detection_boxes->at(index);
        int class_index = class_indices->at(index);
        cv::Point3d dimensions;
        vision_msgs::Detection2D det_msg = target_to_detection(class_index, depth_cv_image, bndbox, dimensions);

        det_msg.header = header;
        det_msg.header.frame_id = _camera_model.tfFrame();

        det_array_msg.detections.push_back(det_msg);
        visualization_msgs::MarkerArray markers = create_markers(_class_descriptions.at(class_index), index, det_msg, dimensions);
        marker_pub->publish(markers);
    }
    
    detection_array_pub->publish(det_array_msg);
}

void LimelightTargetNode::target_callback(const tj2_limelight::LimelightTargetArrayConstPtr& targets)
{
    _current_targets->clear();
    if (targets->targets.size() == 0) {
        return;
    }

    for (size_t index = 0; index < targets->targets.size(); index++) {
        tj2_limelight::LimelightTarget target = targets->targets.at(index);
        if (target.thor <= 0 || target.tvert <= 0) {
            continue;
        }
        cv::Rect bndbox(
            target.tx - target.thor / 2,
            target.ty - target.tvert / 2,
            target.thor, target.tvert
        );
        _current_targets->push_back(bndbox);
    }
}

vision_msgs::Detection2D LimelightTargetNode::target_to_detection(int class_index, cv::Mat depth_cv_image, cv::Rect bndbox, cv::Point3d& dimensions)
{
    vision_msgs::Detection2D det_msg;
    int width = bndbox.width;
    int height = bndbox.height;
    int cx = bndbox.x + width / 2;
    int cy = bndbox.y + height / 2;
    
    // Project limelight target onto depth image
    cv::Point2d target_point;
    target_point.x = cx;
    target_point.y = cy;
    cv::Point3d ray = _camera_model.projectPixelTo3dRay(target_point);

    double z_dist = get_target_z(depth_cv_image, bndbox);
    geometry_msgs::Quaternion quat = vector_to_quat(get_target_normal(depth_cv_image, bndbox));
    
    cv::Point3d center;
    center.x = ray.x * z_dist;
    center.y = ray.y * z_dist;
    center.z = z_dist;

    ROS_DEBUG("center, X: %0.3f, Y: %0.3f, Z: %0.3f", center.x, center.y, center.z);

    cv::Point2d edge_point;
    edge_point.x = cx + width / 2;
    edge_point.y = cy + height / 2;
    ray = _camera_model.projectPixelTo3dRay(edge_point);
    
    cv::Point3d edge;
    edge.x = ray.x * z_dist;
    edge.y = ray.y * z_dist;
    edge.z = z_dist;
    ROS_DEBUG("edge,   X: %0.3f, Y: %0.3f, Z: %0.3f", edge.x, edge.y, edge.z);

    dimensions.x = 2.0 * (edge.x - center.x);
    dimensions.y = 2.0 * (edge.y - center.y);
    if (0 <= class_index && class_index < _class_descriptions.size()) {
        dimensions.z = _z_depth_estimations[_class_descriptions.at(class_index)];
    }
    else {
        dimensions.z = 0.005;
    }

    if (dimensions.x <= 1E-9) {
        dimensions.x = 0.002;
    }
    if (dimensions.y <= 1E-9) {
        dimensions.y = 0.002;
    }
    if (dimensions.z <= 1E-9) {
        dimensions.z = 0.002;
    }

    geometry_msgs::Pose object_pose;
    object_pose.position.x = center.x;
    object_pose.position.y = center.y;
    object_pose.position.z = center.z;
    object_pose.orientation = quat;

    det_msg.bbox.center.x = bndbox.x;
    det_msg.bbox.center.y = bndbox.y;
    det_msg.bbox.size_x = bndbox.width;
    det_msg.bbox.size_y = bndbox.height;

    vision_msgs::ObjectHypothesisWithPose hypothesis;

    hypothesis.id = class_index;
    hypothesis.score = 0.0;
    hypothesis.pose.pose = object_pose;

    det_msg.results.push_back(hypothesis);

    return det_msg;
}


bool LimelightTargetNode::msg_to_frame(const ImageConstPtr msg, cv::Mat& image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg);  // encoding: passthrough
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
    if (msg->width == 0 || msg->height == 0) {
        ROS_ERROR("Image has a zero width dimension!");
        return false;
    }
    image = cv_ptr->image;
    return true;
}

visualization_msgs::MarkerArray LimelightTargetNode::create_markers(string name, int index, vision_msgs::Detection2D det_msg, cv::Point3d dimensions)
{
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker rect_marker = make_marker(name, index, det_msg, dimensions);
    visualization_msgs::Marker text_marker = make_marker(name, index, det_msg, dimensions);

    rect_marker.type = visualization_msgs::Marker::CUBE;
    rect_marker.ns = "cube_" + rect_marker.ns;

    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.text = text_marker.ns;
    text_marker.ns = "text_" + text_marker.ns;
    text_marker.pose.position.y += dimensions.y;  // Z is perpendicular to camera plane. -Y is up in world coordinates here
    // text_marker.scale.x = 0.0;
    // text_marker.scale.y = 0.0;
    text_marker.scale.z = _text_marker_size;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    markers.markers.push_back(rect_marker);
    markers.markers.push_back(text_marker);

    return markers;
}

visualization_msgs::Marker LimelightTargetNode::make_marker(string name, int index, vision_msgs::Detection2D det_msg, cv::Point3d dimensions)
{
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = det_msg.results[0].pose.pose;
    marker.header = det_msg.header;
    marker.lifetime = _marker_persistance;
    marker.ns = name;
    marker.id = index;

    geometry_msgs::Vector3 scale_vector;
    scale_vector.x = dimensions.x;
    scale_vector.y = dimensions.y;
    scale_vector.z = dimensions.z;
    marker.scale = scale_vector;
    marker.color = std_msgs::ColorRGBA();
    marker.color.r = 1.0;
    marker.color.a = 0.5;
    if (_marker_colors.count(name)) {
        marker.color = _marker_colors[name];
    }

    return marker;
}

double LimelightTargetNode::get_target_z(cv::Mat depth_cv_image, cv::Rect target)
{
    cv::Mat rectangle_mask = cv::Mat::zeros(depth_cv_image.rows, depth_cv_image.cols, CV_8UC1);
    cv::rectangle(rectangle_mask, target, cv::Scalar(255, 255, 255), cv::FILLED);

    cv::Mat nonzero_mask = (depth_cv_image > 0.0);
    nonzero_mask.convertTo(nonzero_mask, CV_8U);

    cv::Mat target_mask;
    cv::bitwise_and(rectangle_mask, nonzero_mask, target_mask);

    double z_dist = cv::mean(depth_cv_image, target_mask)[0];  // depth values are in mm
    ROS_DEBUG("z_dist mm: %f", z_dist);

    z_dist /= 1000.0;

    return z_dist;
}


cv::Vec3f LimelightTargetNode::get_target_normal(cv::Mat depth_cv_image, cv::Rect target)
{
    // from: https://stackoverflow.com/questions/34644101/calculate-surface-normals-from-depth-image-using-neighboring-pixels-cross-produc
    cv::Mat depth;
    depth_cv_image.convertTo(depth, CV_32FC3, 1.0 / 0xffff);

    cv::Vec3f normal_sum;
    size_t sum_count;

    for (int x = target.x; x < target.x + target.width; x++)
    {
        for (int y = target.y; y < target.y + target.height; y++)
        {
            float dzdx = (depth.at<float>(x+1, y) - depth.at<float>(x-1, y)) / 2.0;
            float dzdy = (depth.at<float>(x, y+1) - depth.at<float>(x, y-1)) / 2.0;

            if (isnan(dzdx) || isnan(dzdy)) {
                continue;
            }

            cv::Vec3f d(-dzdx, -dzdy, 1.0f);
            cv::Vec3f normal = normalize(d);

            normal_sum += normal;
            sum_count++;
        }
    }
    cv::Vec3f mean_normal(
        normal_sum[0] / sum_count,
        normal_sum[1] / sum_count,
        normal_sum[2] / sum_count
    );
    return mean_normal;
}

float magnitude_vec3f(cv::Vec3f vector)
{
    return sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

geometry_msgs::Quaternion LimelightTargetNode::vector_to_quat(cv::Vec3f vector)
{
    // from: https://math.stackexchange.com/questions/2356649/how-to-find-the-quaternion-representing-the-rotation-between-two-3-d-vectors
    cv::Vec3f null_vector(0.0f, 0.0f, 1.0f);
    cv::Vec3f cross = null_vector.cross(vector);
    float cross_mag = magnitude_vec3f(cross);
    float dot = null_vector[0] * vector[0] + null_vector[1] * vector[1] + null_vector[2] * vector[2];
    float theta = atan(cross_mag / dot);
    cv::Vec3f axis_n;
    axis_n = cross / cross_mag;
    cv::Vec3f quat_xyz = axis_n * sin(theta / 2.0f);

    geometry_msgs::Quaternion quat;
    if (cross_mag == 0.0) {
        quat.w = 1.0;
        quat.x = 0.0;
        quat.y = 0.0;
        quat.z = 0.0;
    }
    else {
        quat.w = cos(theta / 2.0);
        quat.x = quat_xyz[0];
        quat.y = quat_xyz[1];
        quat.z = quat_xyz[2];
    }
    return quat;
}

void LimelightTargetNode::contour_pipeline(cv::Mat frame, vector<cv::Rect>* detection_boxes)
{
    cv::Mat frame_hsv, contour_image;
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv, hsv_lower_bound, hsv_upper_bound, contour_image);
    cv::findContours(contour_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    for (size_t index = 0; index < contours.size(); index++) {
        double area = cv::contourArea(contours.at(index));
        if (area < _min_contour_area || area > _max_contour_area) {
            contours.erase(contours.begin() + index);
            index--;
        }
    }
    for (size_t index = 0; index < contours.size(); index++) {
        cv::Rect bndbox = cv::boundingRect(contours.at(index));
        detection_boxes->push_back(bndbox);
    }
}

void LimelightTargetNode::detection_pipeline(cv::Mat frame, vector<cv::Rect>* detection_boxes, vector<int>* classes)
{
    vector<cv::Rect> rejected_boxes;
    for (size_t index = 0; index < detection_boxes->size(); index++) {
        cv::Rect bndbox = detection_boxes->at(index);
        cv::Mat cropped_frame = frame(bndbox);

        cv::Mat resized_frame;
        cv::resize(cropped_frame, resized_frame, _classify_resize);
        int class_index = classify(resized_frame);
        if (is_bndbox_ok(bndbox) && class_index >= 0) {
            classes->push_back(class_index);
        }
        else {
            detection_boxes->erase(std::next(detection_boxes->begin(), index));
            index--;
            rejected_boxes.push_back(bndbox);
        }
    }

    if (_pipeline_pub.getNumSubscribers() > 0) {
        cv::Mat result_image = frame.clone();
        if (!result_image.empty())
        {
            for (size_t index = 0; index < detection_boxes->size(); index++) {
                cv::rectangle(result_image, detection_boxes->at(index), cv::Scalar(255, 0, 0), 2);
            }
            for (size_t index = 0; index < rejected_boxes.size(); index++) {
                cv::rectangle(result_image, rejected_boxes.at(index), cv::Scalar(0, 0, 255), 2);
            }

            std_msgs::Header header;
            header.frame_id = _camera_model.tfFrame();
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", result_image).toImageMsg();
            _pipeline_pub.publish(msg);
        }
    }
}

bool LimelightTargetNode::is_bndbox_ok(cv::Rect bndbox)
{
    return bndbox.x >= 0 && bndbox.y >= 0 && bndbox.width > 0 && bndbox.height > 0;
}

int LimelightTargetNode::classify(cv::Mat cropped_image)
{
    int width = cropped_image.cols;
    int height = cropped_image.rows;

    // classify the image
    float confidence = 0.0f;

    imageFormat input_format = IMAGE_BGR8;

    // assure memory allocation
	if (!allocate_on_gpu(width, height, input_format)) {
		return -1;
    }

    memcpy(_image_input_cpu, cropped_image.data, imageFormatSize(input_format, width, height));

    // convert image format
    if (CUDA_FAILED(cudaConvertColor(_image_input_gpu, input_format, _image_output_gpu, _internal_format, width, height)))
    {
        ROS_ERROR("failed to convert %ux%u image (from %s to %s) with CUDA", _image_width, _image_height, imageFormatToStr(input_format), imageFormatToStr(_internal_format));
        return -1;
    }

    // ROS_INFO("Calculating image size");
    // int image_size = cropped_image.total() * cropped_image.elemSize();
    // uchar3* image_input = new uchar3[image_size];
    // ROS_INFO("Copying %d to image_input", image_size);
    // memcpy(image_input, cropped_image.data, image_size * sizeof(uchar3));

    // ROS_INFO("Classifying");
    const int img_class = _net->Classify(_image_output_gpu, width, height, &confidence);    
    // ROS_INFO("Image class: %d", img_class);

    // verify the output	
    if (img_class < 0) {            
        return -1;
    }
    if (confidence < _threshold) {
        return -1;
    }
    // ROS_INFO("classified cropped image %f (class=%s)", confidence, _class_descriptions.at(img_class).c_str());

    return img_class;
}

void LimelightTargetNode::free_on_gpu()
{
    if (_image_input_cpu != NULL)
    {
        CUDA(cudaFreeHost(_image_input_cpu));

        _image_input_cpu = NULL;
        _image_input_gpu = NULL;
    }

    if (_image_output_cpu != NULL)
    {
        CUDA(cudaFreeHost(_image_output_cpu));

        _image_output_cpu = NULL;
        _image_output_gpu = NULL;
    }
}

bool LimelightTargetNode::allocate_on_gpu(uint32_t width, uint32_t height, imageFormat inputFormat)
{
    const size_t input_size = imageFormatSize(inputFormat, width, height);
    const size_t output_size = imageFormatSize(_internal_format, width, height);

    if (input_size != _input_size || output_size != _output_size || width != _image_width || height != _image_height)
    {
        free_on_gpu();

        if (!cudaAllocMapped((void**)&_image_input_cpu, (void**)&_image_input_gpu, input_size) ||
            !cudaAllocMapped((void**)&_image_output_cpu, (void**)&_image_output_gpu, output_size))
        {
            ROS_ERROR("failed to allocate memory for %ux%u image conversion", width, height);
            return false;
        }

        ROS_INFO("allocated CUDA memory for %ux%u image conversion", width, height);

        _image_width = width;
        _image_height = height;
        _input_size = input_size;
        _output_size = output_size;		
    }

    return true;
}

void LimelightTargetNode::load_imagenet_model()
{
    /*
     * load image recognition network
     */
    ROS_DEBUG("Loading imagenet model.");
    if (_model_path.size() > 0)
    {
        ROS_DEBUG("Loading model from %s", _model_path.c_str());
        // create network using custom model paths
        _net = imageNet::Create(_prototxt_path.c_str(), _model_path.c_str(),
                            NULL, _class_labels_path.c_str(),
                            _input_blob.c_str(), _output_blob.c_str());
    }
    else
    {
        ROS_DEBUG("Loading model from built-in set");
        // determine which built-in model was requested
        imageNet::NetworkType model = imageNet::NetworkTypeFromStr(_model_name.c_str());

        if (model == imageNet::CUSTOM)
        {
            ROS_ERROR("invalid built-in pretrained model name '%s', defaulting to googlenet", _model_name.c_str());
            model = imageNet::GOOGLENET;
        }

        // create network using the built-in model
        _net = imageNet::Create(model);
    }
}


void LimelightTargetNode::load_labels()
{
    /*
     * create the class labels parameter vector
     */
    std::hash<std::string> model_hasher;  // hash the model path to avoid collisions on the param server
    std::string model_hash_str = std::string(_net->GetModelPath()) + std::string(_net->GetClassPath());
    const size_t model_hash = model_hasher(model_hash_str);

    ROS_INFO("model hash => %zu", model_hash);
    ROS_INFO("hash string => %s", model_hash_str.c_str());

    // obtain the list of class descriptions
    _num_classes = _net->GetNumClasses();
    for (uint32_t n = 0; n < _num_classes; n++) {
        _class_descriptions.push_back(_net->GetClassDesc(n));
    }

    // create the key on the param server
    std::string class_key = std::string("class_labels_") + std::to_string(model_hash);

    if (!nh.hasParam(class_key)) {
        nh.setParam(class_key, _class_descriptions);
    }
}


int LimelightTargetNode::run()
{
    if (!_net)
    {
        ROS_ERROR("failed to load detectNet model");
        return 0;
    }

    ros::spin();
    
    delete _net;

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_limelight");
    ros::NodeHandle nh;
    LimelightTargetNode node(&nh);
    return node.run();
}
