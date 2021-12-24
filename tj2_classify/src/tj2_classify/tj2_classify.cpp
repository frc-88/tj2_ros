#include <tj2_detectnet/tj2_detectnet.h>


TJ2Classify::TJ2Classify(ros::NodeHandle* nodehandle) :
    nh(*nodehandle),
    _image_transport(nh),
    tfListener(tfBuffer)
{
    ros::param::param<string>("~model_name", _model_name, "googlenet");
    ros::param::param<string>("~model_path", _model_path, "");
    ros::param::param<string>("~prototxt_path", _prototxt_path, "");
    ros::param::param<string>("~class_labels_path", _class_labels_path, "");

    ros::param::param<string>("~input_blob", _input_blob, IMAGENET_DEFAULT_INPUT);
    ros::param::param<string>("~output_blob", _output_blob, IMAGENET_DEFAULT_OUTPUT);

    ros::param::param<string>("~color_topic", _color_topic, "color/image_raw");
    ros::param::param<string>("~color_info_topic", _color_info_topic, "color/camera_info");
    ros::param::param<string>("~raw_detection_topic", _raw_detection_topic, "detections/array");
    ros::param::param<int>("~bounding_box_border_px", _bounding_box_border_px, 30);
    ros::param::param<double>("~marker_persistance_s", _marker_persistance_s, 0.5);
    ros::param::param<double>("~threshold", _threshold, 0.5);

    ros::param::param<bool>("~publish_with_frame", _publish_with_frame, true);
    ros::param::param<string>("~target_frame", _target_frame, "base_link");

    ros::param::param<bool>("~always_publish_overlay", _always_publish_overlay, false);

    _marker_persistance = ros::Duration(_marker_persistance_s);

    string key;
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

    if (!ros::param::search("detectnet_z_depth_estimations", key)) {
        THROW_EXCEPTION("Failed to find detectnet_z_depth_estimations parameter");
    }
    ROS_DEBUG("Found detectnet_z_depth_estimations: %s", key.c_str());
    nh.getParam(key, _z_depth_estimations);
    if (_z_depth_estimations.size() == 0) {
        THROW_EXCEPTION("detectnet_z_depth_estimations has zero length!");
    }
    ROS_DEBUG("detectnet_z_depth_estimations is the correct size");

    load_imagenet_model();
    load_labels();

    reset_label_counter();

    // image converter objects
    _input_cvt = new imageConverter();
    _overlay_cvt = new imageConverter();

    // Publishers
    _detection_pub = nh.advertise<vision_msgs::Detection2DArray>("detections/filtered", 25);
    _marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 25);
    _overlay_pub = _image_transport.advertise("overlay/image_raw", 2);
    _overlay_info_pub = nh.advertise<sensor_msgs::CameraInfo>("overlay/camera_info", 2);

    // Subscribers
    color_sub.subscribe(nh, _color_topic, 10);
    color_info_sub.subscribe(nh, _color_info_topic, 10);
    raw_detection_sub.subscribe(nh, _raw_detection_topic, 10);

    sync.reset(new Sync(ApproxSyncPolicy(10), color_sub, color_info_sub, raw_detection_sub));
    sync->registerCallback(boost::bind(&TJ2Classify::raw_detection_callback, this, _1, _2, _3));

    ROS_INFO("tj2_classify init done");
}


void TJ2Classify::load_imagenet_model()
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
        detectNet::NetworkType model = detectNet::NetworkTypeFromStr(_model_name.c_str());

        if (model == detectNet::CUSTOM)
        {
            ROS_ERROR("invalid built-in pretrained model name '%s', defaulting to googlenet", _model_name.c_str());
            model = imageNet::GOOGLENET;
        }

        // create network using the built-in model
        _net = imageNet::Create(model);
    }
}


void TJ2Classify::load_labels()
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


int TJ2Classify::run()
{
    if (!_net)
    {
        ROS_ERROR("failed to load detectNet model");
        return 0;
    }

    if (!_input_cvt || !_overlay_cvt)
    {
        ROS_ERROR("failed to create imageConverter objects");
        return 0;
    }
    ros::spin();

    // free resources
    delete _net;
    delete _input_cvt;
    delete _overlay_cvt;

    return 0;
}

//
// Sub callbacks
//

void TJ2Classify::raw_detection_callback(
    const ImageConstPtr& color_image, const CameraInfoConstPtr& color_info,
    const vision_msgs::Detection2DArrayConstPtr& raw_detection)
{
    ros::Time t0 = ros::Time::now();

    vision_msgs::Detection2DArray filtered_dets;
    filtered_dets.header = raw_detection->header;

    cv::Mat cv_image;
    msg_to_frame(color_image, cv_image);

    for (size_t raw_index = 0; raw_index < raw_detection->detections.size(); raw_index++)
    {
        vision_msgs::BoundingBox2D bbox = raw_detection->detections.at(raw_index).bbox
        int x0 = (int)(bbox.center.x - bbox.size_x / 2);
        int y0 = (int)(bbox.center.y - bbox.size_y / 2);
        int width = (int)(bbox.size_x);
        int height = (int)(bbox.size_y);
        cv::Rect detect_roi(x0, y0, width, height);
        cv::Mat cropped_image = cv_image(detect_roi);

        // convert the image to reside on GPU
        if (!input_cvt || !input_cvt->Convert(cropped_image))
        {
            ROS_INFO("failed to convert %ux%u image", width, height);
            return;
        }

    	// classify the image
        float confidence = 0.0f;

        // was _input_cvt->ImageGPU(). Is now _input_cvt->ImageCPU().
        // Using the GPU stored image added ~0.03s of latency on Jetson Nano for some reason...
        const int img_class = net->Classify(input_cvt->ImageCPU(), input_cvt->GetWidth(), input_cvt->GetHeight(), &confidence);    


        // verify the output	
        if (img_class < 0) {            
            continue;
        }
        if (confidence < _threshold) {
            continue;
        }
        ROS_INFO("classified image #%lu, %f %s (class=%i)", raw_index, confidence, _net->GetClassDesc(img_class), img_class);

        vision_msgs::Detection2D det = raw_detection->detections.at(raw_index);
        det.results.at(0).id = img_class;
        det.results.at(0).score = confidence;

        filtered_dets.detections.push_back(det);
    }
}

bool TJ2Classify::msg_to_frame(const ImageConstPtr msg, cv::Mat& image)
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


void TJ2Classify::reset_label_counter()
{
    for (size_t i = 0; i < _class_descriptions.size(); i++) {
        _label_counter[_class_descriptions[i]] = 0;
    }
}
