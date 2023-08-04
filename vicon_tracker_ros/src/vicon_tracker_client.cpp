#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <DataStreamClient.h>


/* Data types */
typedef struct {
    std::string child_frame_id;
    std::string parent_frame_id;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr ground_pose_publisher;
} object_data_struct;

/* Function prototypes */
std::string viconDirection2string(const ViconDataStreamSDK::CPP::Direction::Enum viconDirection);


int main(int argc, char * argv[])
{
    /* Node initialization */

    // Initialize ROS
    rclcpp::init(argc, argv);

    // Create the node
    rclcpp::NodeOptions nodeOptions;
    nodeOptions.allow_undeclared_parameters(true);
    nodeOptions.automatically_declare_parameters_from_overrides(true);
    
    rclcpp::Node::SharedPtr pNode = rclcpp::Node::make_shared("vicon_tracker_client", nodeOptions);

    // Retrieve server_address parameter from ROS parameter server
    std::string parameter_name = "server_address";

    std::string server_address;
    if (false == pNode->get_parameter(parameter_name, server_address))
        RCLCPP_INFO(pNode->get_logger(), "Node %s: unable to retrieve parameter %s.", pNode->get_name(), parameter_name.c_str());

    // Retrieve object parameters from ROS parameter server
    std::map<std::string, object_data_struct> object_configuration;
    for (int k=1; k<=500; k++)
    {
        std::string agent_name = "object-"+std::to_string(k);

        // Create a publisher for each object
        std::map<std::string, std::string> object_params;
        if (pNode->get_parameters(agent_name, object_params))
        {
            object_data_struct tmp_object_data;

            // Create object publishers
            std::string topic_pose_name = "/"+object_params["object_name"]+"/pose";
            std::string topic_pose2d_name = "/"+object_params["object_name"]+"/ground_pose";

            tmp_object_data.pose_publisher = pNode->create_publisher<geometry_msgs::msg::Pose>(topic_pose_name.c_str(), 1);
            tmp_object_data.ground_pose_publisher = pNode->create_publisher<geometry_msgs::msg::Pose2D>(topic_pose2d_name.c_str(), 1);

            // Store object data
            tmp_object_data.child_frame_id = object_params["child_frame_id"];
            tmp_object_data.parent_frame_id = object_params["parent_frame_id"];
            object_configuration.insert(std::make_pair(object_params["object_name"], tmp_object_data));
        }
        else
        {
            break;
        }
    }

    // Create a TF2 broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*pNode);


    RCLCPP_INFO(pNode->get_logger(), "Node %s ready to run.", pNode->get_name());

    /* Setting up the communication with Vicon workstation */
    // Create a Vicon client object
    ViconDataStreamSDK::CPP::Client viconClient;

    // Connect to the Vicon server
    while (!viconClient.IsConnected().Connected && rclcpp::ok())
    {
        // If is not connected, connect
        viconClient.SetConnectionTimeout(1000);
        const ViconDataStreamSDK::CPP::Output_Connect connect_result = viconClient.Connect(server_address);

        if (connect_result.Result != ViconDataStreamSDK::CPP::Result::Success)
        {
            switch (connect_result.Result)
            {
                case ViconDataStreamSDK::CPP::Result::ClientAlreadyConnected:
                    RCLCPP_ERROR(pNode->get_logger(), "Connecting to %s ... connect failed (client already connected)", server_address.c_str());
                    break;
                case ViconDataStreamSDK::CPP::Result::InvalidHostName:
                    RCLCPP_ERROR(pNode->get_logger(), "Connecting to %s ... connect failed (invalid host name)", server_address.c_str());
                    break;
                case ViconDataStreamSDK::CPP::Result::ClientConnectionFailed:
                    RCLCPP_ERROR(pNode->get_logger(), "Connecting to %s ... connect failed (client connection failed)", server_address.c_str());
                    break;
                default:
                    RCLCPP_ERROR(pNode->get_logger(), "Connecting to %s ... connect failed (unrecognized error)", server_address.c_str());
                    break;
            }
        }
        else
        {
            RCLCPP_INFO(pNode->get_logger(), "Connecting to %s ... client connected", server_address.c_str());
        }

        sleep(1);
    }

    /* Configure the client */
    // Enable labelled marker data types
    if (viconClient.EnableSegmentData().Result != ViconDataStreamSDK::CPP::Result::Success)
    {
        RCLCPP_ERROR(pNode->get_logger(), "Error: cannot enable segment data streaming");

        // Close connection
        viconClient.DisableSegmentData();
        viconClient.Disconnect();

        // Shutdown the node
        rclcpp::shutdown();

        return -1;
    }
    else
    {
        RCLCPP_INFO(pNode->get_logger(), "Segment data streaming enabled");
    }

    // Check that no other data types are enabled
    if (viconClient.IsMarkerDataEnabled().Enabled)
    {
        RCLCPP_ERROR(pNode->get_logger(), "Labelled marker data stream enabled, but marker data streaming is not supported by this client");
    }
    if (viconClient.IsLightweightSegmentDataEnabled().Enabled)
    {
        RCLCPP_ERROR(pNode->get_logger(), "Lightweight segment data stream enabled, but lightweight segment data is not supported by this client");
    }
    if (viconClient.IsUnlabeledMarkerDataEnabled().Enabled)
    {
        RCLCPP_ERROR(pNode->get_logger(), "Unlabelled marker data stream enabled, but unlabelled markers are not supported by this client");
    }
    if (viconClient.IsMarkerRayDataEnabled().Enabled || viconClient.IsCentroidDataEnabled().Enabled ||
        viconClient.IsGreyscaleDataEnabled().Enabled || viconClient.IsVideoDataEnabled().Enabled || viconClient.IsDebugDataEnabled().Enabled)
    {
        RCLCPP_ERROR(pNode->get_logger(), "Additional marker data/video stream enabled, but they are not supported by this client");
    }
    if (viconClient.IsDeviceDataEnabled().Enabled)
    {
        RCLCPP_ERROR(pNode->get_logger(), "Device data stream enabled, but no device data is available on this system");
    }

    // Set the streaming mode
    if (viconClient.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush).Result != ViconDataStreamSDK::CPP::Result::Success)
    {
        RCLCPP_ERROR(pNode->get_logger(), "Error: cannot set stream mode to server push");

        // Close connection
        viconClient.DisableSegmentData();
        viconClient.Disconnect();

        // Shutdown the node
        rclcpp::shutdown();

        return -1;
    }
    else
    {
        RCLCPP_ERROR(pNode->get_logger(), "Streaming mode set to server push");
    }

    // Check axis mapping
    const ViconDataStreamSDK::CPP::Output_GetAxisMapping axisMapping_result = viconClient.GetAxisMapping();
    RCLCPP_INFO(pNode->get_logger(), "Axis Mapping: X-%s Y-%s Z-%s",
                viconDirection2string(axisMapping_result.XAxis).c_str(), viconDirection2string(axisMapping_result.YAxis).c_str(),
                viconDirection2string(axisMapping_result.ZAxis).c_str());

    // Set client buffer size
    viconClient.SetBufferSize(1);

    /* Receive data from Vicon server */
    while(rclcpp::ok())
    {
        unsigned int numObjects = 0;

        // Get a frame from the server
        if (viconClient.GetFrame().Result != ViconDataStreamSDK::CPP::Result::Success)
        {
            RCLCPP_ERROR(pNode->get_logger(), "Error: cannot get a new frame from the server");
        }
        else {
            ViconDataStreamSDK::CPP::Output_GetSubjectCount subjectCount_result = viconClient.GetSubjectCount();
            if (subjectCount_result.Result != ViconDataStreamSDK::CPP::Result::Success) {
                RCLCPP_ERROR(pNode->get_logger(), "Error: cannot get the number of objects from the frame");
            } else {
                numObjects = subjectCount_result.SubjectCount;
            }
        }

        // Get the pose of each object in the frame and publish it
        for (unsigned int objectIndex = 0; objectIndex < numObjects; objectIndex++) {
            double translation[3];
            double eulerXYZ[3];
            double quaternion[4];
            std::string objectName;

            // Get object data
            bool validObjectPose = true;

            // Get the object name
            objectName = viconClient.GetSubjectName(objectIndex).SubjectName;

            // Get the segment name (required by next calls)
            std::string segmentName = viconClient.GetSegmentName(objectName, 0).SegmentName;

            // Get the global object translation
            ViconDataStreamSDK::CPP::Output_GetSegmentGlobalTranslation objectTranslation_output =
                    viconClient.GetSegmentGlobalTranslation(objectName, segmentName);

            if (objectTranslation_output.Result != ViconDataStreamSDK::CPP::Result::Success)
            {
                validObjectPose = false;
                RCLCPP_ERROR(pNode->get_logger(), "Error: cannot get the translation of object %s from the current frame", objectName.c_str());
            }
            else if (objectTranslation_output.Occluded)
            {
                validObjectPose = false;
            }
            else
            {
                translation[0] = objectTranslation_output.Translation[0]/1000.0;
                translation[1] = objectTranslation_output.Translation[1]/1000.0;
                translation[2] = objectTranslation_output.Translation[2]/1000.0;
            }

            // Get the global object rotation in quaternion co-ordinates
            ViconDataStreamSDK::CPP::Output_GetSegmentGlobalRotationQuaternion objectRotationQuat_output =
                    viconClient.GetSegmentGlobalRotationQuaternion(objectName, segmentName);

            if (objectRotationQuat_output.Result != ViconDataStreamSDK::CPP::Result::Success)
            {
                validObjectPose = false;
                RCLCPP_ERROR(pNode->get_logger(), "Error: cannot get the rotation of object %s from the current frame", objectName.c_str());
            }
            else if (objectRotationQuat_output.Occluded)
            {
                validObjectPose = false;
            }
            else
            {
                quaternion[0] = objectRotationQuat_output.Rotation[0];
                quaternion[1] = objectRotationQuat_output.Rotation[1];
                quaternion[2] = objectRotationQuat_output.Rotation[2];
                quaternion[3] = objectRotationQuat_output.Rotation[3];
            }

            // Get the global object rotation in EulerXYZ co-ordinates
            ViconDataStreamSDK::CPP::Output_GetSegmentGlobalRotationEulerXYZ objectRotationEul_output =
                    viconClient.GetSegmentGlobalRotationEulerXYZ(objectName, segmentName);

            if (objectRotationEul_output.Result != ViconDataStreamSDK::CPP::Result::Success)
            {
                validObjectPose = false;
                RCLCPP_ERROR(pNode->get_logger(), "Error: cannot get the rotation of object %s from the current frame", objectName.c_str());
            }
            else if (objectRotationEul_output.Occluded)
            {
                validObjectPose = false;
            }
            else
            {
                eulerXYZ[0] = objectRotationEul_output.Rotation[0];
                eulerXYZ[1] = objectRotationEul_output.Rotation[1];
                eulerXYZ[2] = objectRotationEul_output.Rotation[2];
            }

            if (!validObjectPose) {
                RCLCPP_ERROR(pNode->get_logger(), "Node %s cannot get object %s pose.", pNode->get_name(), objectName.c_str());
            }
            else
            {
                auto it = object_configuration.find(objectName);
                if (it != object_configuration.end()) {
                    // Publish object pose
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = translation[0];
                    pose.position.y = translation[1];
                    pose.position.z = translation[2];
                    pose.orientation.x = quaternion[0];
                    pose.orientation.y = quaternion[1];
                    pose.orientation.z = quaternion[2];
                    pose.orientation.w = quaternion[3];

                    it->second.pose_publisher->publish(pose);

                    // Publish object ground_pose
                    geometry_msgs::msg::Pose2D ground_pose;
                    ground_pose.x = translation[0];
                    ground_pose.y = translation[1];
                    ground_pose.theta = eulerXYZ[2];

                    it->second.ground_pose_publisher->publish(ground_pose);

                    // Publish the transformation
                    geometry_msgs::msg::TransformStamped t;

                    t.header.stamp = pNode->get_clock()->now();
                    t.header.frame_id = it->second.parent_frame_id;
                    t.child_frame_id = it->second.child_frame_id;

                    t.transform.translation.x = translation[0];
                    t.transform.translation.y = translation[1];
                    t.transform.translation.z = translation[2];
                    t.transform.rotation.x = quaternion[0];
                    t.transform.rotation.y = quaternion[1];
                    t.transform.rotation.z = quaternion[2];
                    t.transform.rotation.w = quaternion[3];

                    tf_broadcaster->sendTransform(t);
                }
                else {
                    RCLCPP_ERROR(pNode->get_logger(), "Node %s configuration error, data for Vicon object %s is not available", pNode->get_name(), objectName.c_str());

                    // Close connection
                    viconClient.DisableSegmentData();
                    viconClient.Disconnect();

                    // Shutdown the node
                    rclcpp::shutdown();

                    return -1;
                }
            }
        }

        // Spin to serve ROS callbacks/services
        rclcpp::spin_some(pNode);
    }

    RCLCPP_INFO(pNode->get_logger(), "Node %s shutting down.", pNode->get_name());
    
    /* Closing connection to the server and shutting down */
    // Close connection
    viconClient.DisableSegmentData();
    viconClient.Disconnect();

    // Shutdown the node
    rclcpp::shutdown();

    return 0;
}

/* Other useful functions */
std::string viconDirection2string(const ViconDataStreamSDK::CPP::Direction::Enum viconDirection)
{
    switch(viconDirection)
    {
        case ViconDataStreamSDK::CPP::Direction::Forward:
            return "Forward";
        case ViconDataStreamSDK::CPP::Direction::Backward:
            return "Backward";
        case ViconDataStreamSDK::CPP::Direction::Left:
            return "Left";
        case ViconDataStreamSDK::CPP::Direction::Right:
            return "Right";
        case ViconDataStreamSDK::CPP::Direction::Up:
            return "Up";
        case ViconDataStreamSDK::CPP::Direction::Down:
            return "Down";
        default:
            return "Unknown";
    }
}
